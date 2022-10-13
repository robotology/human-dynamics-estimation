/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "IWearRemapper.h"
#include "Wearable/IWear/IWear.h"
#include "Wearable/IWear/Sensors/impl/SensorsImpl.h"
#include "thrift/WearableData.h"

#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/TypedReaderCallback.h>

#include <algorithm>
#include <mutex>
#include <utility>
#include <unordered_map>

const std::string WrapperName = "IWearRemapper";
const std::string logPrefix = WrapperName + " :";

using namespace wearable;
using namespace wearable::devices;

// ==============
// IMPL AND UTILS
// ==============

class IWearRemapper::impl
{
public:
    yarp::os::Network network;
    TimeStamp timestamp;
    bool firstRun = true;
    bool terminationCall = false;
    bool inputDataPorts = false;
    
    bool allowDynamicData = true;

    // Flag to wait for first data received
    bool waitForAttachAll = false;

    mutable std::mutex mutex;

    msg::WearableData wearableData;
    std::vector<std::unique_ptr<yarp::os::BufferedPort<msg::WearableData>>> inputPortsWearData;
    std::vector<bool> firstInputReceived; //flag to check that at least a first message from the inputs port was received

    // Sensors stored for exposing wearable::IWear
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::Accelerometer>> accelerometers;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::EmgSensor>> emgSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::Force3DSensor>> force3DSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::ForceTorque6DSensor>> forceTorque6DSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::FreeBodyAccelerationSensor>>
        freeBodyAccelerationSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::Gyroscope>> gyroscopes;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::Magnetometer>> magnetometers;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::OrientationSensor>> orientationSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::PoseSensor>> poseSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::PositionSensor>> positionSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::SkinSensor>> skinSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::TemperatureSensor>> temperatureSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::Torque3DSensor>> torque3DSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::VirtualLinkKinSensor>>
        virtualLinkKinSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::VirtualJointKinSensor>>
        virtualJointKinSensors;
    std::unordered_map<std::string, std::shared_ptr<sensor::impl::VirtualSphericalJointKinSensor>>
        virtualSphericalJointKinSensors;

    bool updateData(msg::WearableData& receivedWearData, bool create);

    template <typename SensorInterface, typename SensorImpl>
    SensorPtr<const SensorInterface>
    getSensor(const sensor::SensorName name,
              const sensor::SensorType type,
              const std::unordered_map<std::string, SensorPtr<SensorImpl>>& storage) const;

    template <typename SensorInterface, typename SensorImpl>
    SensorPtr<const SensorInterface>
    getOrCreateSensor(const sensor::SensorName name,
                    const sensor::SensorType type,
                    std::unordered_map<std::string, SensorPtr<SensorImpl>>& storage,
                    bool create);

    SensorPtr<const sensor::IAccelerometer>
    getAccelerometer(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IEmgSensor>
    getEmgSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IForce3DSensor>
    getForce3DSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IGyroscope>
    getGyroscope(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IMagnetometer>
    getMagnetometer(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IOrientationSensor>
    getOrientationSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IPoseSensor>
    getPoseSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IPositionSensor>
    getPositionSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::ISkinSensor>
    getSkinSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::ITemperatureSensor>
    getTemperatureSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::ITorque3DSensor>
    getTorque3DSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IVirtualJointKinSensor>
    getVirtualJointKinSensor(const sensor::SensorName /*name*/) const;

    SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const sensor::SensorName /*name*/) const;
};

// ==============
// IWEAR REMAPPER
// ==============

IWearRemapper::IWearRemapper()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

IWearRemapper::~IWearRemapper() = default;

bool IWearRemapper::open(yarp::os::Searchable& config)
{
    // =====================
    // CHECK THE INPUT PORTS 
    // =====================

    // wait for attachAll
    if (config.check("waitForAttachAll")) {
        if (!config.find("waitForAttachAll").isBool()) {
            yError() << logPrefix << "waitForAttachAll option is not a bool";
            return false;
        }
        pImpl->waitForAttachAll = config.find("waitForAttachAll").asBool();
    }

    if (!config.check("allowDynamicData") || !config.find("allowDynamicData").isBool() )
    {
        yInfo() << logPrefix << "Cannot find a suitable allowDynamicData parameter, using default value.";
    }
    else
    {
        pImpl->allowDynamicData = config.find("allowDynamicData").asBool();
    }
    yInfo() << logPrefix << "Using allowDynamicData parameter:"<<pImpl->allowDynamicData;
    
    pImpl->inputDataPorts = config.check("wearableDataPorts");

    if (pImpl->inputDataPorts) {

        // Check that wearableDataPorts option is a list
        if (!config.find("wearableDataPorts").isList()) {
            yError() << logPrefix << "wearableDataPorts option is not a list";
            return false;
        }

        yarp::os::Bottle* inputDataPortsNamesList = config.find("wearableDataPorts").asList();

        if (inputDataPortsNamesList->size() == 0) {
            pImpl->inputDataPorts = false; 
        }
        else {
            for (unsigned i = 0; i < inputDataPortsNamesList->size(); ++i) {
                if (!inputDataPortsNamesList->get(i).isString()) {
                    yError() << logPrefix << "ith entry of wearableDataPorts list is not a string";
                    return false;
                }
            }

            // ===============================
            // PARSE THE CONFIGURATION OPTIONS
            // ===============================

            // Convert list to vector
            std::vector<std::string> inputDataPortsNamesVector;
            for (unsigned i = 0; i < inputDataPortsNamesList->size(); ++i) {
                inputDataPortsNamesVector.emplace_back(inputDataPortsNamesList->get(i).asString());
            }

            yInfo() << logPrefix << "*** ========================";
            for (unsigned i = 0; i < inputDataPortsNamesVector.size(); ++i) {
                yInfo() << logPrefix << "*** Wearable Data Port" << i + 1 << "  :"
                        << inputDataPortsNamesVector[i];
            }

            yInfo() << logPrefix << "*** ========================";

            // Carrier optional configuration
            std::string carrier = "";
            if (config.check("carrier")) {
                carrier = config.find("carrier").asString();
            }

            // ==========================
            // CONFIGURE INPUT DATA PORTS
            // ==========================
            yDebug() << logPrefix << "Configuring input data ports";

            for (unsigned i = 0; i < config.find("wearableDataPorts").asList()->size(); ++i) {
                pImpl->inputPortsWearData.emplace_back(new yarp::os::BufferedPort<msg::WearableData>());
                pImpl->inputPortsWearData.back()->useCallback(*this);

                if (!pImpl->inputPortsWearData.back()->open("...")) {
                    yError() << logPrefix << "Failed to open local input port";
                    return false;
                }
                pImpl->firstInputReceived.push_back(false);
            }

            // ================
            // OPEN INPUT PORTS
            // ================
            yDebug() << logPrefix << "Opening input ports";

            for (unsigned i = 0; i < config.find("wearableDataPorts").asList()->size(); ++i) {
                if (!yarp::os::Network::connect(inputDataPortsNamesVector[i],
                                                pImpl->inputPortsWearData[i]->getName(),
                                                carrier)) {
                    yError() << logPrefix << "Failed to connect " << inputDataPortsNamesVector[i]
                            << " with " << pImpl->inputPortsWearData[i]->getName();
                    return false;
                }
            }

            // Initialize the network
            pImpl->network = yarp::os::Network();
            if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
                yError() << logPrefix << "YARP server wasn't found active.";
                return false;
            }

            // If it not necessary to wait for the attachAll start the callbacks
            // We use callbacks on the input ports, the loop is a no-op
            if (!pImpl->waitForAttachAll) {
                start();
            }
        
        }
    }
    

    yDebug() << logPrefix << "Opened correctly";
    return true;
}

void IWearRemapper::threadRelease() {}

bool IWearRemapper::close()
{
    pImpl->terminationCall = true;

    while (isRunning()) {
        stop();
    }

    return true;
}

void IWearRemapper::run()
{
    if (getStatus() == WearStatus::Error)
        askToStop();
    return;
}

const std::unordered_map<msg::SensorStatus, sensor::SensorStatus> MapSensorStatus = {
    {msg::SensorStatus::OK, sensor::SensorStatus::Ok},
    {msg::SensorStatus::ERROR, sensor::SensorStatus::Error},
    {msg::SensorStatus::DATA_OVERFLOW, sensor::SensorStatus::Overflow},
    {msg::SensorStatus::CALIBRATING, sensor::SensorStatus::Calibrating},
    {msg::SensorStatus::TIMEOUT, sensor::SensorStatus::Timeout},
    {msg::SensorStatus::WAITING_FOR_FIRST_READ, sensor::SensorStatus::WaitingForFirstRead},
    {msg::SensorStatus::UNKNOWN, sensor::SensorStatus::Unknown},
};




bool IWearRemapper::impl::updateData(msg::WearableData& receivedWearData, bool create)
{
    for (auto& accelerometersMap : receivedWearData.accelerometers) {
        const std::string& inputSensorName = accelerometersMap.first;
        const auto& wearDataInputSensor = accelerometersMap.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        
        auto isensor = getOrCreateSensor<const sensor::IAccelerometer, sensor::impl::Accelerometer>(
                            inputSensorName, sensor::SensorType::Accelerometer, accelerometers, create);

        if (!isensor) {
            yError() << logPrefix << "Failed to get Accelerometer" << inputSensorName;
            return false;
        }
        const auto* constSensor = static_cast<const sensor::impl::Accelerometer*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::Accelerometer*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(
            {wearDataInputSensor.data.x, wearDataInputSensor.data.y, wearDataInputSensor.data.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.emgSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IEmgSensor, sensor::impl::EmgSensor>(
                        inputSensorName, sensor::SensorType::EmgSensor, emgSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get EmgSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor = static_cast<const sensor::impl::EmgSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::EmgSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(wearDataInputSensor.data.value, wearDataInputSensor.data.normalization);
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.force3DSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IForce3DSensor, sensor::impl::Force3DSensor>(
                            inputSensorName, sensor::SensorType::Force3DSensor, force3DSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get Force3DSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor = static_cast<const sensor::impl::Force3DSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::Force3DSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(
            {wearDataInputSensor.data.x, wearDataInputSensor.data.y, wearDataInputSensor.data.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.forceTorque6DSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IForceTorque6DSensor, sensor::impl::ForceTorque6DSensor>(
                            inputSensorName, sensor::SensorType::ForceTorque6DSensor, forceTorque6DSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get ForceTorque6DSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor =
            static_cast<const sensor::impl::ForceTorque6DSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::ForceTorque6DSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer({wearDataInputSensor.data.force.x,
                           wearDataInputSensor.data.force.y,
                           wearDataInputSensor.data.force.z},
                          {wearDataInputSensor.data.torque.x,
                           wearDataInputSensor.data.torque.y,
                           wearDataInputSensor.data.torque.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.freeBodyAccelerationSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IFreeBodyAccelerationSensor, sensor::impl::FreeBodyAccelerationSensor>(
                        inputSensorName, sensor::SensorType::FreeBodyAccelerationSensor, freeBodyAccelerationSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get FreeBodyAccelerationSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor =
            static_cast<const sensor::impl::FreeBodyAccelerationSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::FreeBodyAccelerationSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(
            {wearDataInputSensor.data.x, wearDataInputSensor.data.y, wearDataInputSensor.data.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.gyroscopes) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IGyroscope, sensor::impl::Gyroscope>(
                        inputSensorName, sensor::SensorType::Gyroscope, gyroscopes, create);

        if (!isensor) {
            yError() << logPrefix << "Failed to get Gyroscope" << inputSensorName;
            return false;
        }
        const auto* constSensor = static_cast<const sensor::impl::Gyroscope*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::Gyroscope*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(
            {wearDataInputSensor.data.x, wearDataInputSensor.data.y, wearDataInputSensor.data.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.magnetometers) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IMagnetometer, sensor::impl::Magnetometer>(
                inputSensorName, sensor::SensorType::Magnetometer, magnetometers, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get Magnetometer" << inputSensorName;
            return false;
        }
        const auto* constSensor = static_cast<const sensor::impl::Magnetometer*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::Magnetometer*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(
            {wearDataInputSensor.data.x, wearDataInputSensor.data.y, wearDataInputSensor.data.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.orientationSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IOrientationSensor, sensor::impl::OrientationSensor>(
                        inputSensorName, sensor::SensorType::OrientationSensor, orientationSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get OrientationSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor =
            static_cast<const sensor::impl::OrientationSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::OrientationSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer({wearDataInputSensor.data.w,
                           wearDataInputSensor.data.x,
                           wearDataInputSensor.data.y,
                           wearDataInputSensor.data.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.poseSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IPoseSensor, sensor::impl::PoseSensor>(
                        inputSensorName, sensor::SensorType::PoseSensor, poseSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get PoseSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor = static_cast<const sensor::impl::PoseSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::PoseSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer({wearDataInputSensor.data.orientation.w,
                           wearDataInputSensor.data.orientation.x,
                           wearDataInputSensor.data.orientation.y,
                           wearDataInputSensor.data.orientation.z},
                          {wearDataInputSensor.data.position.x,
                           wearDataInputSensor.data.position.y,
                           wearDataInputSensor.data.position.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.positionSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IPositionSensor, sensor::impl::PositionSensor>(
                        inputSensorName, sensor::SensorType::PositionSensor, positionSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get PositionSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor = static_cast<const sensor::impl::PositionSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::PositionSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(
            {wearDataInputSensor.data.x, wearDataInputSensor.data.y, wearDataInputSensor.data.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.skinSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        // Create the sensor if it does not exist
        auto isensor = getOrCreateSensor<const sensor::ISkinSensor, sensor::impl::SkinSensor>(
                        inputSensorName, sensor::SensorType::SkinSensor, skinSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get SkinSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor = static_cast<const sensor::impl::SkinSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::SkinSensor*>(constSensor);

        //Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(wearDataInputSensor.data);
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.temperatureSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::ITemperatureSensor, sensor::impl::TemperatureSensor>(
                        inputSensorName, sensor::SensorType::TemperatureSensor, temperatureSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get TemperatureSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor =
            static_cast<const sensor::impl::TemperatureSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::TemperatureSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(wearDataInputSensor.data);
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.torque3DSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::ITorque3DSensor, sensor::impl::Torque3DSensor>(
                        inputSensorName, sensor::SensorType::Torque3DSensor, torque3DSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get Torque3DSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor = static_cast<const sensor::impl::Torque3DSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::Torque3DSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer(
            {wearDataInputSensor.data.x, wearDataInputSensor.data.y, wearDataInputSensor.data.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.virtualLinkKinSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IVirtualLinkKinSensor, sensor::impl::VirtualLinkKinSensor>(
                        inputSensorName, sensor::SensorType::VirtualLinkKinSensor, virtualLinkKinSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get VirtualLinkKinSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor =
            static_cast<const sensor::impl::VirtualLinkKinSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::VirtualLinkKinSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer({wearDataInputSensor.data.linearAcceleration.x,
                           wearDataInputSensor.data.linearAcceleration.y,
                           wearDataInputSensor.data.linearAcceleration.z},
                          {wearDataInputSensor.data.angularAcceleration.x,
                           wearDataInputSensor.data.angularAcceleration.y,
                           wearDataInputSensor.data.angularAcceleration.z},
                          {wearDataInputSensor.data.linearVelocity.x,
                           wearDataInputSensor.data.linearVelocity.y,
                           wearDataInputSensor.data.linearVelocity.z},
                          {wearDataInputSensor.data.angularVelocity.x,
                           wearDataInputSensor.data.angularVelocity.y,
                           wearDataInputSensor.data.angularVelocity.z},
                          {wearDataInputSensor.data.position.x,
                           wearDataInputSensor.data.position.y,
                           wearDataInputSensor.data.position.z},
                          {wearDataInputSensor.data.orientation.w,
                           wearDataInputSensor.data.orientation.x,
                           wearDataInputSensor.data.orientation.y,
                           wearDataInputSensor.data.orientation.z});

        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.virtualJointKinSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IVirtualJointKinSensor, sensor::impl::VirtualJointKinSensor>(
                        inputSensorName, sensor::SensorType::VirtualJointKinSensor, virtualJointKinSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get VirtualJointKinSensor" << inputSensorName;
            return false;
        }
        const auto* constSensor =
            static_cast<const sensor::impl::VirtualJointKinSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::VirtualJointKinSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer({wearDataInputSensor.data.position},
                          {wearDataInputSensor.data.velocity},
                          {wearDataInputSensor.data.acceleration});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.virtualSphericalJointKinSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;

        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrCreateSensor<const sensor::IVirtualSphericalJointKinSensor, sensor::impl::VirtualSphericalJointKinSensor>(
                        inputSensorName, sensor::SensorType::VirtualSphericalJointKinSensor, virtualSphericalJointKinSensors, create);
        if (!isensor) {
            yError() << logPrefix << "Failed to get VirtualSphericalJointKinSensor"
                     << inputSensorName;
            return false;
        }
        const auto* constSensor =
            static_cast<const sensor::impl::VirtualSphericalJointKinSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::VirtualSphericalJointKinSensor*>(constSensor);
        // Copy its data to the buffer used for exposing the IWear interface
        sensor->setBuffer({wearDataInputSensor.data.angle.r,
                           wearDataInputSensor.data.angle.p,
                           wearDataInputSensor.data.angle.y},
                          {wearDataInputSensor.data.velocity.x,
                           wearDataInputSensor.data.velocity.y,
                           wearDataInputSensor.data.velocity.z},
                          {wearDataInputSensor.data.acceleration.x,
                           wearDataInputSensor.data.acceleration.y,
                           wearDataInputSensor.data.acceleration.z});
        // Set the status
        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    return true;
}

void IWearRemapper::onRead(msg::WearableData& wearData, const yarp::os::TypedReader<msg::WearableData>& typedReader)
{
    if (pImpl->terminationCall) {
        return;
    }

    bool dataUpdated = true;
    if(pImpl->firstRun || pImpl->allowDynamicData)
    {
        // locked version
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        dataUpdated = pImpl->updateData(wearData, true);
    }
    else
    {
        // non-locked version
        dataUpdated = pImpl->updateData(wearData, false);
    }

    if(!dataUpdated)
    {
        askToStop();
    }

    // Update the timestamp
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        pImpl->timestamp.sequenceNumber++;
        pImpl->timestamp.time = yarp::os::Time::now();

        // This is used to handle the overall status of IWear
        if (pImpl->firstRun) {
            // check if all ports were read
            bool allRead = true;
            for(int i = 0; i<pImpl->inputPortsWearData.size(); i++)
            {
                if(pImpl->inputPortsWearData[i]->getName()==typedReader.getName())
                {
                    pImpl->firstInputReceived[i] = true;
                }
                else if(!pImpl->firstInputReceived[i])
                {
                    allRead = false;
                }
            }
            
            if(allRead)
            {
                pImpl->firstRun = false;
            }
        }
    }
}

yarp::os::Stamp IWearRemapper::getLastInputStamp()
{
    // Stamp count should be always zero
    return yarp::os::Stamp(0, getTimeStamp().time);
}

WearableName IWearRemapper::getWearableName() const
{
    return WrapperName + wearable::Separator;
}

WearStatus IWearRemapper::getStatus() const
{
    if (pImpl->firstRun) {
        return WearStatus::WaitingForFirstRead;
    }

    // Logic for combining the status of all the sensors.
    // The tricky part is deciding how to handle the mixed case of timeout and overflow.
    // The WaitingingForFirstRead is not considered since the data is supposed not to be streamed.
    // TODO: For now, overflow is stronger.
    WearStatus status = WearStatus::Ok;
    auto sensorsList = getAllSensors();

    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);

        for (const auto& s : sensorsList) {
            switch (s->getSensorStatus()) {
                case sensor::SensorStatus::Overflow:
                    yWarning() << logPrefix << "type (" << static_cast<int>(s->getSensorType())
                            << ") sensor [" << s->getSensorName() << "] status is ("
                            << static_cast<int>(s->getSensorStatus()) << ")";
                    status = WearStatus::Overflow;
                    break;
                case sensor::SensorStatus::Timeout:
                    yWarning() << logPrefix << "type (" << static_cast<int>(s->getSensorType())
                            << ") sensor [" << s->getSensorName() << "] status is ("
                            << static_cast<int>(s->getSensorStatus()) << ")";
                    if (status == WearStatus::Overflow) {
                        break;
                    }
                    status = WearStatus::Timeout;
                    break;
                case sensor::SensorStatus::Ok:
                    // Keep checking other sensors
                    break;
                default:
                    // If even just one sensor is Error, Unknown, or
                    // any other state return error
                    yError() << logPrefix << "type (" << static_cast<int>(s->getSensorType())
                            << ") sensor [" << s->getSensorName() << "] status is ("
                            << static_cast<int>(s->getSensorStatus()) << ")";
                    return WearStatus::Error;
            }
        }
    }

    return status;
}

TimeStamp IWearRemapper::getTimeStamp() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->timestamp;
}

SensorPtr<const sensor::ISensor> IWearRemapper::getSensor(const sensor::SensorName name) const
{
    auto sensorsList = getAllSensors();

    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        for (const auto& s : sensorsList) {
            if (s->getSensorName() == name) {
                return s;
            }
        }
    }

    return nullptr;
}

bool IWearRemapper::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    for(int p=0; p<driverList.size(); p++)
    {
        wearable::IWear* iWear = nullptr;
        if (!driverList[p]->poly->view(iWear)) {
            yError() << logPrefix << "Failed to view the IWear interface from the PolyDriver.";
            return false;
        }

        for (const auto& sensor : iWear->getAccelerometers()) {
            const auto* constSensor = static_cast<const sensor::impl::Accelerometer*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::Accelerometer*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->accelerometers.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getEmgSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::EmgSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::EmgSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->emgSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getForce3DSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::Force3DSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::Force3DSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->force3DSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getForceTorque6DSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::ForceTorque6DSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::ForceTorque6DSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->forceTorque6DSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getFreeBodyAccelerationSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::FreeBodyAccelerationSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::FreeBodyAccelerationSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->freeBodyAccelerationSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getGyroscopes()) {
            const auto* constSensor = static_cast<const sensor::impl::Gyroscope*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::Gyroscope*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->gyroscopes.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getMagnetometers()) {
            const auto* constSensor = static_cast<const sensor::impl::Magnetometer*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::Magnetometer*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->magnetometers.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getOrientationSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::OrientationSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::OrientationSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->orientationSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getPoseSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::PoseSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::PoseSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->poseSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getPositionSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::PositionSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::PositionSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->positionSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getSkinSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::SkinSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::SkinSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->skinSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getTemperatureSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::TemperatureSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::TemperatureSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->temperatureSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getTorque3DSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::Torque3DSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::Torque3DSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->torque3DSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getVirtualLinkKinSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::VirtualLinkKinSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::VirtualLinkKinSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->virtualLinkKinSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getVirtualJointKinSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::VirtualJointKinSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::VirtualJointKinSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->virtualJointKinSensors.emplace(sensor->getSensorName(), newSensor);
        }
        for (const auto& sensor : iWear->getVirtualSphericalJointKinSensors()) {
            const auto* constSensor = static_cast<const sensor::impl::VirtualSphericalJointKinSensor*>(sensor.get());
            auto* newSensor = const_cast<sensor::impl::VirtualSphericalJointKinSensor*>(constSensor);
            newSensor->setStatus(sensor->getSensorStatus());
            pImpl->virtualSphericalJointKinSensors.emplace(sensor->getSensorName(), newSensor);
        }

    }

    // If there are not input ports there is no need to wait for the first data and to start the no-op loop
    if (!pImpl->inputDataPorts) {
        pImpl->firstRun = false;
        return true;
    }
    else {
        // If it is wating for the attach all, the loop can now be started
        if (pImpl->waitForAttachAll) {
            start();
        }
    }

    return true;
}

bool IWearRemapper::detachAll()
{
    return true;
}

VectorOfSensorPtr<const sensor::ISensor>
IWearRemapper::getSensors(const sensor::SensorType type) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    VectorOfSensorPtr<const sensor::ISensor> sensors;

    switch (type) {
        case sensor::SensorType::Accelerometer:
            for (const auto& s : pImpl->accelerometers) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::EmgSensor:
            for (const auto& s : pImpl->emgSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::Force3DSensor:
            for (const auto& s : pImpl->force3DSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::ForceTorque6DSensor:
            for (const auto& s : pImpl->forceTorque6DSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::FreeBodyAccelerationSensor:
            for (const auto& s : pImpl->freeBodyAccelerationSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::Gyroscope:
            for (const auto& s : pImpl->gyroscopes) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::Magnetometer:
            for (const auto& s : pImpl->magnetometers) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::OrientationSensor:
            for (const auto& s : pImpl->orientationSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::PoseSensor:
            for (const auto& s : pImpl->poseSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::PositionSensor:
            for (const auto& s : pImpl->positionSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::SkinSensor:
            for (const auto& s : pImpl->skinSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::TemperatureSensor:
            for (const auto& s : pImpl->temperatureSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::Torque3DSensor:
            for (const auto& s : pImpl->torque3DSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::VirtualLinkKinSensor:
            for (const auto& s : pImpl->virtualLinkKinSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::VirtualJointKinSensor:
            for (const auto& s : pImpl->virtualJointKinSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::VirtualSphericalJointKinSensor:
            for (const auto& s : pImpl->virtualSphericalJointKinSensors) {
                sensors.push_back(s.second);
            }
            break;
        case sensor::SensorType::Invalid:
            yWarning() << logPrefix << "Requested Invalid sensor type";
            break;
    }

    return sensors;
}

template <typename SensorInterface, typename SensorImpl>
SensorPtr<const SensorInterface>
IWearRemapper::impl::getSensor(const sensor::SensorName name,
                               const sensor::SensorType type,
                               const std::unordered_map<std::string, SensorPtr<SensorImpl>>& storage) const
{

    if (storage.find(name) == storage.end()) {
        return nullptr;
    }

    return storage.at(name);
}

template <typename SensorInterface, typename SensorImpl>
SensorPtr<const SensorInterface>
IWearRemapper::impl::getOrCreateSensor(const sensor::SensorName name,
                               const sensor::SensorType type,
                               std::unordered_map<std::string, SensorPtr<SensorImpl>>& storage,
                               bool create)
{

    auto sensor = getSensor<SensorInterface, SensorImpl>(
        name, type, storage);

    if (!sensor && create) {
        const auto newSensor =
            std::make_shared<SensorImpl>(name, wearable::sensor::SensorStatus::Unknown);
        storage.emplace(name, newSensor);
        sensor = storage[name];
    }

    return sensor;
}

wearable::SensorPtr<const sensor::IAccelerometer>
IWearRemapper::impl::getAccelerometer(const sensor::SensorName name) const
{
    return getSensor<const sensor::IAccelerometer, sensor::impl::Accelerometer>(
        name, wearable::sensor::SensorType::Accelerometer, accelerometers);
}

wearable::SensorPtr<const sensor::IEmgSensor>
IWearRemapper::impl::getEmgSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IEmgSensor, sensor::impl::EmgSensor>(
        name, sensor::SensorType::EmgSensor, emgSensors);
}

wearable::SensorPtr<const sensor::IForce3DSensor>
IWearRemapper::impl::getForce3DSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IForce3DSensor, sensor::impl::Force3DSensor>(
        name, sensor::SensorType::Force3DSensor, force3DSensors);
}

wearable::SensorPtr<const sensor::IForceTorque6DSensor>
IWearRemapper::impl::getForceTorque6DSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IForceTorque6DSensor, sensor::impl::ForceTorque6DSensor>(
        name, sensor::SensorType::ForceTorque6DSensor, forceTorque6DSensors);
}

wearable::SensorPtr<const sensor::IFreeBodyAccelerationSensor>
IWearRemapper::impl::getFreeBodyAccelerationSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IFreeBodyAccelerationSensor,
                            sensor::impl::FreeBodyAccelerationSensor>(
        name, sensor::SensorType::FreeBodyAccelerationSensor, freeBodyAccelerationSensors);
}

wearable::SensorPtr<const sensor::IGyroscope>
IWearRemapper::impl::getGyroscope(const sensor::SensorName name) const
{
    return getSensor<const sensor::IGyroscope, sensor::impl::Gyroscope>(
        name, sensor::SensorType::Gyroscope, gyroscopes);
}

wearable::SensorPtr<const sensor::IMagnetometer>
IWearRemapper::impl::getMagnetometer(const sensor::SensorName name) const
{
    return getSensor<const sensor::IMagnetometer, sensor::impl::Magnetometer>(
        name, sensor::SensorType::Magnetometer, magnetometers);
}

wearable::SensorPtr<const sensor::IOrientationSensor>
IWearRemapper::impl::getOrientationSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IOrientationSensor, sensor::impl::OrientationSensor>(
        name, sensor::SensorType::OrientationSensor, orientationSensors);
}

wearable::SensorPtr<const sensor::IPoseSensor>
IWearRemapper::impl::getPoseSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IPoseSensor, sensor::impl::PoseSensor>(
        name, sensor::SensorType::PoseSensor, poseSensors);
}

wearable::SensorPtr<const sensor::IPositionSensor>
IWearRemapper::impl::getPositionSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IPositionSensor, sensor::impl::PositionSensor>(
        name, sensor::SensorType::PositionSensor, positionSensors);
}

wearable::SensorPtr<const sensor::ISkinSensor>
IWearRemapper::impl::getSkinSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::ISkinSensor, sensor::impl::SkinSensor>(
        name, sensor::SensorType::SkinSensor, skinSensors);
}

wearable::SensorPtr<const sensor::ITemperatureSensor>
IWearRemapper::impl::getTemperatureSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::ITemperatureSensor, sensor::impl::TemperatureSensor>(
        name, sensor::SensorType::TemperatureSensor, temperatureSensors);
}

wearable::SensorPtr<const sensor::ITorque3DSensor>
IWearRemapper::impl::getTorque3DSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::ITorque3DSensor, sensor::impl::Torque3DSensor>(
        name, sensor::SensorType::Torque3DSensor, torque3DSensors);
}

wearable::SensorPtr<const sensor::IVirtualLinkKinSensor>
IWearRemapper::impl::getVirtualLinkKinSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IVirtualLinkKinSensor, sensor::impl::VirtualLinkKinSensor>(
            name, sensor::SensorType::VirtualLinkKinSensor, virtualLinkKinSensors);
}

wearable::SensorPtr<const sensor::IVirtualJointKinSensor>
IWearRemapper::impl::getVirtualJointKinSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IVirtualJointKinSensor, sensor::impl::VirtualJointKinSensor>(
            name, sensor::SensorType::VirtualJointKinSensor, virtualJointKinSensors);
}

wearable::SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
IWearRemapper::impl::getVirtualSphericalJointKinSensor(const sensor::SensorName name) const
{
    return getSensor<const sensor::IVirtualSphericalJointKinSensor,
                            sensor::impl::VirtualSphericalJointKinSensor>(
        name,
        sensor::SensorType::VirtualSphericalJointKinSensor,
        virtualSphericalJointKinSensors);
}


//////////// Interface

wearable::SensorPtr<const sensor::IAccelerometer>
IWearRemapper::getAccelerometer(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getAccelerometer(name);
}

wearable::SensorPtr<const sensor::IEmgSensor>
IWearRemapper::getEmgSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getEmgSensor(name);
}

wearable::SensorPtr<const sensor::IForce3DSensor>
IWearRemapper::getForce3DSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getForce3DSensor(name);
}

wearable::SensorPtr<const sensor::IForceTorque6DSensor>
IWearRemapper::getForceTorque6DSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getForceTorque6DSensor(name);
}

wearable::SensorPtr<const sensor::IFreeBodyAccelerationSensor>
IWearRemapper::getFreeBodyAccelerationSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getFreeBodyAccelerationSensor(name);
}

wearable::SensorPtr<const sensor::IGyroscope>
IWearRemapper::getGyroscope(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getGyroscope(name);
}

wearable::SensorPtr<const sensor::IMagnetometer>
IWearRemapper::getMagnetometer(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getMagnetometer(name);
}

wearable::SensorPtr<const sensor::IOrientationSensor>
IWearRemapper::getOrientationSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getOrientationSensor(name);
}

wearable::SensorPtr<const sensor::IPoseSensor>
IWearRemapper::getPoseSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getPoseSensor(name);
}

wearable::SensorPtr<const sensor::IPositionSensor>
IWearRemapper::getPositionSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getPositionSensor(name);
}

wearable::SensorPtr<const sensor::ISkinSensor>
IWearRemapper::getSkinSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getSkinSensor(name);
}

wearable::SensorPtr<const sensor::ITemperatureSensor>
IWearRemapper::getTemperatureSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getTemperatureSensor(name);
}

wearable::SensorPtr<const sensor::ITorque3DSensor>
IWearRemapper::getTorque3DSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getTorque3DSensor(name);
}

wearable::SensorPtr<const sensor::IVirtualLinkKinSensor>
IWearRemapper::getVirtualLinkKinSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getVirtualLinkKinSensor(name);
}

wearable::SensorPtr<const sensor::IVirtualJointKinSensor>
IWearRemapper::getVirtualJointKinSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getVirtualJointKinSensor(name);
}

wearable::SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
IWearRemapper::getVirtualSphericalJointKinSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->getVirtualSphericalJointKinSensor(name);
}
