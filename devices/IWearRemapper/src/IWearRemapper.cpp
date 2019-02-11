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
#include "thrift/WearableMetadataService.h"

#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>
#include <yarp/os/TypedReaderCallback.h>

#include <algorithm>
#include <mutex>
#include <utility>

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
    bool useRPC = true;

    mutable std::recursive_mutex mutex;

    msg::WearableData wearableData;
    yarp::os::BufferedPort<msg::WearableData> outputPortWearData;
    std::vector<std::unique_ptr<yarp::os::BufferedPort<msg::WearableData>>> inputPortsWearData;

    std::map<wearable::sensor::SensorName, size_t> sensorNameToIndex;

    // Sensors stored for exposing wearable::IWear
    std::map<std::string, std::shared_ptr<sensor::impl::Accelerometer>> accelerometers;
    std::map<std::string, std::shared_ptr<sensor::impl::EmgSensor>> emgSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::Force3DSensor>> force3DSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::ForceTorque6DSensor>> forceTorque6DSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::FreeBodyAccelerationSensor>>
        freeBodyAccelerationSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::Gyroscope>> gyroscopes;
    std::map<std::string, std::shared_ptr<sensor::impl::Magnetometer>> magnetometers;
    std::map<std::string, std::shared_ptr<sensor::impl::OrientationSensor>> orientationSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::PoseSensor>> poseSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::PositionSensor>> positionSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::SkinSensor>> skinSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::TemperatureSensor>> temperatureSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::Torque3DSensor>> torque3DSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::VirtualLinkKinSensor>>
        virtualLinkKinSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::VirtualSphericalJointKinSensor>>
        virtualSphericalJointKinSensors;

    // Maps storing sensor names read from RPC
    std::map<wearable::sensor::SensorType, std::vector<std::string>> rpcSensorNames;

    bool validSensorName(const sensor::SensorName& name, const wearable::sensor::SensorType type);

    template <typename SensorInterface, typename SensorImpl>
    SensorPtr<const SensorInterface>
    getSensor(const sensor::SensorName name,
              const sensor::SensorType type,
              std::map<std::string, SensorPtr<SensorImpl>>& storage);
};

const std::map<wearable::msg::SensorType, wearable::sensor::SensorType> MapSensorType = {
    {wearable::msg::SensorType::ACCELEROMETER, wearable::sensor::SensorType::Accelerometer},
    {wearable::msg::SensorType::EMG_SENSOR, wearable::sensor::SensorType::EmgSensor},
    {wearable::msg::SensorType::FORCE_3D_SENSOR, wearable::sensor::SensorType::Force3DSensor},
    {wearable::msg::SensorType::FORCE_TORQUE_6D_SENSOR,
     wearable::sensor::SensorType::ForceTorque6DSensor},
    {wearable::msg::SensorType::FREE_BODY_ACCELERATION_SENSOR,
     wearable::sensor::SensorType::FreeBodyAccelerationSensor},
    {wearable::msg::SensorType::GYROSCOPE, wearable::sensor::SensorType::Gyroscope},
    {wearable::msg::SensorType::MAGNETOMETER, wearable::sensor::SensorType::Magnetometer},
    {wearable::msg::SensorType::ORIENTATION_SENSOR,
     wearable::sensor::SensorType::OrientationSensor},
    {wearable::msg::SensorType::POSE_SENSOR, wearable::sensor::SensorType::PoseSensor},
    {wearable::msg::SensorType::POSITION_SENSOR, wearable::sensor::SensorType::PositionSensor},
    {wearable::msg::SensorType::SKIN_SENSOR, wearable::sensor::SensorType::SkinSensor},
    {wearable::msg::SensorType::TEMPERATURE_SENSOR,
     wearable::sensor::SensorType::TemperatureSensor},
    {wearable::msg::SensorType::TORQUE_3D_SENSOR, wearable::sensor::SensorType::Torque3DSensor},
    {wearable::msg::SensorType::VIRTUAL_LINK_KIN_SENSOR,
     wearable::sensor::SensorType::VirtualLinkKinSensor},
    {wearable::msg::SensorType::VIRTUAL_SPHERICAL_JOINT_KIN_SENSOR,
     wearable::sensor::SensorType::VirtualSphericalJointKinSensor}};

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
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    if (!(config.check("wearableDataPorts") && config.find("wearableDataPorts").isList())) {
        yError() << logPrefix << "wearableDataPorts option does not exist or it is not a list";
        return false;
    }
    yarp::os::Bottle* inputDataPortsNamesList = config.find("wearableDataPorts").asList();
    for (unsigned i = 0; i < inputDataPortsNamesList->size(); ++i) {
        if (!inputDataPortsNamesList->get(i).isString()) {
            yError() << logPrefix << "ith entry of wearableDataPorts list is not a string";
            return false;
        }
    }

    // RPC ports
    if (config.check("useRPC")) {
        if (!config.find("useRPC").isBool()) {
            yError() << logPrefix << "useRPC option is not a bool";
        }
        pImpl->useRPC = config.find("useRPC").asBool();
    }

    yarp::os::Bottle* inputRPCPortsNamesList = nullptr;
    if (pImpl->useRPC)
    {
        if (!config.check("wearableRPCPorts"))
        {
            yInfo() << logPrefix << "wearableRPCPorts parameter does not exist. RPC port will not be used";
            pImpl->useRPC = false;
        }
        else
        {
            if (!config.find("wearableRPCPorts").isList()) {
                yError() << logPrefix << "wearableRPCPorts option is not a list";
                return false;
            }
            inputRPCPortsNamesList = config.find("wearableRPCPorts").asList();
            for (unsigned i = 0; i < inputRPCPortsNamesList->size(); ++i) {
                if (!inputRPCPortsNamesList->get(i).isString()) {
                    yError() << logPrefix << "ith entry of wearableRPCPorts list is not a string";
                    return false;
                }
            }
        }
    }

    // Output port
    if (!(config.check("outputPortName") && config.find("outputPortName").isString())) {
        yError() << logPrefix << "outputPortName option does not exist or it is not a string";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    // Get the values of the options
    std::string outputPortName = config.find("outputPortName").asString();

    // Convert list to vector
    std::vector<std::string> inputDataPortsNamesVector;
    for (unsigned i = 0; i < inputDataPortsNamesList->size(); ++i) {
        inputDataPortsNamesVector.emplace_back(inputDataPortsNamesList->get(i).asString());
    }

    // Convert list to vector
    std::vector<std::string> inputRPCPortsNamesVector;
    if (pImpl->useRPC)
    {
        for (unsigned i = 0; i < inputRPCPortsNamesList->size(); ++i) {
            inputRPCPortsNamesVector.emplace_back(inputRPCPortsNamesList->get(i).asString());
        }
    }

    yInfo() << logPrefix << "*** ========================";
    for (unsigned i = 0; i < inputDataPortsNamesVector.size(); ++i) {
        yInfo() << logPrefix << "*** Wearable Data Port" << i + 1 << "  :"
                << inputDataPortsNamesVector[i];
    }
    if (pImpl->useRPC)
    {
        for (unsigned i = 0; i < inputDataPortsNamesVector.size(); ++i) {
            yInfo() << logPrefix << "*** Wearable RPC Port" << i + 1 << "   :"
                    << inputRPCPortsNamesVector[i];
        }
    }
    else
    {
        yInfo() << logPrefix << "*** Wearable RPC Port not configured";
    }
    yInfo() << logPrefix << "*** Output port            :" << outputPortName;
    yInfo() << logPrefix << "*** ========================";

    // Initialize the network
    // TODO: is this required in every DeviceDriver?
    pImpl->network = yarp::os::Network();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << logPrefix << "YARP server wasn't found active.";
        return false;
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
    }

    // ======================
    // CONFIGURE OUTPUT PORTS
    // ======================
    yDebug() << logPrefix << "Configuring output ports";

    if (!pImpl->outputPortWearData.open(outputPortName)) {
        yError() << logPrefix << "Failed to open port " << outputPortName;
        return false;
    }

    // ====================
    // OPEN INPUT RPC PORTS
    // ====================
    if(pImpl->useRPC)
    {
        yDebug() << logPrefix << "Opening input RPC ports";

        if (inputDataPortsNamesVector.size() != inputRPCPortsNamesVector.size()) {
            yError() << logPrefix << "wearableDataPorts and wearableRPCPorts lists should contain "
                     << "the same number of elements.";
            return false;
        }

        using SensorTypeMetadata = std::vector<wearable::msg::WearableSensorMetadata>;
        using WearableMetadata = std::map<wearable::msg::SensorType, SensorTypeMetadata>;

        std::vector<WearableMetadata> metadata;

        // Get metadata from remote rpc ports
        for (const auto& inputRpcPortName : inputRPCPortsNamesVector) {
            // Open a local port for the rpc data
            yarp::os::RpcClient localRpcPort;
            if (!localRpcPort.open("...")) {
                yInfo() << logPrefix << "Failed to open local port";
                return false;
            }

            // Wait the connection with the remote port
            while (true) {
                yarp::os::Network::connect(localRpcPort.getName(), inputRpcPortName);
                if (yarp::os::Network::isConnected(localRpcPort.getName(), inputRpcPortName)) {
                    yInfo() << logPrefix << "Connected with" << inputRpcPortName;
                    break;
                }
                yInfo() << logPrefix << "Waiting connection with" << inputRpcPortName;
                yarp::os::Time::delay(10);
            }

            // Attach the rpc client
            wearable::msg::WearableMetadataService service;
            if (!service.yarp().attachAsClient(localRpcPort)) {
                yError() << logPrefix << "Failed to attach to" << localRpcPort.getName();
                return false;
            }

            // Ask metadata and store them
            metadata.push_back(service.getMetadata());

            // Close the temporary port
            localRpcPort.close();
        }

        // ====================================
        // PROCESS METADATA READ FROM RPC PORTS
        // ====================================
        yDebug() << logPrefix << "Processing RPC metadata";

        auto findStoredSensorName =
            [](const std::string& name,
               std::map<wearable::sensor::SensorType, std::vector<std::string>>& map) -> bool {
            for (const auto& entryPair : map) {
                for (const std::string& sensorName : entryPair.second) {
                    if (sensorName == name) {
                        return true;
                    }
                }
            }
            return false;
        };

        // Store the sensor names checking that there are no name duplicates in the
        // connected implementation of IWear
        for (const WearableMetadata& wearableMetadataFromSingleSource : metadata) {

            for (const auto& mapEntry : wearableMetadataFromSingleSource) {

                const wearable::msg::SensorType& sensorType = mapEntry.first;
                const SensorTypeMetadata& sensorTypeMetadata = mapEntry.second;

                for (const wearable::msg::WearableSensorMetadata& sensorMD : sensorTypeMetadata) {
                    // Check there isn't any sensor name collision
                    if (findStoredSensorName(sensorMD.name, pImpl->rpcSensorNames)) {
                        yError() << logPrefix << "Sensor with name " << sensorMD.name
                                 << " alredy exists.";
                        return false;
                    }
                    // Add the sensor name
                    yInfo() << logPrefix << "Found sensor" << sensorMD.name;
                    pImpl->rpcSensorNames[MapSensorType.at(sensorType)].push_back(sensorMD.name);
                }
            }
        }
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << logPrefix << "Opening input ports";

    for (unsigned i = 0; i < config.find("wearableDataPorts").asList()->size(); ++i) {
        if (!yarp::os::Network::connect(inputDataPortsNamesVector[i],
                                        pImpl->inputPortsWearData.back()->getName())) {
            yError() << logPrefix << "Failed to connect " << inputDataPortsNamesVector[i]
                     << " with " << pImpl->inputPortsWearData.back()->getName();
            return false;
        }
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << logPrefix << "Opened correctly";
    return true;
}

bool IWearRemapper::close()
{
    return true;
}

void IWearRemapper::run()
{
    return;
}

const std::map<msg::SensorStatus, sensor::SensorStatus> MapSensorStatus = {
    {msg::SensorStatus::OK, sensor::SensorStatus::Ok},
    {msg::SensorStatus::ERROR, sensor::SensorStatus::Error},
    {msg::SensorStatus::DATA_OVERFLOW, sensor::SensorStatus::Overflow},
    {msg::SensorStatus::CALIBRATING, sensor::SensorStatus::Calibrating},
    {msg::SensorStatus::TIMEOUT, sensor::SensorStatus::Timeout},
    {msg::SensorStatus::WAITING_FOR_FIRST_READ, sensor::SensorStatus::WaitingForFirstRead},
    {msg::SensorStatus::UNKNOWN, sensor::SensorStatus::Unknown},
};

void IWearRemapper::onRead(msg::WearableData& receivedWearData)
{
    auto& wearableData = pImpl->outputPortWearData.prepare();
    wearableData.producerName = WrapperName;

    for (auto& accelerometersMap : receivedWearData.accelerometers) {
        const auto& inputSensorName = accelerometersMap.first;
        const auto& wearDataInputSensor = accelerometersMap.second;
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.accelerometers[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getAccelerometer(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get Accelerometer" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.emgSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getEmgSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get EmgSensor" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.force3DSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getForce3DSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get Force3DSensor" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.forceTorque6DSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getForceTorque6DSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get ForceTorque6DSensor" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.freeBodyAccelerationSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getFreeBodyAccelerationSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get FreeBodyAccelerationSensor" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.gyroscopes[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getGyroscope(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get Gyroscope" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.magnetometers[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getMagnetometer(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get Magnetometer" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.orientationSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getOrientationSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get OrientationSensor" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.poseSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getPoseSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get PoseSensor" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.positionSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getPositionSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get PositionSensor" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.skinSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        // Create the sensor if it does not exist
        auto isensor = getSkinSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get SkinSensor" << inputSensorName;
            askToStop();
            return;
        }
        const auto* constSensor = static_cast<const sensor::impl::SkinSensor*>(isensor.get());
        auto* sensor = const_cast<sensor::impl::SkinSensor*>(constSensor);
        yWarning() << logPrefix << "SkinSensor is not yet implemented";
        // Copy its data to the buffer used for exposing the IWear interface
        //        pImpl->skinSensors[inputSensorName]->setBuffer(
        //            {wearDataInputSensor.data.x, wearDataInputSensor.data.y,
        //            wearDataInputSensor.data.z});
        //        // Set the status
        //        sensor->setStatus(MapSensorStatus.at(wearDataInputSensor.info.status));
    }

    for (auto& s : receivedWearData.temperatureSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.temperatureSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getTemperatureSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get TemperatureSensor" << inputSensorName;
            askToStop();
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.torque3DSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getTorque3DSensor(inputSensorName);
        if (!isensor) {
            askToStop();
            yError() << logPrefix << "Failed to get Torque3DSensor" << inputSensorName;
            return;
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
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.virtualLinkKinSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getVirtualLinkKinSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get VirtualLinkKinSensor" << inputSensorName;
            askToStop();
            return;
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

    for (auto& s : receivedWearData.virtualSphericalJointKinSensors) {
        const auto& inputSensorName = s.first;
        const auto& wearDataInputSensor = s.second;
        // ==========================
        // FORWARD TO THE OUTPUT PORT
        // ==========================
        auto& wearData = pImpl->outputPortWearData.prepare();
        wearData.virtualSphericalJointKinSensors[inputSensorName] = wearDataInputSensor;
        // ====================
        // EXPOSE THE INTERFACE
        // ====================
        auto isensor = getVirtualSphericalJointKinSensor(inputSensorName);
        if (!isensor) {
            yError() << logPrefix << "Failed to get VirtualSphericalJointKinSensor"
                     << inputSensorName;
            askToStop();
            return;
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

    // Update the timestamp
    pImpl->timestamp.sequenceNumber++;
    pImpl->timestamp.time = yarp::os::Time::now();

    // Write to the output port
    pImpl->outputPortWearData.write();

    // This is used to handle the overall status of IWear
    if (pImpl->firstRun) {
        pImpl->firstRun = false;
    }
}

yarp::os::Stamp IWearRemapper::getLastInputStamp()
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    // Stamp count should be always zero
    return yarp::os::Stamp(0, getTimeStamp().time);
}

WearableName IWearRemapper::getWearableName() const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return WrapperName + wearable::Separator;
}

WearStatus IWearRemapper::getStatus() const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    if (pImpl->firstRun) {
        return WearStatus::WaitingForFirstRead;
    }

    for (const auto& s : getAllSensors()) {
        if (s->getSensorStatus() != sensor::SensorStatus::Ok) {
            yError() << logPrefix << "The status of" << s->getSensorName() << "is not Ok ("
                     << static_cast<int>(s->getSensorStatus()) << ")";
            return WearStatus::Error;
        }
        // TODO: improve handling of the overall status
    }

    return WearStatus::Ok;
}

TimeStamp IWearRemapper::getTimeStamp() const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->timestamp;
}

SensorPtr<const sensor::ISensor> IWearRemapper::getSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    for (const auto& s : getAllSensors()) {
        if (s->getSensorName() == name) {
            return s;
        }
    }
    return nullptr;
}

VectorOfSensorPtr<const sensor::ISensor>
IWearRemapper::getSensors(const sensor::SensorType type) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
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
        case sensor::SensorType::VirtualSphericalJointKinSensor:
            for (const auto& s : pImpl->virtualSphericalJointKinSensors) {
                sensors.push_back(s.second);
            }
            break;
    }

    return sensors;
}

bool IWearRemapper::impl::validSensorName(const sensor::SensorName& name,
                                          const wearable::sensor::SensorType type)
{
    const std::vector<std::string> sensorNames = rpcSensorNames[type];

    const auto it = std::find(sensorNames.begin(), sensorNames.end(), name);
    return (it != sensorNames.end()) ? true : false;
}

template <typename SensorInterface, typename SensorImpl>
SensorPtr<const SensorInterface>
IWearRemapper::impl::getSensor(const sensor::SensorName name,
                               const sensor::SensorType type,
                               std::map<std::string, SensorPtr<SensorImpl>>& storage)
{
    // TODO check if this check on the sensor name is required, and how it is done in case rpc is not available
    if (useRPC && !validSensorName(name, type)) {
        yError() << logPrefix << "Sensor" << name << "not found in the list read from the wrapper";
        return nullptr;
    }

    if (storage.find(name) == storage.end()) {
        const auto newSensor =
            std::make_shared<SensorImpl>(name, wearable::sensor::SensorStatus::Unknown);
        storage.emplace(name, newSensor);
    }

    return storage[name];
}

wearable::SensorPtr<const sensor::IAccelerometer>
IWearRemapper::getAccelerometer(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IAccelerometer, sensor::impl::Accelerometer>(
        name, wearable::sensor::SensorType::Accelerometer, pImpl->accelerometers);
}

wearable::SensorPtr<const sensor::IEmgSensor>
IWearRemapper::getEmgSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IEmgSensor, sensor::impl::EmgSensor>(
        name, sensor::SensorType::EmgSensor, pImpl->emgSensors);
}

wearable::SensorPtr<const sensor::IForce3DSensor>
IWearRemapper::getForce3DSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IForce3DSensor, sensor::impl::Force3DSensor>(
        name, sensor::SensorType::Force3DSensor, pImpl->force3DSensors);
}

wearable::SensorPtr<const sensor::IForceTorque6DSensor>
IWearRemapper::getForceTorque6DSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IForceTorque6DSensor, sensor::impl::ForceTorque6DSensor>(
        name, sensor::SensorType::ForceTorque6DSensor, pImpl->forceTorque6DSensors);
}

wearable::SensorPtr<const sensor::IFreeBodyAccelerationSensor>
IWearRemapper::getFreeBodyAccelerationSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IFreeBodyAccelerationSensor,
                            sensor::impl::FreeBodyAccelerationSensor>(
        name, sensor::SensorType::FreeBodyAccelerationSensor, pImpl->freeBodyAccelerationSensors);
}

wearable::SensorPtr<const sensor::IGyroscope>
IWearRemapper::getGyroscope(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IGyroscope, sensor::impl::Gyroscope>(
        name, sensor::SensorType::Gyroscope, pImpl->gyroscopes);
}

wearable::SensorPtr<const sensor::IMagnetometer>
IWearRemapper::getMagnetometer(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IMagnetometer, sensor::impl::Magnetometer>(
        name, sensor::SensorType::Magnetometer, pImpl->magnetometers);
}

wearable::SensorPtr<const sensor::IOrientationSensor>
IWearRemapper::getOrientationSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IOrientationSensor, sensor::impl::OrientationSensor>(
        name, sensor::SensorType::OrientationSensor, pImpl->orientationSensors);
}

wearable::SensorPtr<const sensor::IPoseSensor>
IWearRemapper::getPoseSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IPoseSensor, sensor::impl::PoseSensor>(
        name, sensor::SensorType::PoseSensor, pImpl->poseSensors);
}

wearable::SensorPtr<const sensor::IPositionSensor>
IWearRemapper::getPositionSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IPositionSensor, sensor::impl::PositionSensor>(
        name, sensor::SensorType::PositionSensor, pImpl->positionSensors);
}

wearable::SensorPtr<const sensor::ISkinSensor>
IWearRemapper::getSkinSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::ISkinSensor, sensor::impl::SkinSensor>(
        name, sensor::SensorType::SkinSensor, pImpl->skinSensors);
}

wearable::SensorPtr<const sensor::ITemperatureSensor>
IWearRemapper::getTemperatureSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::ITemperatureSensor, sensor::impl::TemperatureSensor>(
        name, sensor::SensorType::TemperatureSensor, pImpl->temperatureSensors);
}

wearable::SensorPtr<const sensor::ITorque3DSensor>
IWearRemapper::getTorque3DSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::ITorque3DSensor, sensor::impl::Torque3DSensor>(
        name, sensor::SensorType::Torque3DSensor, pImpl->torque3DSensors);
}

wearable::SensorPtr<const sensor::IVirtualLinkKinSensor>
IWearRemapper::getVirtualLinkKinSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl
        ->getSensor<const sensor::IVirtualLinkKinSensor, sensor::impl::VirtualLinkKinSensor>(
            name, sensor::SensorType::VirtualLinkKinSensor, pImpl->virtualLinkKinSensors);
}

wearable::SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
IWearRemapper::getVirtualSphericalJointKinSensor(const sensor::SensorName name) const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return pImpl->getSensor<const sensor::IVirtualSphericalJointKinSensor,
                            sensor::impl::VirtualSphericalJointKinSensor>(
        name,
        sensor::SensorType::VirtualSphericalJointKinSensor,
        pImpl->virtualSphericalJointKinSensors);
}
