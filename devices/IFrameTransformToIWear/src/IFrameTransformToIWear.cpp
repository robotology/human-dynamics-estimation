// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "IFrameTransformToIWear.h"

#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

const std::string DeviceName = "IFrameTransformToIWear";
const std::string LogPrefix = DeviceName + " :";

using namespace wearable;
using namespace wearable::sensor;
using namespace wearable::devices;

// Class that provides each frame information, and utilites
// to map its data to containers compatible with IWear
class IFrameTransformHandler
{
public:
    IFrameTransformHandler(const std::string& targetFrame,
                           const std::string& rootFrame,
                           yarp::dev::IFrameTransform* frameTransformInterface)
        : m_targetFrame(targetFrame)
        , m_rootFrame(rootFrame)
        , m_frameTransformInterface(frameTransformInterface)
        , m_matrixBuffer(4, 4){};

    wearable::sensor::SensorStatus getStatus()
    {
        if (m_frameTransformInterface->canTransform(m_targetFrame, m_rootFrame)) {
            return SensorStatus::Ok;
        }
        else {
            return SensorStatus::Error;
        }
    }

    bool getTransform(wearable::Quaternion& quaternion, wearable::Vector3& position)
    {

        if (!m_frameTransformInterface->getTransform(m_targetFrame, m_rootFrame, m_matrixBuffer)) {
            return false;
        }

        position[0] = m_matrixBuffer(0, 3);
        position[1] = m_matrixBuffer(1, 3);
        position[2] = m_matrixBuffer(2, 3);

        for (size_t r = 0; r < 3; r++) {
            for (size_t c = 0; c < 3; c++) {
                m_rotationMatrixBuffer[r][c] = m_matrixBuffer(r, c);
            }
        }

        quaternion = wearable::utils::rotationMatrixToQuaternion(m_rotationMatrixBuffer);
        return true;
    }

private:
    yarp::dev::IFrameTransform* m_frameTransformInterface = nullptr;
    std::string m_rootFrame, m_targetFrame;
    yarp::sig::Matrix m_matrixBuffer;
    wearable::Matrix3 m_rotationMatrixBuffer;
};

class IFrameTransformToIWear::Impl
{
public:
    bool firstRun = true;
    mutable std::recursive_mutex mutex;

    TimeStamp timestamp;
    struct
    {
        wearable::WearableName wearableName;
        wearable::sensor::SensorType wearableSensorType;
        std::string rootFrameID;
        std::vector<std::string> frameIDs;
    } options;

    std::unique_ptr<yarp::os::Network> network = nullptr;

    yarp::dev::IFrameTransform* frameTransformInterface = nullptr;

    template <typename T>
    using SensorsMap = std::map<std::string, SensorPtr<T>>;

    // Sensors stored for exposing wearable::IWear
    class PoseSensor;
    SensorsMap<PoseSensor> poseSensorsMap;

    template <typename SensorType>
    bool isSensorAvailable(const std::string& name, const SensorsMap<SensorType>& map)
    {
        if (map.find(name) == map.end()) {
            return false;
        }
        return true;
    }
};

// ================================
// WEARABLE SENSORS IMPLEMENTATIONS
// ================================

class IFrameTransformToIWear::Impl::PoseSensor : public wearable::sensor::IPoseSensor
{
public:
    // ------------------------
    // Constructor / Destructor
    // ------------------------
    PoseSensor(
        IFrameTransformHandler* frameTransformHandler,
        const wearable::sensor::SensorName& aName,
        IFrameTransformToIWear::Impl* impl,
        const wearable::sensor::SensorStatus& aStatus = wearable::sensor::SensorStatus::Unknown)
        : IPoseSensor(aName, aStatus)
        , m_frameTransformHandler(frameTransformHandler)
        , m_frameTransformToIWearImpl(impl)
    {}

    ~PoseSensor() override = default;

    // -------------------------
    // IOrientationSensor interface
    // -------------------------
    bool getPose(Quaternion& orientation, Vector3& position) const override
    {
        std::lock_guard<std::recursive_mutex> lock(m_frameTransformToIWearImpl->mutex);
        return this->m_frameTransformHandler->getTransform(orientation, position);
    }

    // ------------------------
    // Custom utility functions
    // ------------------------
    inline void setStatus(const wearable::sensor::SensorStatus aStatus) { m_status = aStatus; }

private:
    IFrameTransformHandler* m_frameTransformHandler = nullptr;
    IFrameTransformToIWear::Impl* m_frameTransformToIWearImpl = nullptr;
};

// TODO: implement other sensors

// ======================
// IFrameTransformToIWear
// ======================

IFrameTransformToIWear::IFrameTransformToIWear()
    : pImpl{std::make_unique<Impl>()}
{}

// Without this destructor here, the linker complains for
// undefined reference to vtable
IFrameTransformToIWear::~IFrameTransformToIWear() = default;

bool IFrameTransformToIWear::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("wearableName") && config.find("wearableName").isString())) {
        yError() << LogPrefix << "Parameter 'wearableName' missing or invalid";
        return false;
    }

    if (!(config.check("rootFrameID") && config.find("rootFrameID").isString())) {
        yError() << LogPrefix << "Parameter 'rootFrameID' missing or invalid";
        return false;
    }

    if (!(config.check("wearableSensorType") && config.find("wearableSensorType").isString())) {
        yError() << LogPrefix << "Parameter 'wearableSensorType' missing or invalid";
        return false;
    }

    if (!(config.check("frameIDs") && config.find("frameIDs").isList())) {
        yError() << LogPrefix << "Parameter 'frameIDs' missing or invalid";
        return false;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    pImpl->options.wearableName = config.find("wearableName").asString();
    pImpl->options.rootFrameID = config.find("rootFrameID").asString();
    std::string sensorType = config.find("wearableSensorType").asString();
    pImpl->options.wearableSensorType = wearable::sensor::sensorTypeFromString(sensorType);

    auto frameIDsBottle = config.find("frameIDs").asList();
    for (size_t it = 0; it < frameIDsBottle->size(); it++) {
        pImpl->options.frameIDs.push_back(frameIDsBottle->get(it).asString());
    }

    yInfo() << LogPrefix << "*** ====================";
    yInfo() << LogPrefix << "*** Wearable name      :" << pImpl->options.wearableName;
    yInfo() << LogPrefix << "*** Sensor Type        :" << sensorType;
    yInfo() << LogPrefix << "*** Root Frame ID      :" << pImpl->options.rootFrameID;
    yInfo() << LogPrefix << "*** Selected Frame IDs :";
    for (auto ID : pImpl->options.frameIDs) {
        yInfo() << LogPrefix << "***                     " << ID;
    }
    yInfo() << LogPrefix << "*** ====================";

    // =================================
    // CHECK YARP NETWORK INITIALIZATION
    // =================================

    pImpl->network = std::make_unique<yarp::os::Network>();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << LogPrefix << "YARP server wasn't found active.";
        return false;
    }

    return true;
}

bool IFrameTransformToIWear::close()
{
    detach();
    return true;
}

yarp::os::Stamp IFrameTransformToIWear::getLastInputStamp()
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return yarp::os::Stamp(pImpl->timestamp.sequenceNumber, yarp::os::Time::now());
}

bool IFrameTransformToIWear::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (!(poly->view(pImpl->frameTransformInterface) && pImpl->frameTransformInterface)) {
        yError() << LogPrefix << "Failed to view the IFrameTransform interface from the PolyDriver";
        return false;
    }

    // ===========
    // ADD SENSORS
    // ===========

    // sensor prefixes
    auto poseSensPrefix = getWearableName() + sensor::IPoseSensor::getPrefix();
    auto virtualLinkSensPrefix = getWearableName() + sensor::IVirtualLinkKinSensor::getPrefix();

    if (!pImpl->frameTransformInterface->frameExists(pImpl->options.rootFrameID)) {
        yError() << LogPrefix << "No transform found for the root frame ( "
                 << pImpl->options.rootFrameID << " )";
        return false;
    }

    for (auto ID : pImpl->options.frameIDs) {

        if (!pImpl->frameTransformInterface->frameExists(ID)) {
            yError() << LogPrefix << "No transform found for the frame ( " << ID << " )";
            return false;
        }

        // The sensors are initialized as Ok in order to trigger the first data read.
        // If there is any error during the first read, the sensor updates its own status
        // that is then propagated to the global IWear status.
        switch (pImpl->options.wearableSensorType) {
            case wearable::sensor::SensorType::PoseSensor: {
                auto poseSensorName = poseSensPrefix + ID;
                auto poseSensor = std::make_shared<IFrameTransformToIWear::Impl::PoseSensor>(
                    new IFrameTransformHandler(
                        ID, pImpl->options.rootFrameID, pImpl->frameTransformInterface),
                    poseSensorName,
                    pImpl.get(),
                    wearable::sensor::SensorStatus::Ok);
                pImpl->poseSensorsMap.emplace(poseSensorName, poseSensor);
                break;
            }
            default:
                // TODO: implement the remaining sensors
                yError() << LogPrefix << "Selected wearableSensorType is not valid.";
                return false;
        }
    }

    // Notify that the sensor is ready to be used
    pImpl->firstRun = false;

    return true;
}

bool IFrameTransformToIWear::detach()
{
    return true;
}

bool IFrameTransformToIWear::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool IFrameTransformToIWear::detachAll()
{
    return detach();
}

wearable::SensorPtr<const ISensor> IFrameTransformToIWear::getSensor(const SensorName name) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> sensors = getAllSensors();
    for (const auto& s : sensors) {
        if (s->getSensorName() == name) {
            return s;
        }
    }
    yWarning() << LogPrefix << "User specified name <" << name << "> not found.";
    return nullptr;
}

wearable::VectorOfSensorPtr<const ISensor>
IFrameTransformToIWear::getSensors(const SensorType type) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> allSensors;
    switch (type) {
        case wearable::sensor::SensorType::PoseSensor: {
            allSensors.reserve(pImpl->poseSensorsMap.size());
            for (const auto& poseSens : pImpl->poseSensorsMap) {
                allSensors.push_back(static_cast<SensorPtr<sensor::ISensor>>(poseSens.second));
            }
            break;
        }
        default: {
            yWarning() << LogPrefix << "Selected sensor type (" << static_cast<int>(type)
                       << ") is not supported by IFrameTransformToIWear.";
            return {};
        }
    }
    return allSensors;
}

wearable::WearableName IFrameTransformToIWear::getWearableName() const
{
    return pImpl->options.wearableName + wearable::Separator;
}

wearable::WearStatus IFrameTransformToIWear::getStatus() const
{
    // TODO
    return wearable::WearStatus::Ok;
}

wearable::TimeStamp IFrameTransformToIWear::getTimeStamp() const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);

    pImpl->timestamp.sequenceNumber = 0; // Always zero
    pImpl->timestamp.time = yarp::os::Time::now();

    return pImpl->timestamp;
}

wearable::SensorPtr<const wearable::sensor::IAccelerometer>
IFrameTransformToIWear::getAccelerometer(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
IFrameTransformToIWear::getForce3DSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
IFrameTransformToIWear::getForceTorque6DSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IGyroscope>
IFrameTransformToIWear::getGyroscope(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IMagnetometer>
IFrameTransformToIWear::getMagnetometer(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
IFrameTransformToIWear::getOrientationSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
IFrameTransformToIWear::getTemperatureSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
IFrameTransformToIWear::getTorque3DSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IEmgSensor>
IFrameTransformToIWear::getEmgSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
IFrameTransformToIWear::getFreeBodyAccelerationSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IPoseSensor>
IFrameTransformToIWear::getPoseSensor(const wearable::sensor::SensorName name) const
{
    if (!pImpl->isSensorAvailable(name, pImpl->poseSensorsMap)) {
        yError() << LogPrefix << "Invalid sensor name.";
        return nullptr;
    }

    return static_cast<std::shared_ptr<sensor::IPoseSensor>>(
        pImpl->poseSensorsMap.at(static_cast<std::string>(name)));
}

wearable::SensorPtr<const wearable::sensor::IPositionSensor>
IFrameTransformToIWear::getPositionSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ISkinSensor>
IFrameTransformToIWear::getSkinSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
IFrameTransformToIWear::getVirtualLinkKinSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
IFrameTransformToIWear::getVirtualJointKinSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
IFrameTransformToIWear::getVirtualSphericalJointKinSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}
