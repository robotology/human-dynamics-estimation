/**
 * @file HapticGlove.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */


#include <HapticGlove.h>
//#include <SenseGloveHelper.hpp>

#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>

#include <mutex>
#include <string>
#include <stdio.h>
#include <vector>
#include <assert.h>

const std::string DeviceName = "HapticGlove";
const std::string LogPrefix = DeviceName + wearable::Separator;
double period = 0.01;

using namespace wearable;
using namespace wearable::devices;

struct SenseGloveIMUData
{
    double x;
    double y;
    double z;
    double w;
};

class HapticGlove::SenseGloveImpl
{
public:
    mutable std::mutex mutex;
    SenseGloveIMUData gloveData;

    WearableName wearableName;
    std::string portsPrefix;

    TimeStamp timeStamp;

    // Number of sensors
    const int nSensors = 1;

    // Numbe of actuators
    const int nActuators = 0;

    // link Sensor
    std::string linkSensorPrefix;
    const std::string linkSensorName = "palmIMU";
    class SenseGloveVirtualLinkKinSensor;
    SensorPtr<SenseGloveVirtualLinkKinSensor> sensegloveLinkSensor;

    SenseGloveImpl();
    bool run();

};

HapticGlove::SenseGloveImpl::SenseGloveImpl()
{
    wearableName="SenseGlove";
    portsPrefix ="/wearable/SenseGlove";
}

bool HapticGlove::SenseGloveImpl::run()
{
    std::lock_guard<std::mutex> lock(mutex);

    gloveData.x=0;
    gloveData.y=0;
    gloveData.z=0;
    gloveData.w=yarp::os::Time::now();

    return true;
}

HapticGlove::HapticGlove()
    : PeriodicThread(period)
    , pImpl{new SenseGloveImpl()}
{}

// Destructor
HapticGlove::~HapticGlove() = default;

bool HapticGlove::open(yarp::os::Searchable& config)
{
    yInfo()<<LogPrefix<<"HapticGlove::open(yarp::os::Searchable& config)";

    // ==================================
    // Check the configuration parameters
    // ==================================
    // Period of the this device
    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period: " << period << "s";
    }
    else {
        period = config.find("period").asFloat64();
        yInfo() << LogPrefix << "Using the period : " << period << "s";
    }

    // Get port prefix name
    if (!(config.check("portsPrefixName") && config.find("portsPrefixName").isString())) {
        yInfo() << LogPrefix << "Using default port prefix /wearable/senseGlove"; // to check?
    }
    else {
        pImpl->portsPrefix = config.find("portsPrefixName").asString();
        yInfo() << LogPrefix << "Using the ports prefix " << pImpl->portsPrefix;
    }

    if (!(config.check("wearableName") && config.find("wearableName").isString())) {
        yInfo() << LogPrefix << "Using default wearable name SenseGlove";
        pImpl->wearableName = DeviceName;
    }
    else {
        pImpl->wearableName = config.find("wearableName").asString();
        yInfo() << LogPrefix << "Using the wearable name " << pImpl->wearableName;
    }

    // ===================
    // Ports configuration ???
    // ===================
//    if(!pImpl->dataPort.open(pImpl->portsPrefix + ":o")) {
//        yError() << LogPrefix << "Failed to open data port " << pImpl->portsPrefix + ":o";
//        return false;
//    }

    pImpl->linkSensorPrefix =getWearableName() +sensor::IVirtualLinkKinSensor::getPrefix();
    pImpl->sensegloveLinkSensor= SensorPtr<SenseGloveImpl::SenseGloveVirtualLinkKinSensor>{std::make_shared<SenseGloveImpl::SenseGloveVirtualLinkKinSensor>(pImpl.get(),
                                                                                                                                                       pImpl->linkSensorPrefix + pImpl->linkSensorName)};


    // Initialize snese glove data buffer
    pImpl->gloveData.x    = 0.0;
    pImpl->gloveData.y    = 0.0;
    pImpl->gloveData.z    = 0.0;
    pImpl->gloveData.w    = 1.0;

    return true;
}

// ===========================================
// SenseGlove implementation of VirtualLinkKinSensor
// ===========================================

class HapticGlove::SenseGloveImpl::SenseGloveVirtualLinkKinSensor : public sensor::IVirtualLinkKinSensor
{
public:
    SenseGloveVirtualLinkKinSensor( HapticGlove::SenseGloveImpl* gloveImplPtr,
                                    const sensor::SensorName& name = {},
                                    const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
                                  : IVirtualLinkKinSensor(name, status),
                                    m_gloveImpl(gloveImplPtr)
    { }

    ~SenseGloveVirtualLinkKinSensor() override = default;

    bool getLinkAcceleration(Vector3& linear, Vector3& angular) const override
    {
        // we do not handle linear and angular accelerations in the current implementation
        linear.fill(0.0);
        angular.fill(0.0);

        return true;
    }

    bool getLinkPose(Vector3& position, Quaternion& orientation) const override
    {
        // we do not handle position in the current implementation
        position.fill(0.0);

        std::vector<double> gloveImuData; // w, x, y, z
        assert(m_gloveImpl != nullptr);

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);
        orientation = {m_gloveImpl->gloveData.w, m_gloveImpl->gloveData.x, m_gloveImpl->gloveData.y, m_gloveImpl->gloveData.z};
        return true;
    }

    bool getLinkVelocity(Vector3& linear, Vector3& angular) const override
    {
        // we do not handle linear and angular velocity
        angular.fill(0.0);
        linear.fill(0.0);
        return true;
    }

    inline void setStatus(const sensor::SensorStatus& status)
    {
        m_status = status;
    }

private:
    HapticGlove::SenseGloveImpl* m_gloveImpl{nullptr};
};


void HapticGlove::run()
{
    // Get timestamp
    pImpl->timeStamp.time = yarp::os::Time::now();

    // to implement
    pImpl->run();
}

bool HapticGlove::close()
{
    detach();
    return true;
}

bool HapticGlove::attach(yarp::dev::PolyDriver* poly)
{
    yError()<<LogPrefix<<"HapticGlove::attach(yarp::dev::PolyDriver* poly)";

    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is a nullptr";
        return false;
    }

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the period thread.";
        return false;
    }

    yInfo() << LogPrefix << "attach() successful";
    return true;

}

bool HapticGlove::detach()
{
    while(yarp::os::PeriodicThread::isRunning()) {
        yarp::os::PeriodicThread::stop();
    }

    return true;
}

// TO Explain TO check
bool HapticGlove::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached yarp Serial device";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool HapticGlove::detachAll()
{
    return detach();
}

void HapticGlove::threadRelease(){}

// =========================
// IPreciselyTimed interface
// =========================
yarp::os::Stamp HapticGlove::getLastInputStamp()
{
    // Stamp count should be always zero
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return yarp::os::Stamp(0, pImpl->timeStamp.time);
}

// ---------------------------
// Implement Sensors Methods
// ---------------------------

wearable::WearableName HapticGlove::getWearableName() const
{
    return pImpl->wearableName + wearable::Separator;
}

wearable::WearStatus HapticGlove::getStatus() const
{
    wearable::WearStatus status = wearable::WearStatus::Ok;

    for (const auto& s : getAllSensors()) {
        if (s->getSensorStatus() != sensor::SensorStatus::Ok) {
            status = wearable::WearStatus::Error;
            // TO CHECK
            break;
        }
    }
    // Default return status is Ok
    return status;
}

wearable::TimeStamp HapticGlove::getTimeStamp() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return {pImpl->timeStamp.time, 0};
}

wearable::SensorPtr<const wearable::sensor::ISensor>
HapticGlove::getSensor(const wearable::sensor::SensorName name) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> sensors = getAllSensors();
    for (const auto& s : sensors) {
        if (s->getSensorName() == name) {
            return s;
        }
    }
    yWarning() << LogPrefix << "User specified name <" << name << "> not found";
    return nullptr;
}

wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
HapticGlove::getSensors(const wearable::sensor::SensorType aType) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> outVec;
    outVec.reserve(pImpl->nSensors);
    switch (aType) {
    case sensor::SensorType::VirtualLinkKinSensor: {
        outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(pImpl->sensegloveLinkSensor));
        break;
    }
    default: {
        return {};
    }
    }

    return outVec;
}

wearable::ElementPtr<const wearable::actuator::IActuator>
HapticGlove::getActuator(const wearable::actuator::ActuatorName name) const
{
    wearable::VectorOfElementPtr<const wearable::actuator::IActuator> actuators = getAllActuators();

    for (const auto& a : actuators)
    {
        if (a->getActuatorName() == name)
        {
            return a;
        }
    }
    yWarning() << LogPrefix << "User specified actuator name <" << name << "> not found";
    return nullptr;
}

wearable::VectorOfElementPtr<const wearable::actuator::IActuator>
HapticGlove::getActuators(const wearable::actuator::ActuatorType aType) const
{
    wearable::VectorOfElementPtr<const wearable::actuator::IActuator> outVec;
    outVec.reserve(pImpl->nActuators);

    switch (aType) {
        default: {
            return {};
        }
    }
    return outVec;
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
HapticGlove::getVirtualLinkKinSensor(const wearable::sensor::SensorName name) const
{

    if (name == pImpl->linkSensorPrefix + pImpl->linkSensorName) {
        yError() << LogPrefix << "Invalid sensor name " << name;
        return nullptr;
    }

    // Return a shared point to the required sensor
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>&>(
        *pImpl->sensegloveLinkSensor);
}


