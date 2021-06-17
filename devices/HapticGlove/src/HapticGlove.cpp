/**
 * @file HapticGlove.cpp
 * @authors  Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */


#include <HapticGlove.h>
#include <SenseGloveHelper.hpp>

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
#include <map>


const std::string DeviceName = "HapticGlove";
const std::string LogPrefix = DeviceName + wearable::Separator;
double period = 0.01;

using namespace wearable;
using namespace wearable::devices;

struct SenseGloveIMUData
{
    // sensors
    std::vector<double> orientation; // [w, x, y , z]
    std::vector<std::string> humanJointNames;
    std::vector<double> humanJointValues;
    std::map<std::string, std::size_t> humanJointSensorNameIdMap;

    // actuators
    std::vector<double> fingersForceFeedback;
    std::vector<double> fingersVibroTactileFeedback;
    std::vector<double> fingersHapticFeedback; // [fingersForceFeedback, fingersVibroTactileFeedback]

    std::vector<std::string> humanFingerNames;
    std::map<std::string, std::size_t> humanForceFeedbackActuatorNameIdMap;
    std::map<std::string, std::size_t> humanVibroTactileFeedbackActuatorNameIdMap;
    std::map<std::string, std::size_t> humanHapticActuatorNameIdMap; // First part is Force Feedback, second part is Vibrotactile feedback


};

class HapticGlove::SenseGloveImpl
{
public:
    mutable std::mutex mutex;
    SenseGloveIMUData gloveData;

    WearableName wearableName;
    std::string portsPrefix;

    TimeStamp timeStamp;

    const size_t nFingers = 5;

    // Number of sensors
    const int nLinkSensors = 1; // 1 palm imu

    // Numbe of actuators
    const int nActuators = 10; // humanFingerNames.size()*2

    std::unique_ptr<senseGlove::SenseGloveHelper> m_glove; /**< Pointer to the glove object. */

    // link Sensor
    std::string linkSensorPrefix;
    const std::string linkSensorName = "palmIMU";

    class SenseGloveVirtualLinkKinSensor;
    SensorPtr<SenseGloveVirtualLinkKinSensor> sensegloveLinkSensor;

    // joint sensor
    std::string jointSensorPrefix;
    class SenseGloveVirtualJointKinSensor;
    std::vector<SensorPtr<SenseGloveVirtualJointKinSensor>> sensegloveJointSensorVector;

    // haptic actuator
    std::string hapticActuatorPrefix;
    class SenseGloveHapticActuator;
    std::vector<SensorPtr<SenseGloveHapticActuator>> sensegloveHapticActuatorVector;

    bool return_val=false;

    //
    SenseGloveImpl();

    bool run();

    bool configure(yarp::os::Searchable& config);

    bool isAvailable(const std::string& name, const std::map<std::string, std::size_t>& map)
    {
        if (map.find(name) == map.end())
        {
            return false;
        }
        return true;
    }
};

HapticGlove::SenseGloveImpl::SenseGloveImpl()
{
    std::lock_guard<std::mutex> lock(mutex);

    wearableName="SenseGlove";
    portsPrefix ="/wearable/SenseGlove";
    m_glove= std::make_unique<senseGlove::SenseGloveHelper>();
}

bool HapticGlove::SenseGloveImpl::configure(yarp::os::Searchable& config)
{
    bool rightHand;
    if (!(config.check("rightHand") && config.find("rightHand").isBool())) {
        yInfo() << LogPrefix << "Using default hand Sense Glove: Right hand";
        rightHand= true;
    }
    else {
        rightHand = config.find("rightHand").asBool();
        yInfo() << LogPrefix << "Using the right hand: " << rightHand <<"(if false, using left hand)";
    }

    // configure the glove device
    if (!m_glove->configure(config, rightHand))
    {
        yError() << LogPrefix<< "Unable to initialize the sense glove helper device.";
        return false;
    }

    // Initialize snese glove imu data buffer
    gloveData.orientation.resize(4, 0.0);
    if (!m_glove->getGloveIMUData(gloveData.orientation))
    {
        yError() << LogPrefix<< "Unable to get the human palm IMU data.";
        return false;
    }

    // get joint names
    if (!m_glove->getHumanJointNameList(gloveData.humanJointNames))
    {
        yError() << LogPrefix<< "Unable to get the human joint names.";
        return false;
    }

    // get the human joint values
    gloveData.humanJointValues.resize(gloveData.humanJointNames.size(), 0.0);
    if (!m_glove->getHandJointsAngles(gloveData.humanJointValues))
    {
        yError() << LogPrefix<< "Unable to get the human hand joint angles.";
        return false;
    }

    // get finger names
    if (!m_glove->getHumanFingerNameList(gloveData.humanFingerNames))
    {
        yError() << LogPrefix<< "Unable to get the human finger names.";
        return false;
    }

    gloveData.fingersForceFeedback.resize(nFingers, 0.0); // 5 actuators

    gloveData.fingersVibroTactileFeedback.resize(nFingers, 0.0); // 5 actuators

    // [force feedback, vibrotactile feedback] = nActuators
    gloveData.fingersHapticFeedback.resize(nActuators, 0.0);

    return true;
}

bool HapticGlove::SenseGloveImpl::run()
{
    yInfo()<<"SenseGloveImpl::run()";
    std::lock_guard<std::mutex> lock(mutex);

    // sensors
    m_glove->getGloveIMUData(gloveData.orientation);
    m_glove->getHandJointsAngles(gloveData.humanJointValues);

    // actuators
    for(size_t i=0; i < nFingers; i++)
    {
        gloveData.fingersForceFeedback[i]=gloveData.fingersHapticFeedback[i];
        gloveData.fingersVibroTactileFeedback[i]=gloveData.fingersHapticFeedback[ i + nFingers ];
    }
    m_glove->setFingersForceReference(gloveData.fingersForceFeedback);
    m_glove->setBuzzMotorsReference(gloveData.fingersVibroTactileFeedback);

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

    // Configure the implementation class
    if(!pImpl->configure(config))
    {
        yInfo() << LogPrefix << "Cannot configure the implementation class";
        return false;
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

    pImpl->jointSensorPrefix =getWearableName() +sensor::IVirtualJointKinSensor::getPrefix();
    for(size_t i=0; i < pImpl->gloveData.humanJointNames.size();i++)
    {
        pImpl->sensegloveJointSensorVector.push_back(SensorPtr<SenseGloveImpl::SenseGloveVirtualJointKinSensor>{std::make_shared<SenseGloveImpl::SenseGloveVirtualJointKinSensor>(pImpl.get(),
                                                                                                                pImpl->jointSensorPrefix + pImpl->gloveData.humanJointNames[i])});
    }

    pImpl->hapticActuatorPrefix =getWearableName() +actuator::IHaptic::getPrefix();

    for(size_t i=0; i < pImpl->gloveData.humanFingerNames.size();i++)
    {
        pImpl->sensegloveHapticActuatorVector.push_back(ElementPtr<SenseGloveImpl::SenseGloveHapticActuator>{std::make_shared<SenseGloveImpl::SenseGloveHapticActuator>(pImpl.get(),
                                                                                                                pImpl->hapticActuatorPrefix + pImpl->gloveData.humanFingerNames[i] + "::ForceFeedback" )});
    }

    for(size_t i=0; i < pImpl->gloveData.humanFingerNames.size();i++)
    {
        pImpl->sensegloveHapticActuatorVector.push_back(ElementPtr<SenseGloveImpl::SenseGloveHapticActuator>{std::make_shared<SenseGloveImpl::SenseGloveHapticActuator>(pImpl.get(),
                                                                                                                pImpl->hapticActuatorPrefix + pImpl->gloveData.humanFingerNames[i] + "::VibroTactileFeedback" )});
    }

    for(size_t i=0; i<pImpl->gloveData.humanJointNames.size(); i++)
        pImpl->gloveData.humanJointSensorNameIdMap.emplace(std::make_pair(pImpl->jointSensorPrefix + pImpl->gloveData.humanJointNames[i], i));

    for(size_t i=0; i<pImpl->gloveData.humanFingerNames.size(); i++)
        pImpl->gloveData.humanHapticActuatorNameIdMap.emplace(std::make_pair(pImpl->hapticActuatorPrefix + pImpl->gloveData.humanFingerNames[i] + "::ForceFeedback", i));

    for(size_t i=0; i<pImpl->gloveData.humanFingerNames.size(); i++)
        pImpl->gloveData.humanHapticActuatorNameIdMap.emplace(std::make_pair(pImpl->hapticActuatorPrefix + pImpl->gloveData.humanFingerNames[i] + "::VibroTactileFeedback", i));

    yInfo()<<LogPrefix<<"The device is opened successfully.";
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

        assert(m_gloveImpl != nullptr);

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);
        orientation = {m_gloveImpl->gloveData.orientation[0], m_gloveImpl->gloveData.orientation[1],
                       m_gloveImpl->gloveData.orientation[2], m_gloveImpl->gloveData.orientation[3]};
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

// ===========================================
// SenseGlove implementation of VirtualJointKinSensor
// ===========================================


class HapticGlove::SenseGloveImpl::SenseGloveVirtualJointKinSensor : public sensor::IVirtualJointKinSensor
{
public:
    SenseGloveVirtualJointKinSensor( HapticGlove::SenseGloveImpl* gloveImplPtr,
                                    const sensor::SensorName& name = {},
                                    const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
                                  : IVirtualJointKinSensor(name, status),
                                    m_gloveImpl(gloveImplPtr),
                                    m_sensorName(name)
    {    }

    ~SenseGloveVirtualJointKinSensor() override = default;

    bool getJointPosition(double& position) const override
    {
        assert(m_gloveImpl != nullptr);

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        position = m_gloveImpl->gloveData.humanJointValues[m_gloveImpl->gloveData.humanJointSensorNameIdMap[m_sensorName]];

        return true;
    }

    bool getJointVelocity(double& velocity) const override
    {
        assert(m_gloveImpl != nullptr);

        velocity = 0.0;

        return true;
    }

    bool getJointAcceleration(double& acceleration) const override
    {
        assert(m_gloveImpl != nullptr);

        acceleration = 0.0;

        return true;
    }

    inline void setStatus(const sensor::SensorStatus& status)
    {
        m_status = status;
    }

private:
    HapticGlove::SenseGloveImpl* m_gloveImpl{nullptr};
    std::string m_sensorName;
};

// ===========================================
// SenseGlove implementation of  IHaptic actuator
// ===========================================

class HapticGlove::SenseGloveImpl::SenseGloveHapticActuator : public wearable::actuator::IHaptic
{
public:
    SenseGloveHapticActuator( HapticGlove::SenseGloveImpl* gloveImplPtr,
                              const actuator::ActuatorName& name = {},
                              const actuator::ActuatorStatus& status = actuator::ActuatorStatus::Ok)
        :IHaptic(name, status)
        , m_gloveImpl(gloveImplPtr),
          m_actuatorName(name)
    {  }
    ~SenseGloveHapticActuator() override = default;

    bool setHapticCommand(double& value) const override
    {
        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        if ( !m_gloveImpl->isAvailable(m_actuatorName, m_gloveImpl->gloveData.humanHapticActuatorNameIdMap) )
        {
            yError()<<LogPrefix<<"The actuator name ("<<m_actuatorName<<") is not found in the list of actuators.";

            return false;
        }
        m_gloveImpl->gloveData.fingersHapticFeedback[m_gloveImpl->gloveData.humanHapticActuatorNameIdMap[m_actuatorName]]=value;

        return true;
    }

    inline void setStatus(const actuator::ActuatorStatus& status)
    {
        m_status = status;
    }

private:
    HapticGlove::SenseGloveImpl* m_gloveImpl{nullptr};
    std::string m_actuatorName;
};


// ===========================================
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
    switch (aType) {
    case sensor::SensorType::VirtualLinkKinSensor: {
        outVec.reserve(pImpl->nLinkSensors);
        outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(pImpl->sensegloveLinkSensor));
        break;
    }
    case sensor::SensorType::VirtualJointKinSensor:
    {
        outVec.reserve(pImpl->sensegloveJointSensorVector.size());
        for( const auto&  sensegloveJointSensor: pImpl->sensegloveJointSensorVector)
            outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(sensegloveJointSensor));
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
    switch (aType) {
    case wearable::actuator::ActuatorType::Haptic: {
        outVec.reserve(pImpl->sensegloveHapticActuatorVector.size());
        for(const auto& hapticActuator : pImpl->sensegloveHapticActuatorVector)
        {
            outVec.push_back(static_cast<ElementPtr<actuator::IActuator>>(hapticActuator));
        }
        break;
    }
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

wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
HapticGlove::getVirtualJointKinSensor(const wearable::sensor::SensorName name) const
{
    if ( !pImpl->isAvailable(name, pImpl->gloveData.humanJointSensorNameIdMap) )
    {
        yError() << LogPrefix << "Invalid sensor name " << name;
        return nullptr;
    }

    // Return a shared point to the required sensor
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>&>(
        *pImpl->sensegloveJointSensorVector[pImpl->gloveData.humanJointSensorNameIdMap[name]]);
}

wearable::ElementPtr<const wearable::actuator::IHaptic>
HapticGlove::getHapticActuator(const actuator::ActuatorName name) const
{
    // Check if user-provided name corresponds to an available actuator
        if (!pImpl->isAvailable(name, pImpl->gloveData.humanHapticActuatorNameIdMap)) {
            yError() << LogPrefix << "Invalid actuator name " << name;
            return nullptr;
        }

        return dynamic_cast<wearable::ElementPtr<const wearable::actuator::IHaptic>&>(
            *pImpl->sensegloveHapticActuatorVector[pImpl->gloveData.humanHapticActuatorNameIdMap[name]]);
}
