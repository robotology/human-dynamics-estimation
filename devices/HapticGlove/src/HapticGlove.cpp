// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <HapticGlove.h>
#include <SenseGloveHelper.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>

#include <assert.h>
#include <cmath>
#include <map>
#include <mutex>
#include <stdio.h>
#include <string>
#include <thread>
#include <vector>

const std::string DeviceName = "HapticGlove";
const std::string LogPrefix = DeviceName + wearable::Separator;
const size_t PoseSize = 7;

using namespace wearable;
using namespace wearable::devices;

struct SenseGloveIMUData
{
    // sensors
    std::string humanHandLinkName;
    // [pos(x, y, z), quat(w, x, y , z)]
    // palm/hand frame wrt to hand inertial frame
    std::vector<double> humanPalmLinkPose;
    // [pos(x, y, z), quat(w, x, y , z)]
    // wrt to hand frame
    std::vector<std::vector<double>> fingertipPoses;
    std::vector<std::string> humanLinkNames;
    std::vector<std::vector<double>> humanLinkPoses;
    std::map<std::string, std::size_t> humanLinkSensorNameIdMap;

    std::vector<std::string> humanJointNames;
    std::vector<double> humanJointValues;
    std::map<std::string, std::size_t> humanJointSensorNameIdMap;

    // actuators
    std::vector<double> fingersForceFeedback;
    std::vector<double> fingersVibroTactileFeedback;
    senseGlove::ThumperCmd palmThumperFeedback;
    std::vector<double> fingersHapticFeedback; // [fingersForceFeedback,
                                               // fingersVibroTactileFeedback,
                                               // palmThumperFeedback]

    std::vector<std::string> humanFingerNames;
    std::map<std::string, std::size_t>
        humanHapticActuatorNameIdMap; // First part is Force Feedback, second part
                                      // is Vibrotactile feedback
};

class HapticGlove::SenseGloveImpl
{
public:
    mutable std::mutex mutex;

    double period = 0.01; //default 100Hz

    SenseGloveIMUData gloveData;

    WearableName wearableName;

    TimeStamp timeStamp;

    const size_t nFingers = 5;

    // Number of sensors
    const int nLinkSensors = 6; // 1 palm (hand) imu + 5 fingertips

    // Number of actuators
    const int nActuators = 11; // humanFingerNames.size()*2 + hand/palm thumper
    const int nActuatorsPerGlove = 5; // Number of the actuators per glove

    std::unique_ptr<senseGlove::SenseGloveHelper> pGlove; /**< Pointer to the glove object. */

    // link Sensor
    std::string linkSensorPrefix;
    class SenseGloveVirtualLinkKinSensor;
    std::vector<SensorPtr<SenseGloveVirtualLinkKinSensor>> sensegloveLinkSensorVector;

    // joint sensor
    std::string jointSensorPrefix;
    class SenseGloveVirtualJointKinSensor;
    std::vector<SensorPtr<SenseGloveVirtualJointKinSensor>> sensegloveJointSensorVector;

    // haptic actuator
    std::string hapticActuatorPrefix;
    class SenseGloveHapticActuator;
    std::vector<SensorPtr<SenseGloveHapticActuator>> sensegloveHapticActuatorVector;

    // Methods
    SenseGloveImpl();

    bool run();

    bool configure(yarp::os::Searchable& config);

    bool isAvailable(const std::string& name, const std::map<std::string, std::size_t>& map)
    {
        if (map.find(name) == map.end()) {
            return false;
        }
        return true;
    }

    bool close();
};

HapticGlove::SenseGloveImpl::SenseGloveImpl() {}

bool HapticGlove::SenseGloveImpl::configure(yarp::os::Searchable& config)
{
    std::lock_guard<std::mutex> lock(this->mutex);

    this->pGlove = std::make_unique<senseGlove::SenseGloveHelper>();

    // configure the glove device
    if (!this->pGlove->configure(config)) {
        yError() << LogPrefix << "Unable to initialize the sense glove helper device.";
        return false;
    }

    if (!this->pGlove->getHumanHandLinkName(this->gloveData.humanHandLinkName)) {
        yError() << LogPrefix << "Unable to get human hand link name.";
        return false;
    }
    // Initialize snese glove imu data buffer
    this->gloveData.humanPalmLinkPose.resize(PoseSize, 0.0);
    if (!this->pGlove->getPalmLinkPose(this->gloveData.humanPalmLinkPose)) {
        yError() << LogPrefix << "Unable to get the human palm IMU data.";
        return false;
    }

    this->gloveData.fingertipPoses.resize(this->nFingers, std::vector<double>(PoseSize));
    if (!this->pGlove->getGloveFingertipLinksPose(this->gloveData.fingertipPoses)) {
        yError() << LogPrefix << "Unable to get the human fingertip poses.";
        return false;
    }

    // get joint names
    if (!this->pGlove->getHumanJointNameList(this->gloveData.humanJointNames)) {
        yError() << LogPrefix << "Unable to get the human joint names.";
        return false;
    }

    // get the human joint values
    this->gloveData.humanJointValues.resize(this->gloveData.humanJointNames.size(), 0.0);
    if (!this->pGlove->getHandJointsAngles(this->gloveData.humanJointValues)) {
        yError() << LogPrefix << "Unable to get the human hand joint angles.";
        return false;
    }

    // get finger names
    if (!this->pGlove->getHumanFingerNameList(this->gloveData.humanFingerNames)) {
        yError() << LogPrefix << "Unable to get the human finger names.";
        return false;
    }

    this->gloveData.humanLinkPoses.resize(this->nLinkSensors, std::vector<double>(PoseSize));

    this->gloveData.fingersForceFeedback.resize(this->nFingers,
                                                0.0); // 5 actuators

    this->gloveData.fingersVibroTactileFeedback.resize(this->nFingers,
                                                       0.0); // 5 actuators

    this->gloveData.palmThumperFeedback = senseGlove::ThumperCmd::TurnOff;

    // [force feedback, vibrotactile feedback] = nActuators
    this->gloveData.fingersHapticFeedback.resize(this->nActuators, 0.0);

    return true;
}

bool HapticGlove::SenseGloveImpl::run()
{
    yTrace() << "SenseGloveImpl::run()";
    std::lock_guard<std::mutex> lock(this->mutex);

    // sensors
    this->pGlove->getPalmLinkPose(this->gloveData.humanPalmLinkPose);

    this->pGlove->getHandJointsAngles(this->gloveData.humanJointValues);

    this->pGlove->getGloveFingertipLinksPose(this->gloveData.fingertipPoses);

    // link poses
    this->gloveData.humanLinkPoses[0] = this->gloveData.humanPalmLinkPose;
    for (size_t i = 0; i < this->nFingers; i++) {
        this->gloveData.humanLinkPoses[i + 1] = this->gloveData.fingertipPoses[i];
    }

    // actuators
    for (size_t i = 0; i < this->nFingers; i++) {
        this->gloveData.fingersForceFeedback[i] = this->gloveData.fingersHapticFeedback[i];
        this->gloveData.fingersVibroTactileFeedback[i] =
            this->gloveData.fingersHapticFeedback[i + this->nFingers];
    }
    this->gloveData.palmThumperFeedback = static_cast<senseGlove::ThumperCmd>(
        round(this->gloveData.fingersHapticFeedback[2 * this->nFingers]));

    this->pGlove->setFingersForceReference(this->gloveData.fingersForceFeedback);
    this->pGlove->setBuzzMotorsReference(this->gloveData.fingersVibroTactileFeedback);
    this->pGlove->setPalmFeedbackThumper(this->gloveData.palmThumperFeedback);

    return true;
}

bool HapticGlove::SenseGloveImpl::close()
{

    if (!this->pGlove->turnOffBuzzMotors()) {
        yError() << LogPrefix << "cannot turn off the glove buzz motors.";
        return false;
    }

    if (!this->pGlove->turnOffForceFeedback()) {
        yError() << LogPrefix << "cannot turn off the glove force feedback.";
        return false;
    }

    if (!this->pGlove->turnOffPalmFeedbackThumper()) {
        yError() << LogPrefix << "cannot turn off the glove palm thumper feedback.";
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for 100ms.

    return true;
}
HapticGlove::HapticGlove()
    : PeriodicThread(0.01) //default 100Hz
    , m_pImpl{std::make_unique<SenseGloveImpl>()}
{}

// Destructor
HapticGlove::~HapticGlove() = default;

bool HapticGlove::open(yarp::os::Searchable& config)
{
    yTrace() << LogPrefix << "HapticGlove::open(yarp::os::Searchable& config)";

    // ==================================
    // Check the configuration parameters
    // ==================================
    // Period of the this device
    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period: " << m_pImpl->period << "s";
    }
    else {
        m_pImpl->period = config.find("period").asFloat64();
        yInfo() << LogPrefix << "Using the period : " << m_pImpl->period << "s";
    }
    setPeriod(m_pImpl->period);

    if (!(config.check("wearableName") && config.find("wearableName").isString())) {
        yInfo() << LogPrefix << "Using default wearable name SenseGlove";
        m_pImpl->wearableName = DeviceName;
    }
    else {
        m_pImpl->wearableName = config.find("wearableName").asString();
        yInfo() << LogPrefix << "Using the wearable name " << m_pImpl->wearableName;
    }

    // Configure the implementation class
    if (!m_pImpl->configure(config)) {
        yInfo() << LogPrefix << "Cannot configure the implementation class";
        return false;
    }

    // ======================================
    // Sensors ans Actuators Initialization
    // ======================================

    m_pImpl->linkSensorPrefix = getWearableName() + sensor::IVirtualLinkKinSensor::getPrefix();

    m_pImpl->sensegloveLinkSensorVector.push_back(
        std::make_shared<SenseGloveImpl::SenseGloveVirtualLinkKinSensor>(
            m_pImpl.get(), m_pImpl->linkSensorPrefix + m_pImpl->gloveData.humanHandLinkName));

    for (size_t i = 0; i < m_pImpl->gloveData.humanFingerNames.size(); i++) {
        m_pImpl->sensegloveLinkSensorVector.push_back(
            std::make_shared<SenseGloveImpl::SenseGloveVirtualLinkKinSensor>(
                m_pImpl.get(),
                m_pImpl->linkSensorPrefix + m_pImpl->gloveData.humanFingerNames[i]
                    + "::fingertip"));
    }

    m_pImpl->jointSensorPrefix = getWearableName() + sensor::IVirtualJointKinSensor::getPrefix();
    for (size_t i = 0; i < m_pImpl->gloveData.humanJointNames.size(); i++) {
        m_pImpl->sensegloveJointSensorVector.push_back(
            std::make_shared<SenseGloveImpl::SenseGloveVirtualJointKinSensor>(
                m_pImpl.get(), m_pImpl->jointSensorPrefix + m_pImpl->gloveData.humanJointNames[i]));
    }

    m_pImpl->hapticActuatorPrefix = getWearableName() + actuator::IHaptic::getPrefix();

    for (size_t i = 0; i < m_pImpl->gloveData.humanFingerNames.size(); i++) {
        m_pImpl->sensegloveHapticActuatorVector.push_back(
            std::make_shared<SenseGloveImpl::SenseGloveHapticActuator>(
                m_pImpl.get(),
                m_pImpl->hapticActuatorPrefix
                    + "HapticFeedback"));
    }

    for (size_t i = 0; i < m_pImpl->gloveData.humanFingerNames.size(); i++) {
        m_pImpl->sensegloveHapticActuatorVector.push_back(
            std::make_shared<SenseGloveImpl::SenseGloveHapticActuator>(
                m_pImpl.get(),
                m_pImpl->hapticActuatorPrefix + m_pImpl->gloveData.humanFingerNames[i]
                    + "::ForceFeedback"));
    }

    for (size_t i = 0; i < m_pImpl->gloveData.humanFingerNames.size(); i++) {
        m_pImpl->sensegloveHapticActuatorVector.push_back(
            std::make_shared<SenseGloveImpl::SenseGloveHapticActuator>(
                m_pImpl.get(),
                m_pImpl->hapticActuatorPrefix + m_pImpl->gloveData.humanFingerNames[i]
                    + "::VibroTactileFeedback"));
    }

    m_pImpl->sensegloveHapticActuatorVector.push_back(
        std::make_shared<SenseGloveImpl::SenseGloveHapticActuator>(
            m_pImpl.get(),
            m_pImpl->hapticActuatorPrefix + m_pImpl->gloveData.humanHandLinkName
                + "::palmThumper"));

    m_pImpl->gloveData.humanLinkSensorNameIdMap.emplace(
        std::make_pair(m_pImpl->linkSensorPrefix + m_pImpl->gloveData.humanHandLinkName, 0));

    for (size_t i = 0; i < m_pImpl->gloveData.humanFingerNames.size(); i++)
        m_pImpl->gloveData.humanLinkSensorNameIdMap.emplace(std::make_pair(
            m_pImpl->linkSensorPrefix + m_pImpl->gloveData.humanFingerNames[i] + "::fingertip",
            i + 1)); // +1 for the human hand link

    for (size_t i = 0; i < m_pImpl->gloveData.humanJointNames.size(); i++)
        m_pImpl->gloveData.humanJointSensorNameIdMap.emplace(
            std::make_pair(m_pImpl->jointSensorPrefix + m_pImpl->gloveData.humanJointNames[i], i));

    for (size_t i = 0; i < m_pImpl->gloveData.humanFingerNames.size(); i++)
        m_pImpl->gloveData.humanHapticActuatorNameIdMap.emplace(
            std::make_pair(m_pImpl->hapticActuatorPrefix
                               + "HapticFeedback",
                           i));

    for (size_t i = 0; i < m_pImpl->gloveData.humanFingerNames.size(); i++)
        m_pImpl->gloveData.humanHapticActuatorNameIdMap.emplace(
            std::make_pair(m_pImpl->hapticActuatorPrefix + m_pImpl->gloveData.humanFingerNames[i]
                               + "::ForceFeedback",
                           i));

    for (size_t i = 0; i < m_pImpl->gloveData.humanFingerNames.size(); i++)
        m_pImpl->gloveData.humanHapticActuatorNameIdMap.emplace(
            std::make_pair(m_pImpl->hapticActuatorPrefix + m_pImpl->gloveData.humanFingerNames[i]
                               + "::VibroTactileFeedback",
                           i + m_pImpl->nFingers));

    m_pImpl->gloveData.humanHapticActuatorNameIdMap.emplace(std::make_pair(
        m_pImpl->hapticActuatorPrefix + m_pImpl->gloveData.humanHandLinkName + "::palmThumper",
        2 * m_pImpl->nFingers));

    yInfo() << LogPrefix << "The device is opened successfully.";

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the period thread.";
        return false;
    }

    return true;
}

// ===========================================
// SenseGlove implementation of VirtualLinkKinSensor
// ===========================================

class HapticGlove::SenseGloveImpl::SenseGloveVirtualLinkKinSensor
    : public sensor::IVirtualLinkKinSensor
{
public:
    SenseGloveVirtualLinkKinSensor(HapticGlove::SenseGloveImpl* gloveImplPtr,
                                   const sensor::SensorName& name = {},
                                   const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
        : IVirtualLinkKinSensor(name, status)
        , m_gloveImpl(gloveImplPtr)
        , m_sensorName(name)
    {
        assert(m_gloveImpl != nullptr);
    }

    ~SenseGloveVirtualLinkKinSensor() override = default;

    bool getLinkAcceleration(Vector3& linear, Vector3& angular) const override
    {

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        // we do not handle linear and angular accelerations in the current
        // implementation

        linear.fill(0.0);

        angular.fill(0.0);

        return true;
    }

    bool getLinkPose(Vector3& position, Quaternion& orientation) const override
    {

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        if (!m_gloveImpl->isAvailable(m_sensorName,
                                      m_gloveImpl->gloveData.humanLinkSensorNameIdMap)) {
            yError() << LogPrefix << "The sensor name (" << m_sensorName
                     << ") is not found in the list of link sensors.";

            return false;
        }

        size_t id = m_gloveImpl->gloveData.humanLinkSensorNameIdMap[m_sensorName];

        // we do not handle position in the current implementation
        position = {m_gloveImpl->gloveData.humanLinkPoses[id][0],
                    m_gloveImpl->gloveData.humanLinkPoses[id][1],
                    m_gloveImpl->gloveData.humanLinkPoses[id][2]};

        orientation = {m_gloveImpl->gloveData.humanLinkPoses[id][3],
                       m_gloveImpl->gloveData.humanLinkPoses[id][4],
                       m_gloveImpl->gloveData.humanLinkPoses[id][5],
                       m_gloveImpl->gloveData.humanLinkPoses[id][6]};

        return true;
    }

    bool getLinkVelocity(Vector3& linear, Vector3& angular) const override
    {

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        // we do not handle linear and angular velocity

        angular.fill(0.0);

        linear.fill(0.0);

        return true;
    }

    inline void setStatus(const sensor::SensorStatus& status) { m_status = status; }

private:
    HapticGlove::SenseGloveImpl* m_gloveImpl{nullptr};
    std::string m_sensorName;
};

// ===========================================
// SenseGlove implementation of VirtualJointKinSensor
// ===========================================

class HapticGlove::SenseGloveImpl::SenseGloveVirtualJointKinSensor
    : public sensor::IVirtualJointKinSensor
{
public:
    SenseGloveVirtualJointKinSensor(HapticGlove::SenseGloveImpl* gloveImplPtr,
                                    const sensor::SensorName& name = {},
                                    const sensor::SensorStatus& status = sensor::SensorStatus::Ok)
        : IVirtualJointKinSensor(name, status)
        , m_gloveImpl(gloveImplPtr)
        , m_sensorName(name)
    {
        assert(m_gloveImpl != nullptr);
    }

    ~SenseGloveVirtualJointKinSensor() override = default;

    bool getJointPosition(double& position) const override
    {

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        if (!m_gloveImpl->isAvailable(m_sensorName,
                                      m_gloveImpl->gloveData.humanJointSensorNameIdMap)) {
            yError() << LogPrefix << "The sensor name (" << m_sensorName
                     << ") is not found in the list of joint sensors.";

            return false;
        }

        position =
            m_gloveImpl->gloveData
                .humanJointValues[m_gloveImpl->gloveData.humanJointSensorNameIdMap[m_sensorName]];

        return true;
    }

    bool getJointVelocity(double& velocity) const override
    {

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        velocity = 0.0;

        return true;
    }

    bool getJointAcceleration(double& acceleration) const override
    {

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        acceleration = 0.0;

        return true;
    }

    inline void setStatus(const sensor::SensorStatus& status) { m_status = status; }

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
    SenseGloveHapticActuator(HapticGlove::SenseGloveImpl* gloveImplPtr,
                             const actuator::ActuatorName& name = {},
                             const actuator::ActuatorStatus& status = actuator::ActuatorStatus::Ok)
        : IHaptic(name, status)
        , m_gloveImpl(gloveImplPtr)
        , m_actuatorName(name)
    {
        assert(m_gloveImpl != nullptr);
    }
    ~SenseGloveHapticActuator() override = default;

    bool setHapticCommand(double& value) const override
    {
        yError() << LogPrefix << "Wrong method has been called! To set the haptic command please use the setHapticsCommand method.";
        return false;
    }

    bool setHapticCommands(const std::vector<double>& forceValue, const std::vector<double>& vibrotactileValue) const override
    {

        std::lock_guard<std::mutex> lock(m_gloveImpl->mutex);

        if (forceValue.size() != m_gloveImpl->nActuatorsPerGlove || vibrotactileValue.size() != m_gloveImpl->nActuatorsPerGlove)
        {
            yError() << "The sizes of the forceValue and the vibrotactileValue vectors are not correct!";
            return false;
        }
        for (size_t i = 0; i < m_gloveImpl->nFingers; i++)
        {
            m_gloveImpl->gloveData.fingersHapticFeedback[i] = forceValue[i];
            m_gloveImpl->gloveData.fingersHapticFeedback[i + 5] = vibrotactileValue[i];
        }
        return true;
    }


    inline void setStatus(const actuator::ActuatorStatus& status) { m_status = status; }

private:
    HapticGlove::SenseGloveImpl* m_gloveImpl{nullptr};
    std::string m_actuatorName;
};

// ===========================================
void HapticGlove::run()
{
    // Get timestamp
    m_pImpl->timeStamp.time = yarp::os::Time::now();

    // to implement
    m_pImpl->run();
}

bool HapticGlove::close()
{
    yTrace() << LogPrefix << "HapticGlove::close()";

    if (!m_pImpl->close()) {
        yError() << LogPrefix << "Cannot close correctly the sense glove implementation.";
        return false;
    }
    return true;
}

void HapticGlove::threadRelease() {}

// =========================
// IPreciselyTimed interface
// =========================
yarp::os::Stamp HapticGlove::getLastInputStamp()
{
    // Stamp count should be always zero
    std::lock_guard<std::mutex> lock(m_pImpl->mutex);
    return yarp::os::Stamp(0, m_pImpl->timeStamp.time);
}

// ---------------------------
// Implement Sensors Methods
// ---------------------------

wearable::WearableName HapticGlove::getWearableName() const
{
    return m_pImpl->wearableName + wearable::Separator;
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
    std::lock_guard<std::mutex> lock(m_pImpl->mutex);
    return {m_pImpl->timeStamp.time, 0};
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
            outVec.reserve(m_pImpl->sensegloveLinkSensorVector.size());
            for (const auto& sensegloveLinkSensor : m_pImpl->sensegloveLinkSensorVector)
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(sensegloveLinkSensor));
            break;
        }
        case sensor::SensorType::VirtualJointKinSensor: {
            outVec.reserve(m_pImpl->sensegloveJointSensorVector.size());
            for (const auto& sensegloveJointSensor : m_pImpl->sensegloveJointSensorVector)
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

    for (const auto& a : actuators) {
        if (a->getActuatorName() == name) {
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
            outVec.reserve(m_pImpl->sensegloveHapticActuatorVector.size());
            for (const auto& hapticActuator : m_pImpl->sensegloveHapticActuatorVector) {
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

    if (!m_pImpl->isAvailable(name, m_pImpl->gloveData.humanLinkSensorNameIdMap)) {
        yError() << LogPrefix << "Invalid sensor name " << name;
        return nullptr;
    }

    // Return a shared point to the required sensor
    return static_cast<wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>>(
        m_pImpl->sensegloveLinkSensorVector.at(m_pImpl->gloveData.humanLinkSensorNameIdMap[name]));
}

wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
HapticGlove::getVirtualJointKinSensor(const wearable::sensor::SensorName name) const
{
    if (!m_pImpl->isAvailable(name, m_pImpl->gloveData.humanJointSensorNameIdMap)) {
        yError() << LogPrefix << "Invalid sensor name " << name;
        return nullptr;
    }

    // Return a shared point to the required sensor
    return static_cast<wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>>(
        m_pImpl->sensegloveJointSensorVector.at(
            m_pImpl->gloveData.humanJointSensorNameIdMap[name]));
}

wearable::ElementPtr<const wearable::actuator::IHaptic>
HapticGlove::getHapticActuator(const actuator::ActuatorName name) const
{
    // Check if user-provided name corresponds to an available actuator
    if (!m_pImpl->isAvailable(name, m_pImpl->gloveData.humanHapticActuatorNameIdMap)) {
        yError() << LogPrefix << "Invalid actuator name " << name;
        return nullptr;
    }

    return static_cast<wearable::ElementPtr<const wearable::actuator::IHaptic>>(
        m_pImpl->sensegloveHapticActuatorVector.at(
            m_pImpl->gloveData.humanHapticActuatorNameIdMap[name]));
}
