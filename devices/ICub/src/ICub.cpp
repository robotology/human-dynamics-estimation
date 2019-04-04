/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "ICub.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Property.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IAxisInfo.h>

#include <assert.h>
#include <map>
#include <mutex>
#include <string>
#include <vector>

using namespace wearable::devices;

const std::string LogPrefix = "ICub :";

class WrenchPort : public yarp::os::BufferedPort<yarp::os::Bottle>
{
public:
    std::vector<double> wrenchValues;
    bool firstRun = true;
    mutable std::mutex mtx;
    yarp::os::Network yarpNetwork;

    using yarp::os::BufferedPort<yarp::os::Bottle>::onRead;
    virtual void onRead(yarp::os::Bottle& wrench)
    {
        if (!wrench.isNull()) {
            std::lock_guard<std::mutex> lock(mtx);

            this->wrenchValues.resize(wrench.size());

            for (size_t i = 0; i < wrench.size(); ++i) {
                this->wrenchValues.at(i) = wrench.get(i).asDouble();
            }

            if (this->firstRun) {
                this->firstRun = false;
            }
        }
        else {
            yWarning() << LogPrefix << "[WrenchPort] read an invalid wrench bottle";
        }
    }
};

class ICub::ICubImpl
{
public:
    //Generic
    const WearableName wearableName = "ICub";

    size_t nSensors;
    std::vector<std::string> sensorNames;

    wearable::TimeStamp timeStamp;

    //FT Sensor
    class ICubForceTorque6DSensor;

    std::vector<std::string> ftSensorNames;
    std::map<std::string, SensorPtr<ICubForceTorque6DSensor>> ftSensorsMap;

    WrenchPort leftHandFTPort;
    WrenchPort rightHandFTPort;

    //Joint Sensor
    class ICubVirtualJointKinSensor;

    std::vector<std::string> jointSensorNames;
    std::map<std::string, SensorPtr<ICubVirtualJointKinSensor>> jointSensorsMap;

    //Vector of motor control boards
    yarp::os::Property options;
    std::vector<yarp::dev::PolyDriver*> remoteControlBoardsVec;
    std::vector<std::string> controlBoardNamesVec;
    std::vector<double> controlBoardJointsVec;

    yarp::dev::IEncoders* iEncoders = nullptr;
    yarp::dev::IAxisInfo* iAxisInfo = nullptr;
};

// ==========================================
// ICub implementation of ForceTorque6DSensor
// ==========================================
class ICub::ICubImpl::ICubForceTorque6DSensor : public wearable::sensor::IForceTorque6DSensor
{
public:
    ICub::ICubImpl* icubImpl = nullptr;

    // ------------------------
    // Constructor / Destructor
    // ------------------------
    ICubForceTorque6DSensor(
        ICub::ICubImpl* impl,
        const wearable::sensor::SensorName name = {},
        const wearable::sensor::SensorStatus status =
            wearable::sensor::SensorStatus::WaitingForFirstRead) // Default sensor status
        : IForceTorque6DSensor(name, status)
        , icubImpl(impl)
    {
        // Set the sensor status Ok after receiving the first data on the wrench ports
        if (!icubImpl->leftHandFTPort.firstRun || !icubImpl->rightHandFTPort.firstRun) {
            auto nonConstThis = const_cast<ICubForceTorque6DSensor*>(this);
            nonConstThis->setStatus(wearable::sensor::SensorStatus::Ok);
        }
    }

    ~ICubForceTorque6DSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    // ==============================
    // IForceTorque6DSensor interface
    // ==============================
    bool getForceTorque6D(Vector3& force3D, Vector3& torque3D) const override
    {
        assert(icubImpl != nullptr);

        std::vector<double> wrench;

        // Reading wrench from WBD ports
        if (this->m_name
            == icubImpl->wearableName + wearable::Separator
                   + sensor::IForceTorque6DSensor::getPrefix() + "leftWBDFTSensor") {

            std::lock_guard<std::mutex> lck(icubImpl->leftHandFTPort.mtx);
            wrench = icubImpl->leftHandFTPort.wrenchValues;

            icubImpl->timeStamp.time = yarp::os::Time::now();
        }
        else if (this->m_name
                 == icubImpl->wearableName + wearable::Separator
                        + sensor::IForceTorque6DSensor::getPrefix() + "rightWBDFTSensor") {

            std::lock_guard<std::mutex> lck(icubImpl->rightHandFTPort.mtx);
            wrench = icubImpl->rightHandFTPort.wrenchValues;

            icubImpl->timeStamp.time = yarp::os::Time::now();
        }

        // Check the size of wrench values
        if (wrench.size() == 6) {
            force3D[0] = wrench.at(0);
            force3D[1] = wrench.at(1);
            force3D[2] = wrench.at(2);

            torque3D[0] = wrench.at(3);
            torque3D[1] = wrench.at(4);
            torque3D[2] = wrench.at(5);
        }
        else {
            yWarning() << LogPrefix << "Size of wrench read is wrong. Setting zero values instead.";
            force3D.fill(0.0);
            torque3D.fill(0.0);

            // Set sensor status to error if wrong data is read
            // auto nonConstThis = const_cast<ICubForceTorque6DSensor*>(this);
            // nonConstThis->setStatus(wearable::sensor::SensorStatus::Error);
        }

        return true;
    }
};

// ================================================
// ICub implementation of ICubVirtualJointKinSensor
// ================================================
class ICub::ICubImpl::ICubVirtualJointKinSensor : public wearable::sensor::IVirtualJointKinSensor
{
public:
    ICub::ICubImpl* icubImpl = nullptr;

    yarp::dev::IEncoders* jointIEncoders = nullptr;
    int                   jointIndex;

    //Joint Variables
    double                jointPos;
    double                jointVel;
    double                jointAcc;

    // ------------------------
    // Constructor / Destructor
    // ------------------------
    ICubVirtualJointKinSensor(
        ICub::ICubImpl* impl,
        const wearable::sensor::SensorName name = {},
        const wearable::sensor::SensorStatus status =
            wearable::sensor::SensorStatus::WaitingForFirstRead) // Default sensor status
        : IVirtualJointKinSensor(name, status)
        , icubImpl(impl)
    {
        //Get the joint index corresponding to the sensor and the encoders pointer
        size_t boardCount = 0;
        yarp::dev::IEncoders* m_iEncoders = nullptr;
        yarp::dev::IAxisInfo* m_iAxisInfo = nullptr;

        for (const auto& controlBoard :  icubImpl->controlBoardNamesVec) {
            //Get encoder interface
            if (!icubImpl->remoteControlBoardsVec.at(boardCount)->view(m_iEncoders) || !m_iEncoders) {
                yError() << LogPrefix << "Failed to view the IEncoder interface from the " << controlBoard << " remote control board device";
            }

            //Get axis info interface
            if (!icubImpl->remoteControlBoardsVec.at(boardCount)->view(m_iAxisInfo) || !m_iAxisInfo) {
                yError() << LogPrefix << "Failed to view the IAxisInfo interface from the " << controlBoard << " remote control board device";
            }

            //Get joint axes from encoder interface
            int nJoints;
            m_iEncoders->getAxes(&nJoints);

            //Get axes names
            for (int j = 0; j < nJoints; j++) {
                std::string axisName;
                m_iAxisInfo->getAxisName(j, axisName);
                //Check if the joint name is equal to sensor name
                if (this->m_name
                        == icubImpl->wearableName + wearable::Separator
                        + sensor::IVirtualJointKinSensor::getPrefix() + axisName) {
                        this->jointIndex = j;
                        this->jointIEncoders = m_iEncoders;
                        break;
                }

            }

            boardCount++;
        }

        //Set the sensor status Ok after receiving the first data from joint encoder
        if (this->jointIEncoders->getEncoder(this->jointIndex, &this->jointPos) &&
            this->jointIEncoders->getEncoderSpeed(this->jointIndex, &this->jointVel) &&
            this->jointIEncoders->getEncoderAcceleration(this->jointIndex, &this->jointAcc)) {
            auto nonConstThis = const_cast<ICubVirtualJointKinSensor*>(this);
            nonConstThis->setStatus(wearable::sensor::SensorStatus::Ok);
        }

    }

    ~ICubVirtualJointKinSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    // ====================================
    // ICubVirtualJointKinSensor interfaces
    // ====================================
    //ICub joint quantities are in radians
    bool getJointPosition(double& position) const override
    {
        assert(icubImpl != nullptr);

        // Reading joint position
        if (!this->jointIEncoders->getEncoder(this->jointIndex, &position)) {
            return false;
        }

        return true;
    }

    bool getJointVelocity(double& velocity) const override
    {
        assert(icubImpl != nullptr);

        // Reading joint velocity
        if (!this->jointIEncoders->getEncoderSpeed(this->jointIndex, &velocity)) {
            return false;
        }

        return true;
    }

    bool getJointAcceleration(double& acceleration) const override
    {
        assert(icubImpl != nullptr);

        // Reading joint acceleration
        if (!this->jointIEncoders->getEncoderAcceleration(this->jointIndex, &acceleration)) {
            return false;
        }

        return true;
    }
};

// ==========================================
// ICub constructor/destructor implementation
// ==========================================
ICub::ICub()
    : pImpl{new ICubImpl()}
{}

ICub::~ICub() = default;

// ======================
// DeviceDriver interface
// ======================
bool ICub::open(yarp::os::Searchable& config)
{
    yInfo() << LogPrefix << "Starting to configure";

    // Configure clock
    if (!yarp::os::Time::isSystemClock()) {
        yarp::os::Time::useSystemClock();
    }

    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    yarp::os::Bottle wbdHandsFTSensorsGroup = config.findGroup("wbd-hand-ft-sensors");
    if (wbdHandsFTSensorsGroup.isNull()) {
        yError() << LogPrefix << "REQUIRED parameter <wbd-hand-ft-sensors> NOT found";
        return false;
    }

    if (!wbdHandsFTSensorsGroup.check("leftHand") || !wbdHandsFTSensorsGroup.check("rightHand")) {
        yError() << LogPrefix << "REQUIRED parameter <leftHand> or <rightHand> NOT found";
        return false;
    }

    yarp::os::Bottle icubJointSensorGroup = config.findGroup("icub-joint-sensors");
    if (icubJointSensorGroup.isNull()) {
        yError() << LogPrefix << "REQUIRED parameter <icub-joint-sensors> NOT found";
        return false;
    }

    if (!icubJointSensorGroup.check("controlBoardsList")) {
        yError() << LogPrefix << "REQUIRED parameter <controlBoardsList> NOT found";
        return false;
    }

    if (!icubJointSensorGroup.check("remotePrefix") || !icubJointSensorGroup.check("localPrefix")) {
        yError() << LogPrefix << "REQUIRED parameter <remotePrefix> or <localPrefix> NOT found";
        return false;
    }

    // ======================================
    // PARSE FT SENSORS CONFIGURATION OPTIONS
    // ======================================

    pImpl->nSensors = wbdHandsFTSensorsGroup.size() - 1; //Set number of sensors equal to number of ft sensors
    std::string leftHandFTPortName =
        wbdHandsFTSensorsGroup.check("leftHand", yarp::os::Value(false)).asString();
    std::string rightHandFTPortName =
        wbdHandsFTSensorsGroup.check("rightHand", yarp::os::Value(false)).asString();

    // =========================
    // FT Sensors Initialization
    // =========================

    if (!(pImpl->leftHandFTPort.open("/ICub/leftHandFTSensor:i")
          && yarp::os::Network::connect(leftHandFTPortName,
                                        pImpl->leftHandFTPort.getName().c_str()))) {
        yError() << LogPrefix << "Failed to open or connect to "
                 << pImpl->leftHandFTPort.getName().c_str();
        return false;
    }

    pImpl->ftSensorNames.push_back("leftWBDFTSensor");
    pImpl->sensorNames.push_back("leftWBDFTSensor");

    if (!(pImpl->rightHandFTPort.open("/ICub/rightHandFTSensor:i")
          && yarp::os::Network::connect(rightHandFTPortName,
                                        pImpl->rightHandFTPort.getName().c_str()))) {
        yError() << LogPrefix << "Failed to open or connect to "
                 << pImpl->rightHandFTPort.getName().c_str();
        return false;
    }

    pImpl->ftSensorNames.push_back("rightWBDFTSensor");
    pImpl->sensorNames.push_back("rightWBDFTSensor");

    // Enable the callback of the ports
    pImpl->leftHandFTPort.useCallback();
    pImpl->rightHandFTPort.useCallback();

    // Wait to receive first data
    while (pImpl->leftHandFTPort.firstRun && pImpl->rightHandFTPort.firstRun) {
        yarp::os::Time::delay(10);
    }

    std::string ft6dPrefix =
        getWearableName() + wearable::Separator + sensor::IForceTorque6DSensor::getPrefix();

    for (size_t s = 0; s < pImpl->ftSensorNames.size(); ++s) {
        // Create the new sensors
        auto ft6d = std::make_shared<ICubImpl::ICubForceTorque6DSensor>(
            pImpl.get(), ft6dPrefix + pImpl->ftSensorNames[s]);

        pImpl->ftSensorsMap.emplace(ft6dPrefix + pImpl->ftSensorNames[s],
                                    std::shared_ptr<ICubImpl::ICubForceTorque6DSensor>{ft6d});
    }

    // =========================================
    // PARSE JOINT SENSORS CONFIGURATION OPTIONS
    // =========================================

    //Get control board list
    int controlBoardsListIndex;
    bool controlBoardsListExists = false;
    for (size_t i = 0; i < icubJointSensorGroup.size(); i++) {
        if (icubJointSensorGroup.get(i).isList()) {
            yarp::os::Bottle* param = icubJointSensorGroup.get(i).asList();
            if (param->get(0).asString() == "controlBoardsList") {
                controlBoardsListIndex = i;
                controlBoardsListExists = true;
            }
        }
    }

    if (!controlBoardsListExists) {
        yError() << LogPrefix << "<controlBoardsList> param is not a list";
        return false;
    }

    yarp::os::Bottle* controlBoardsListParam = icubJointSensorGroup.get(controlBoardsListIndex).asList();
    yarp::os::Bottle *controlBoardList = controlBoardsListParam->get(1).asList();

    std::string remotePrefix = icubJointSensorGroup.check("remotePrefix", yarp::os::Value(false)).asString();
    std::string localPrefix = icubJointSensorGroup.check("localPrefix", yarp::os::Value(false)).asString();

    yInfo() << LogPrefix << "Control boards list         : " << controlBoardList->toString().c_str();
    yInfo() << LogPrefix << "Remote control board prefix : " << remotePrefix;
    yInfo() << LogPrefix << "Local control board prefix  : " << localPrefix;

    // ============================
    // Joint Sensors Initialization
    // ============================

    //Resize control board variables
    pImpl->controlBoardNamesVec.resize(controlBoardList->size());
    pImpl->controlBoardJointsVec.resize(controlBoardList->size());
    pImpl->remoteControlBoardsVec.resize(controlBoardList->size());

    //Set control board names
    for (unsigned index = 0; index < controlBoardList->size(); index++) {
        std::string controlBoardName = controlBoardList->get(index).asString();
        pImpl->controlBoardNamesVec.at(index) = controlBoardName;
    }

    //Initialize remote control boards
    size_t boardCount = 0;
    for (const auto& controlBoard : pImpl->controlBoardNamesVec) {
        pImpl->options.put("device", "remote_controlboard");
        pImpl->options.put("remote", remotePrefix + "/" + controlBoard);
        pImpl->options.put("local", localPrefix + "/" + controlBoard);

        //Open remote control board
        pImpl->remoteControlBoardsVec.at(boardCount) = new yarp::dev::PolyDriver;
        pImpl->remoteControlBoardsVec.at(boardCount)->open(pImpl->options);

        if (!pImpl->remoteControlBoardsVec.at(boardCount)->isValid()) {
            yError() << LogPrefix << "Failed to open the remote control board device for " << controlBoard << " part";
            return false;
        }

        //Get encoder interface
        if (!pImpl->remoteControlBoardsVec.at(boardCount)->view(pImpl->iEncoders) || !pImpl->iEncoders) {
            yError() << LogPrefix << "Failed to view the IEncoder interface from the " << controlBoard << " remote control board device";
            return false;
        }

        //Get axis info interface
        if (!pImpl->remoteControlBoardsVec.at(boardCount)->view(pImpl->iAxisInfo) || !pImpl->iAxisInfo) {
            yError() << LogPrefix << "Failed to view the IAxisInfo interface from the " << controlBoard << " remote control board device";
            return false;
        }

        //Get joint axes from encoder interface
        int remoteControlBoardJoints;
        pImpl->iEncoders->getAxes(&remoteControlBoardJoints);

        //Get axes names and pad to sensor names
        for (int j = 0; j < remoteControlBoardJoints; j++) {
            std::string axisName;
            pImpl->iAxisInfo->getAxisName(j, axisName);
            pImpl->jointSensorNames.push_back(axisName);
            pImpl->sensorNames.push_back(axisName);
        }

        //Update control board joints vector
        pImpl->controlBoardJointsVec.at(boardCount) = remoteControlBoardJoints;

        //Update the number of sensors
        pImpl->nSensors = pImpl->nSensors + remoteControlBoardJoints;

        pImpl->options.clear();
        boardCount++;
    }

    //Get joint sensor prefix
    std::string jointSensorPrefix =
        getWearableName() + wearable::Separator + sensor::IVirtualJointKinSensor::getPrefix();

    for (size_t s = 0; s < pImpl->jointSensorNames.size(); s++) {
        // Create the new sensors
        auto jointsensor = std::make_shared<ICubImpl::ICubVirtualJointKinSensor>(
            pImpl.get(), jointSensorPrefix + pImpl->jointSensorNames[s]);

        pImpl->jointSensorsMap.emplace(jointSensorPrefix + pImpl->jointSensorNames[s],
                                       std::shared_ptr<ICubImpl::ICubVirtualJointKinSensor>{jointsensor});
    }

    return true;
}

// ---------------
// Generic Methods
// ---------------

wearable::WearableName ICub::getWearableName() const
{
    return pImpl->wearableName;
}

wearable::WearStatus ICub::getStatus() const
{
    wearable::WearStatus status = wearable::WearStatus::Ok;

    for (const auto& s : getAllSensors()) {
        if (s->getSensorStatus() != sensor::SensorStatus::Ok) {
            status = wearable::WearStatus::Error;
        }

    }

    // Default return status is Ok
    return status;
}

wearable::TimeStamp ICub::getTimeStamp() const
{
    // Stamp count should be always zero
    return {pImpl->timeStamp.time, 0};
}

bool ICub::close()
{
    return true;
}

// =========================
// IPreciselyTimed interface
// =========================
yarp::os::Stamp ICub::getLastInputStamp()
{
    // Stamp count should be always zero
    return yarp::os::Stamp(0, pImpl->timeStamp.time);
}

// ---------------------------
// Implement Sensors Methods
// ---------------------------

wearable::SensorPtr<const wearable::sensor::ISensor>
ICub::getSensor(const wearable::sensor::SensorName name) const
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
ICub::getSensors(const wearable::sensor::SensorType type) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> outVec;
    outVec.reserve(pImpl->nSensors);
    switch (type) {
        case sensor::SensorType::ForceTorque6DSensor: {
            for (const auto& ft6d : pImpl->ftSensorsMap) {
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(ft6d.second));
            }

            break;
        }
        case sensor::SensorType::VirtualJointKinSensor : {
            for (const auto& jointSensor : pImpl->jointSensorsMap) {
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(jointSensor.second));
            }

            break;
        }
        default: {
            return {};
        }
    }

    return outVec;
}

// -----------
// FT6D Sensor
// -----------

wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
ICub::getForceTorque6DSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->ftSensorsMap.find(name) == pImpl->ftSensorsMap.end()) {
        yError() << LogPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared point to the required sensor
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>&>(*pImpl->ftSensorsMap.at(name));
}

// ------------
// JOINT Sensor
// ------------

wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
ICub::getVirtualJointKinSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->jointSensorsMap.find(name) == pImpl->jointSensorsMap.end()) {
        yError() << LogPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared point to the required sensor
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>&>(*pImpl->jointSensorsMap.at(name));
}
