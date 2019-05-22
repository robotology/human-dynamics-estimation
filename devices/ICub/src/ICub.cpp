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
#include <yarp/os/ResourceFinder.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>

#include <assert.h>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

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

    mutable std::mutex mtx;

    // Generic
    const WearableName wearableName = "ICub";

    //Gravity variable
    iDynTree::Vector3 world_gravity;

    // model
    iDynTree::Model robotModel;
    std::string floatingBaseName;

    // kindyn object
    iDynTree::KinDynComputations kinDynComputations;

    bool updatedLinkQuantities;
    void computeLinkQuantities();

    size_t nSensors;
    std::vector<std::string> sensorNames;

    wearable::TimeStamp timeStamp;

    // FT Sensor
    class ICubForceTorque6DSensor;

    std::vector<std::string> ftSensorNames;
    std::vector<std::string> ftSensorPortNames;
    std::vector<WrenchPort*> wrenchPortsVector;
    std::map<std::string, SensorPtr<ICubForceTorque6DSensor>> ftSensorsMap;

    // Joint Sensor
    class ICubVirtualJointKinSensor;

    std::vector<std::string> jointSensorNames;
    std::map<std::string, SensorPtr<ICubVirtualJointKinSensor>> jointSensorsMap;

    // Vector of motor control boards
    yarp::os::Property options;
    std::vector<yarp::dev::PolyDriver*> remoteControlBoardsVec;
    std::vector<std::string> controlBoardNamesVec;
    std::vector<double> controlBoardJointsVec;

    yarp::dev::IEncoders* iEncoders = nullptr;
    yarp::dev::IAxisInfo* iAxisInfo = nullptr;

    // Virtual Link Sensor
    class ICubVirtualLinkKinSensor;

    std::vector<std::string> linkSensorNames;
    std::map<std::string, SensorPtr<ICubVirtualLinkKinSensor>> linkSensorsMap;

    // Joint variables
    iDynTree::VectorDynSize jointPositionsVec;
    iDynTree::VectorDynSize jointVelocitiesVec;
    iDynTree::VectorDynSize jointAccelerationsVec;

    // Link variables
    std::map<std::string, wearable::Vector3>    linkPositionMap;
    std::map<std::string, wearable::Quaternion> linkOrientationMap;
    std::map<std::string, wearable::Vector3>    linkLinearVelocityMap;
    std::map<std::string, wearable::Vector3>    linkAngularVelocityMap;
    std::map<std::string, wearable::Vector3>    linkLinearAccelerationMap;
    std::map<std::string, wearable::Vector3>    linkAngularAccelerationMap;
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
        for (size_t n = 0; n < icubImpl->ftSensorNames.size(); n++) {
            if (this->m_name.find(icubImpl->ftSensorNames.at(n))) {
                auto nonConstThis = const_cast<ICubForceTorque6DSensor*>(this);
                nonConstThis->setStatus(wearable::sensor::SensorStatus::Ok);
                break;
            }
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
        for (size_t n = 0; n < icubImpl->ftSensorNames.size(); n++) {
            if (this->m_name.find(icubImpl->ftSensorNames.at(n))) {
                std::lock_guard<std::mutex> lck(icubImpl->wrenchPortsVector.at(n)->mtx);
                wrench = icubImpl->wrenchPortsVector.at(n)->wrenchValues;
                icubImpl->timeStamp.time = yarp::os::Time::now();
                break;
            }
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
    int jointIndex;

    // Joint Variables
    double jointPos;
    double jointVel;
    double jointAcc;

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
        // Get the joint index corresponding to the sensor and the encoders pointer
        size_t boardCount = 0;
        yarp::dev::IEncoders* m_iEncoders = nullptr;
        yarp::dev::IAxisInfo* m_iAxisInfo = nullptr;

        for (const auto& controlBoard : icubImpl->controlBoardNamesVec) {
            // Get encoder interface
            if (!icubImpl->remoteControlBoardsVec.at(boardCount)->view(m_iEncoders)
                || !m_iEncoders) {
                yError() << LogPrefix << "Failed to view the IEncoder interface from the "
                         << controlBoard << " remote control board device";
            }

            // Get axis info interface
            if (!icubImpl->remoteControlBoardsVec.at(boardCount)->view(m_iAxisInfo)
                || !m_iAxisInfo) {
                yError() << LogPrefix << "Failed to view the IAxisInfo interface from the "
                         << controlBoard << " remote control board device";
            }

            // Get joint axes from encoder interface
            int nJoints;
            m_iEncoders->getAxes(&nJoints);

            // Get axes names
            for (int j = 0; j < nJoints; j++) {
                std::string axisName;
                m_iAxisInfo->getAxisName(j, axisName);
                // Check if the joint name is equal to sensor name
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

        // Set the sensor status Ok after receiving the first data from joint encoder
        if (this->jointIEncoders->getEncoder(this->jointIndex, &this->jointPos)
            && this->jointIEncoders->getEncoderSpeed(this->jointIndex, &this->jointVel)
            && this->jointIEncoders->getEncoderAcceleration(this->jointIndex, &this->jointAcc)) {
            auto nonConstThis = const_cast<ICubVirtualJointKinSensor*>(this);
            nonConstThis->setStatus(wearable::sensor::SensorStatus::Ok);
        }
    }

    ~ICubVirtualJointKinSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    // ====================================
    // ICubVirtualJointKinSensor interfaces
    // ====================================
    // ICub joint quantities are in radians
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

void ICub::ICubImpl::computeLinkQuantities()
{
    size_t boardCount = 0;
    int jointIndex = 0;
    yarp::dev::IEncoders* iEncoders = nullptr;
    yarp::dev::IAxisInfo* iAxisInfo = nullptr;

    // Clear joint buffers
    this->jointPositionsVec.zero();
    this->jointVelocitiesVec.zero();
    this->jointAccelerationsVec.zero();

    for (const auto& controlBoard : this->controlBoardNamesVec) {
        // Get encoder interface
        if (!this->remoteControlBoardsVec.at(boardCount)->view(iEncoders)
                || !iEncoders) {
            yError() << LogPrefix << "Failed to view the IEncoder interface from the "
                     << controlBoard << " remote control board device";
        }

        // Get axis info interface
        if (!this->remoteControlBoardsVec.at(boardCount)->view(iAxisInfo)
                || !iAxisInfo) {
            yError() << LogPrefix << "Failed to view the IAxisInfo interface from the "
                     << controlBoard << " remote control board device";
        }

        // Get joint axes from encoder interface
        int nJoints;
        iEncoders->getAxes(&nJoints);

        // Get axes names
        for (int j = 0; j < nJoints; j++) {
            std::string axisName;
            iAxisInfo->getAxisName(j, axisName);

            // Store the joint quantities
            double position;
            double velocity;
            double acceleration;

            // Get joint quantities
            iEncoders->getEncoder(j, &position);
            iEncoders->getEncoderSpeed(j, &velocity);
            iEncoders->getEncoderAcceleration(j, &acceleration);

            if (this->robotModel.getJointIndex(axisName) != iDynTree::JOINT_INVALID_INDEX) {

                // Update the joint buffers
                this->jointPositionsVec.setVal(jointIndex, position*(M_PI/180));
                this->jointVelocitiesVec.setVal(jointIndex, velocity*(M_PI/180));
                this->jointAccelerationsVec.setVal(jointIndex, acceleration*(M_PI/180));

                jointIndex++;

            }

        }

        boardCount++;
    }

    // Set floating base
    this->kinDynComputations.setFloatingBase(this->floatingBaseName);

    // Set robot state
    this->kinDynComputations.setRobotState(iDynTree::Transform::Identity(),
                                           this->jointPositionsVec,
                                           iDynTree::Twist::Zero(),
                                           this->jointVelocitiesVec,
                                           this->world_gravity);

    // Set frame velocity representation
    // TODO: Double check this representation
    this->kinDynComputations.setFrameVelocityRepresentation(iDynTree::INERTIAL_FIXED_REPRESENTATION);

    // Clear link variables
    this->linkPositionMap.clear();
    this->linkOrientationMap.clear();
    this->linkLinearVelocityMap.clear();
    this->linkAngularVelocityMap.clear();

    {
        std::lock_guard<std::mutex> lock(this->mtx);
        for (size_t l = 0; l < this->robotModel.getNrOfLinks(); l++)
        {
            // Get link name
            std::string linkName = this->robotModel.getFrameName(l);
            // Get link pose
            iDynTree::Transform linkTransform;
            linkTransform = this->kinDynComputations.getRelativeTransform(this->floatingBaseName,
                                                                          linkName);

            iDynTree::Position position = linkTransform.getPosition();
            iDynTree::Vector4 orientation = linkTransform.getRotation().asQuaternion(); // format: w x y z

            wearable::Vector3 linkPosition;
            linkPosition.at(0) =  position.getVal(0);
            linkPosition.at(1) =  position.getVal(1);
            linkPosition.at(2) =  position.getVal(2);

            this->linkPositionMap.emplace(linkName, linkPosition);

            wearable::Quaternion linkOrientation;
            //TODO: Double check quaternion handling
            linkOrientation.at(0) = orientation.getVal(0);
            linkOrientation.at(1) = orientation.getVal(1);
            linkOrientation.at(2) = orientation.getVal(2);
            linkOrientation.at(3) = orientation.getVal(3);

            this->linkOrientationMap.emplace(linkName, linkOrientation);

            // Get link velocity
            iDynTree::Twist linkVelocity;
            linkVelocity = this->kinDynComputations.getFrameVel(linkName);

            iDynTree::Vector3 linearVelocity = linkVelocity.getLinearVec3();
            iDynTree::Vector3 angularVelocity = linkVelocity.getAngularVec3();

            wearable::Vector3 linkLinearVelocity;
            linkLinearVelocity.at(0) = linearVelocity.getVal(0);
            linkLinearVelocity.at(1) = linearVelocity.getVal(1);
            linkLinearVelocity.at(2) = linearVelocity.getVal(2);

            wearable::Vector3 linkAngularVelocity;
            linkAngularVelocity.at(0) = angularVelocity.getVal(0);
            linkAngularVelocity.at(1) = angularVelocity.getVal(1);
            linkAngularVelocity.at(2) = angularVelocity.getVal(2);

            this->linkLinearVelocityMap.emplace(linkName, linkLinearVelocity);
            this->linkAngularVelocityMap.emplace(linkName, linkAngularVelocity);

            // Get link acceleration
            //TODO
            wearable::Vector3 linkLinearAcceleration;
            wearable::Vector3 linkAngularAcceleration;

            linkLinearAcceleration.fill(0.0);
            linkAngularAcceleration.fill(0.0);

            this->linkLinearAccelerationMap.emplace(linkName, linkLinearAcceleration);
            this->linkAngularAccelerationMap.emplace(linkName, linkAngularAcceleration);
        }
    }

}

// ===========================================
// ICub implementation of VirtualLinkKinsensor
// ===========================================

class ICub::ICubImpl::ICubVirtualLinkKinSensor : public wearable::sensor::IVirtualLinkKinSensor
{
public:
    ICub::ICubImpl* icubImpl = nullptr;

    // Constructor
    ICubVirtualLinkKinSensor(
        ICub::ICubImpl* impl,
        const wearable::sensor::SensorName name = {},
        const wearable::sensor::SensorStatus status =
            wearable::sensor::SensorStatus::WaitingForFirstRead) // Default sensor status
        : IVirtualLinkKinSensor(name, status)
        , icubImpl(impl)
    {
        //TODO: Check if any other sensor checks can be used
        auto nonConstThis = const_cast<ICubVirtualLinkKinSensor*>(this);
        nonConstThis->setStatus(wearable::sensor::SensorStatus::Ok);
    }

    // Destructor
    ~ICubVirtualLinkKinSensor() override = default;

    // -------------------------------
    // IVirtualLinkKinSensor interface
    // -------------------------------

    bool getLinkPose(Vector3& position, Quaternion& orientation) const override
    {
        if (icubImpl->linkSensorsMap.find(this->m_name)
            == icubImpl->linkSensorsMap.end()) {
            yError() << LogPrefix << "Sensor" << this->m_name << "NOT found";
            position.fill(0.0);
            orientation.fill(0.0);
            return false;
        }

        // Call link quantities computation
        //TODO: This call has to be handled differently
        icubImpl->computeLinkQuantities();

        // Get link name
        size_t found = this->m_name.find_last_of(":");
        std::string linkName = this->m_name.substr(found+1);

        {
            // Get link position
            std::lock_guard<std::mutex> lock(icubImpl->mtx);
            if (icubImpl->linkPositionMap.find(linkName) == icubImpl->linkPositionMap.end()) {
                yError() << LogPrefix << "Link " << linkName << "not found in link position map";
                position.fill(0.0);
                return false;
            }

            auto linkPos = icubImpl->linkPositionMap.find(linkName);
            position = linkPos->second;

            // Get link orientation
            if (icubImpl->linkOrientationMap.find(linkName) == icubImpl->linkOrientationMap.end()) {
                yError() << LogPrefix << "Link " << linkName << "not found in link orientation map";
                orientation.fill(0.0);
                return false;
            }

            auto linkOri = icubImpl->linkOrientationMap.find(linkName);
            orientation = linkOri->second;
        }

        return true;
    }

    bool getLinkVelocity(Vector3& linear, Vector3& angular) const override
    {
        if (icubImpl->linkSensorsMap.find(this->m_name)
            == icubImpl->linkSensorsMap.end()) {
            yError() << LogPrefix << "Sensor" << this->m_name << "NOT found";
            linear.fill(0.0);
            angular.fill(0.0);
            return false;
        }

        // Call link quantities computation
        icubImpl->computeLinkQuantities();

        // Get link name
        size_t found = this->m_name.find_last_of(":");
        std::string linkName = this->m_name.substr(found+1);

        {
            std::lock_guard<std::mutex> lock(icubImpl->mtx);
            if (icubImpl->linkLinearVelocityMap.find(linkName) == icubImpl->linkLinearVelocityMap.end()) {
                yError() << LogPrefix << "Link " << linkName << "not found in link linear velocity map";
                linear.fill(0.0);
                return false;
            }

            auto linearVel = icubImpl->linkLinearVelocityMap.find(linkName);
            linear = linearVel->second;

            if (icubImpl->linkAngularVelocityMap.find(linkName) == icubImpl->linkAngularVelocityMap.end()) {
                yError() << LogPrefix << "Link " << linkName << "not found in link angular velocity map";
                angular.fill(0.0);
                return false;
            }

            auto angularVel = icubImpl->linkAngularVelocityMap.find(linkName);
            angular = angularVel->second;
        }

        return true;
    }

    bool getLinkAcceleration(Vector3& linear, Vector3& angular) const override
    {
        if (icubImpl->linkSensorsMap.find(this->m_name)
            == icubImpl->linkSensorsMap.end()) {
            yError() << LogPrefix << "Sensor" << this->m_name << "NOT found";
            linear.fill(0.0);
            angular.fill(0.0);
            return false;
        }

        //TODO: Do a dummy implementation later
        linear.fill(0.0);
        angular.fill(0.0);
        return true;
    }

    // ------------------------
    // Custom utility functions
    // ------------------------
    inline void setStatus(const wearable::sensor::SensorStatus aStatus) { m_status = aStatus; }
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

    // Set gravity vector
    pImpl->world_gravity.zero();
    pImpl->world_gravity.setVal(2, -9.81);

    // Configure clock
    if (!yarp::os::Time::isSystemClock()) {
        yarp::os::Time::useSystemClock();
    }

    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("urdf") && config.find("urdf").isString())) {
        yError() << LogPrefix << "urdf option not found or not valid";
        return false;
    }

    if (!(config.check("floatingBase") && config.find("floatingBase").isString())) {
        yError() << LogPrefix << "floatingBase option not found or not valid";
        return false;
    }

    yarp::os::Bottle wbdHandsFTSensorsGroup = config.findGroup("ft-sensors");
    if (wbdHandsFTSensorsGroup.isNull()) {
        yError() << LogPrefix << "REQUIRED parameter <ft-sensors> NOT found";
        return false;
    }

    yarp::os::Bottle icubJointSensorGroup = config.findGroup("joint-sensors");
    if (icubJointSensorGroup.isNull()) {
        yError() << LogPrefix << "REQUIRED parameter <joint-sensors> NOT found";
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

    // ==========================
    // INITIALIZE THE ROBOT MODEL
    // ==========================

    const std::string urdfFileName = config.find("urdf").asString();

    auto& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string urdfFilePath = rf.findFile(urdfFileName);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("urdf").asString();
        return false;
    }

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << urdfFilePath;
        return false;
    }

    pImpl->floatingBaseName = config.find("floatingBase").asString();

    // Get the model from the loader
    pImpl->robotModel = modelLoader.model();

    // Set the model to kindyn object
    pImpl->kinDynComputations.loadRobotModel(pImpl->robotModel);

    // Initialize joint quantities size
    pImpl->jointPositionsVec.resize(pImpl->robotModel.getNrOfDOFs());
    pImpl->jointVelocitiesVec.resize(pImpl->robotModel.getNrOfDOFs());
    pImpl->jointAccelerationsVec.resize(pImpl->robotModel.getNrOfDOFs());

    // ======================================
    // PARSE FT SENSORS CONFIGURATION OPTIONS
    // ======================================

    pImpl->nSensors =
        wbdHandsFTSensorsGroup.size() - 1; // Set number of sensors equal to number of ft sensors

    for (size_t n = 1; n <= pImpl->nSensors; n++) {
        yarp::os::Bottle* param = wbdHandsFTSensorsGroup.get(n).asList();
        pImpl->ftSensorNames.push_back(param->get(0).asString());
        pImpl->sensorNames.push_back(param->get(0).asString());
        pImpl->ftSensorPortNames.push_back(param->get(1).asString());
    }

    // =========================
    // FT Sensors Initialization
    // =========================

    std::string ft6dPrefix =
        getWearableName() + wearable::Separator + sensor::IForceTorque6DSensor::getPrefix();

    for (size_t n = 0; n < pImpl->nSensors; n++) {

        WrenchPort* wrenchPort = new WrenchPort;

        if (!(wrenchPort->open("/ICub" + pImpl->ftSensorPortNames.at(n) + ":i")
              && yarp::os::Network::connect(pImpl->ftSensorPortNames.at(n),
                                            wrenchPort->getName().c_str()))) {
            yError() << LogPrefix << "Failed to open " << wrenchPort->getName().c_str()
                     << " port, "
                        "or connect to "
                     << pImpl->ftSensorPortNames.at(n);
            return false;
        }

        // Enable the callback of the ports
        wrenchPort->useCallback();

        // Wait to receive first data
        while (wrenchPort->firstRun) {
            yarp::os::Time::delay(1);
        }

        // Push to wrench ports vector
        pImpl->wrenchPortsVector.push_back(wrenchPort);

        // Create the new ft6d sensors
        auto ft6d = std::make_shared<ICubImpl::ICubForceTorque6DSensor>(
            pImpl.get(), ft6dPrefix + pImpl->ftSensorNames[n]);

        pImpl->ftSensorsMap.emplace(ft6dPrefix + pImpl->ftSensorNames[n],
                                    SensorPtr<ICubImpl::ICubForceTorque6DSensor>{ft6d});
    }

    yInfo() << "check";

    // =========================================
    // PARSE JOINT SENSORS CONFIGURATION OPTIONS
    // =========================================

    // Get control board list
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

    yarp::os::Bottle* controlBoardsListParam =
        icubJointSensorGroup.get(controlBoardsListIndex).asList();
    yarp::os::Bottle* controlBoardList = controlBoardsListParam->get(1).asList();

    std::string remotePrefix =
        icubJointSensorGroup.check("remotePrefix", yarp::os::Value(false)).asString();
    std::string localPrefix =
        icubJointSensorGroup.check("localPrefix", yarp::os::Value(false)).asString();

    yInfo() << LogPrefix
            << "Control boards list         : " << controlBoardList->toString().c_str();
    yInfo() << LogPrefix << "Remote control board prefix : " << remotePrefix;
    yInfo() << LogPrefix << "Local control board prefix  : " << localPrefix;

    // ============================
    // Joint Sensors Initialization
    // ============================

    // Resize control board variables
    pImpl->controlBoardNamesVec.resize(controlBoardList->size());
    pImpl->controlBoardJointsVec.resize(controlBoardList->size());
    pImpl->remoteControlBoardsVec.resize(controlBoardList->size());

    // Set control board names
    for (unsigned index = 0; index < controlBoardList->size(); index++) {
        std::string controlBoardName = controlBoardList->get(index).asString();
        pImpl->controlBoardNamesVec.at(index) = controlBoardName;
    }

    // Initialize remote control boards
    size_t boardCount = 0;
    for (const auto& controlBoard : pImpl->controlBoardNamesVec) {
        pImpl->options.put("device", "remote_controlboard");
        pImpl->options.put("remote", remotePrefix + "/" + controlBoard);
        pImpl->options.put("local", localPrefix + "/" + controlBoard);

        // Open remote control board
        pImpl->remoteControlBoardsVec.at(boardCount) = new yarp::dev::PolyDriver;
        pImpl->remoteControlBoardsVec.at(boardCount)->open(pImpl->options);

        if (!pImpl->remoteControlBoardsVec.at(boardCount)->isValid()) {
            yError() << LogPrefix << "Failed to open the remote control board device for "
                     << controlBoard << " part";
            return false;
        }

        // Get encoder interface
        if (!pImpl->remoteControlBoardsVec.at(boardCount)->view(pImpl->iEncoders)
            || !pImpl->iEncoders) {
            yError() << LogPrefix << "Failed to view the IEncoder interface from the "
                     << controlBoard << " remote control board device";
            return false;
        }

        // Get axis info interface
        if (!pImpl->remoteControlBoardsVec.at(boardCount)->view(pImpl->iAxisInfo)
            || !pImpl->iAxisInfo) {
            yError() << LogPrefix << "Failed to view the IAxisInfo interface from the "
                     << controlBoard << " remote control board device";
            return false;
        }

        // Get joint axes from encoder interface
        int remoteControlBoardJoints;
        pImpl->iEncoders->getAxes(&remoteControlBoardJoints);

        // Get axes names and pad to sensor names
        for (int j = 0; j < remoteControlBoardJoints; j++) {
            std::string axisName;
            pImpl->iAxisInfo->getAxisName(j, axisName);
            pImpl->jointSensorNames.push_back(axisName);
            pImpl->sensorNames.push_back(axisName);
        }

        // Update control board joints vector
        pImpl->controlBoardJointsVec.at(boardCount) = remoteControlBoardJoints;

        // Update the number of sensors
        pImpl->nSensors = pImpl->nSensors + remoteControlBoardJoints;

        pImpl->options.clear();
        boardCount++;
    }

    // Get joint sensor prefix
    std::string jointSensorPrefix =
        getWearableName() + wearable::Separator + sensor::IVirtualJointKinSensor::getPrefix();

    for (size_t s = 0; s < pImpl->jointSensorNames.size(); s++) {
        // Create the new joint sensors
        auto jointsensor = std::make_shared<ICubImpl::ICubVirtualJointKinSensor>(
            pImpl.get(), jointSensorPrefix + pImpl->jointSensorNames[s]);

        pImpl->jointSensorsMap.emplace(
            jointSensorPrefix + pImpl->jointSensorNames[s],
            SensorPtr<ICubImpl::ICubVirtualJointKinSensor>{jointsensor});
    }

    // ============================
    // Link Sensors Initialization
    // ============================

    // Set the size of joint variables
    pImpl->jointPositionsVec.resize(pImpl->robotModel.getNrOfDOFs());
    pImpl->jointVelocitiesVec.resize(pImpl->robotModel.getNrOfDOFs());

    // Update the number of sensors
    pImpl->nSensors = pImpl->nSensors + pImpl->robotModel.getNrOfLinks();

    // Get joint sensor prefix
    std::string linkSensorPrefix =
        getWearableName() + wearable::Separator + sensor::IVirtualLinkKinSensor::getPrefix();

    for (size_t l = 0; l < pImpl->robotModel.getNrOfLinks(); l++) {
        //create the new link sensors
        auto linksensor = std::make_shared<ICubImpl::ICubVirtualLinkKinSensor>(
                    pImpl.get(), linkSensorPrefix + pImpl->robotModel.getLinkName(l));

        pImpl->linkSensorsMap.emplace(
                    linkSensorPrefix + pImpl->robotModel.getLinkName(l),
                    SensorPtr<ICubImpl::ICubVirtualLinkKinSensor>{linksensor});
    }

    // Initialize link quantities flag
    pImpl->updatedLinkQuantities = false;

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
        case sensor::SensorType::VirtualJointKinSensor: {
            for (const auto& jointSensor : pImpl->jointSensorsMap) {
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(jointSensor.second));
            }

            break;
        }
        case sensor::SensorType::VirtualLinkKinSensor: {
            for (const auto& linkSensor : pImpl->linkSensorsMap) {
                outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(linkSensor.second));
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
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>&>(
        *pImpl->ftSensorsMap.at(name));
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
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>&>(
        *pImpl->jointSensorsMap.at(name));
}

// ------------
// LINK Sensor
// ------------

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
ICub::getVirtualLinkKinSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->linkSensorsMap.find(name) == pImpl->linkSensorsMap.end()) {
        yError() << LogPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared point to the required sensor
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>&>(
        *pImpl->linkSensorsMap.at(name));
}
