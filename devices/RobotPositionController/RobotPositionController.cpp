/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "RobotPositionController.h"
#include "IHumanState.h"

#include <array>
#include <numeric>
#include <cmath>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/minJerkCtrl.h>

const std::string DeviceName = "RobotPositionController";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

class RobotPositionController::impl
{
public:
    // Attached interface
    hde::interfaces::IHumanState* iHumanState = nullptr;

    // Polydriver device
    std::vector<yarp::dev::PolyDriver*> remoteControlBoards;
    yarp::dev::IEncoders* iEncoders = nullptr;
    yarp::dev::IControlMode* iControlMode = nullptr;
    yarp::dev::IPositionControl* iPosControl = nullptr;
    yarp::dev::IPositionDirect* iPosDirectControl = nullptr;
    yarp::os::Property options;

    double refSpeed;
    std::string controlMode;

    // Joint variables
    std::vector<int> nJointsVectorFromConfig;
    int totalControlBoardJoints;
    std::vector<double> jointPositionsVector;

    // Min jerk trajectory
    double samplingTime;
    double smoothingTime;
    yarp::sig::Vector posDirectRefJointPosVector;
    yarp::sig::Vector posDirectInputJointPosVector;
    std::vector<iCub::ctrl::minJerkTrajGen*> minJerkTrajGeneratorVec;
    std::vector<int> positionDirectLoopCount;

    std::vector<std::string> jointNameListFromConfigControlBoards;
    std::vector<std::string> jointNameListFromHumanState;

    // Motion variable
    bool checkMotion = false;

    bool firstDataCheck = false;
    double jointPosVectorSum = 0; //Variabe to check first zero data
};

// =============================
// ROBOPOSITIONCONTROLLER DEVICE
// =============================

RobotPositionController::RobotPositionController()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

RobotPositionController::~RobotPositionController()
{}

bool RobotPositionController::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isDouble())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("refSpeed") && config.find("refSpeed").isDouble())) {
        yInfo() << LogPrefix << "refSpeed option not found or not valid";
    }

    if (!(config.check("samplingTime") && config.find("samplingTime").isDouble())) {
        yInfo() << LogPrefix << "samplingTime option not found or not valid";
    }

    if (!(config.check("smoothingTime") && config.find("smoothingTime").isDouble())) {
        yInfo() << LogPrefix << "smoothingTime option not found or not valid";
    }

    if (!(config.check("controlMode") && config.find("controlMode").isString())) {
        yError() << LogPrefix << "controlMode option not found or not valid";
        return false;
    }

    if (!(config.check("controlBoardsList") && config.find("controlBoardsList").isList())) {
        yError() << LogPrefix << "controlBoardsList option not found or not valid";
        return false;
    }

    if (!(config.check("remotePrefix") && config.find("remotePrefix").isString())) {
        yError() << LogPrefix << "remotePrefix option not found or not valid";
        return false;
    }

    if (!(config.check("localPrefix") && config.find("localPrefix").isString())) {
        yError() << LogPrefix << "localPrefix option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asDouble();
    pImpl->controlMode = config.find("controlMode").asString();
    pImpl->refSpeed = config.find("refSpeed").asDouble();
    pImpl->samplingTime = config.find("samplingTime").asDouble();
    pImpl->smoothingTime = config.find("smoothingTime").asDouble();
    yarp::os::Bottle* controlBoardsList = config.find("controlBoardsList").asList();
    const std::string remotePrefix  = config.find("remotePrefix").asString();
    const std::string localPrefix  = config.find("localPrefix").asString();

    yInfo() << LogPrefix << "*** ========================";
    yInfo() << LogPrefix << "*** Period                 :" << period;
    yInfo() << LogPrefix << "*** Control mode           :" << pImpl->controlMode;
    yInfo() << LogPrefix << "*** Reference speed        :" << pImpl->refSpeed;
    yInfo() << LogPrefix << "*** Sampling time          :" << pImpl->samplingTime;
    yInfo() << LogPrefix << "*** Smoothing time         :" << pImpl->smoothingTime;
    yInfo() << LogPrefix << "*** Control boards list    :" << controlBoardsList->toString();
    yInfo() << LogPrefix << "*** Remote prefix          :" << remotePrefix;
    yInfo() << LogPrefix << "*** Local prefix           :" << localPrefix;
    yInfo() << LogPrefix << "*** ========================";

    // ====================================
    // INITIALIZE THE REMOTE CONTROL BOARDS
    // ====================================

    std::vector<std::string> controlBoards;

    for (unsigned index = 0; index < controlBoardsList->size(); index++) {
        controlBoards.push_back(controlBoardsList->get(index).asString());
    }

    // Set the size of remote control boards vector
    pImpl->remoteControlBoards.resize(controlBoards.size());
    pImpl->nJointsVectorFromConfig.resize(controlBoards.size());
    pImpl->minJerkTrajGeneratorVec.resize(controlBoards.size());
    pImpl->positionDirectLoopCount.resize(controlBoards.size());

    yInfo() << LogPrefix << "minJerkTrajGeneratorVec size : " << pImpl->minJerkTrajGeneratorVec.size();

    // Open the control boards
    size_t boardCount = 0;
    for (const auto& controlBoard : controlBoards) {
        pImpl->options.put("device", "remote_controlboard");
        pImpl->options.put("remote", remotePrefix + "/" + controlBoard);
        pImpl->options.put("local", localPrefix + "/" + controlBoard);

        pImpl->remoteControlBoards.at(boardCount) = new yarp::dev::PolyDriver;
        pImpl->remoteControlBoards.at(boardCount)->open(pImpl->options);

        if (!pImpl->remoteControlBoards.at(boardCount)->isValid()) {
            yError() << LogPrefix << "Failed to open the remote control board device";
            return false;
        }

        // Get control mode interface
        if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iControlMode) || !pImpl->iControlMode) {
            yError() << LogPrefix << "Failed to view the IControlMode interface from the " << controlBoard << " remote control board device";
            return false;
        }

        // Get encoder interface
        if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iEncoders) || !pImpl->iEncoders) {
            yError() << LogPrefix << "Failed to view the IEncoder interface from the " << controlBoard << " remote control board device";
            return false;
        }

        // Get joint axes from encoder interface
        int remoteControlBoardJoints;
        pImpl->iEncoders->getAxes(&remoteControlBoardJoints);
        yInfo() << LogPrefix << "Remote control board " << controlBoard << " joints : " << remoteControlBoardJoints;

        // Get initial joint positions
        double initEncoderJointPositions[remoteControlBoardJoints];
        pImpl->iEncoders->getEncoders(initEncoderJointPositions);

        yarp::sig::Vector initEncoderJointPositionsVector;
        initEncoderJointPositionsVector.resize(remoteControlBoardJoints);
        initEncoderJointPositionsVector.zero();

        for(int k = 0; k < remoteControlBoardJoints; k++) {
            initEncoderJointPositionsVector[k] = initEncoderJointPositions[k];
        }

        yInfo() << LogPrefix << "Initial encoder values : " << initEncoderJointPositionsVector.toString();

        if (pImpl->controlMode == "position") {

            // Check position control interface
            if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iPosControl) || !pImpl->iPosControl) {
                yError() << LogPrefix << "Failed to view the IPositionControl interface from the " << controlBoard << " remote control board device";
                return false;
            }

            // Set control mode
            for (unsigned i = 0; i < remoteControlBoardJoints; i++) {
                pImpl->iControlMode->setControlMode(i,VOCAB_CM_POSITION);
                pImpl->iPosControl->setRefSpeed(i, pImpl->refSpeed);
            }

        }

        if (pImpl->controlMode == "positionDirect") {

            // Check position control direct interface
            if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iPosDirectControl) || !pImpl->iPosDirectControl) {
                yError() << LogPrefix << "Failed to view the IPositionDirectControl interface from the " << controlBoard << " remote control board device";
                return false;
            }

            // Set control mode
            for (unsigned i = 0; i < remoteControlBoardJoints; i++) {
                pImpl->iControlMode->setControlMode(i,VOCAB_CM_POSITION_DIRECT);
            }

            // Initialize min jerk object pointer
            pImpl->minJerkTrajGeneratorVec.at(boardCount) = new iCub::ctrl::minJerkTrajGen(remoteControlBoardJoints, pImpl->samplingTime, pImpl->smoothingTime);

            // Set min jerk object initial values
            yInfo() << LogPrefix << "Initial Encoder Values : " << initEncoderJointPositionsVector.toString();
            pImpl->minJerkTrajGeneratorVec.at(boardCount)->init(initEncoderJointPositionsVector);

            pImpl->positionDirectLoopCount.at(boardCount) = 0;
        }

        // Get joint names of the control board from configuration
        if (!(config.check(controlBoard) && config.find(controlBoard).isList())) {
            yError() << LogPrefix << "joint list option not found or not valid for " << controlBoard << " control board";
            return false;
        }

        yarp::os::Bottle* jointsList = config.find(controlBoard).asList();

        // Set the number of joints from config file
        pImpl->nJointsVectorFromConfig.at(boardCount) = jointsList->size();
        yInfo() << LogPrefix << controlBoard << " control board joints : " << pImpl->nJointsVectorFromConfig.at(boardCount);

        for (unsigned index = 0; index < jointsList->size(); index++) {
            pImpl->jointNameListFromConfigControlBoards.push_back(jointsList->get(index).asString());
        }

        pImpl->options.clear();
        boardCount++;
    }

    // Compute total joints from control boards
    pImpl->totalControlBoardJoints = std::accumulate(pImpl->nJointsVectorFromConfig.begin(), pImpl->nJointsVectorFromConfig.end(), 0);

    /*if (pImpl->totalControlBoardJoints != pImpl->jointNameListFromConfigControlBoards.size()) {
     yError() << LogPrefix << "Control board joints number and names mismatch";
     return false;
     }*/

    yInfo() << LogPrefix << "Control boards joint names size : " << pImpl->jointNameListFromConfigControlBoards.size();
    yInfo() << LogPrefix << "Total number of joints from control boards : " << pImpl->totalControlBoardJoints;

    return true;
}

bool RobotPositionController::close()
{
    for(auto& board : pImpl->remoteControlBoards) {
        board->close();
    }

    return true;
}

void RobotPositionController::run()
{
    // Get joint positions from iHumanState interface
    pImpl->jointPositionsVector = pImpl->iHumanState->getJointPositions();
    pImpl->jointNameListFromHumanState = pImpl->iHumanState->getJointNames();

    // Check for first data
    pImpl->jointPosVectorSum = std::accumulate(pImpl->jointPositionsVector.begin(), pImpl->jointPositionsVector.end(), 0.0);

    //yInfo() << LogPrefix << "Joint position vector sum : " << pImpl->jointPosVectorSum;

    // TODO: This is not the best way to check
    // This check is to see if the first data read from the  IHumanState interface is all zero angles
    if (pImpl->jointPosVectorSum != 0 && !pImpl->firstDataCheck) {
        pImpl->firstDataCheck = true;
    }

    //yInfo() << LogPrefix << "First data check : " << pImpl->firstDataCheck;

    if (pImpl->firstDataCheck) {

        // Initialize joint position array with a dummy value
        double jointPositionsArray[pImpl->jointNameListFromConfigControlBoards.size()];

        // Set the joint position values array for iPositionControl interface
        for (unsigned controlBoardJointIndex = 0; controlBoardJointIndex < pImpl->jointNameListFromConfigControlBoards.size(); controlBoardJointIndex++) {
            for (unsigned humanStateJointIndex = 0; humanStateJointIndex < pImpl->jointNameListFromHumanState.size(); humanStateJointIndex++) {
                if (pImpl->jointNameListFromConfigControlBoards.at(controlBoardJointIndex) == pImpl->jointNameListFromHumanState.at(humanStateJointIndex)) {
                    jointPositionsArray[controlBoardJointIndex] = pImpl->jointPositionsVector.at(humanStateJointIndex)*(180/M_PI);
                    break;
                }
            }
        }

        // Set the desired joint positions and ask to move
        int jointNumber = 0;
        for (size_t i = 0; i < pImpl->remoteControlBoards.size(); i++) {

            pImpl->remoteControlBoards.at(i)->view(pImpl->iEncoders);

            // Get joints from iEncoder interface
            int joints;
            pImpl->iEncoders->getAxes(&joints);

            // Read joint position through IEncoder interface
            double encoderJointPositions[joints];
            pImpl->iEncoders->getEncoders(encoderJointPositions);

            //yInfo() << LogPrefix << "Control board (" << i << ") number of joints : " << joints;

            if (pImpl->controlMode == "position") {
                pImpl->remoteControlBoards.at(i)->view(pImpl->iPosControl);
            }

            if (pImpl->controlMode == "positionDirect") {
                pImpl->remoteControlBoards.at(i)->view(pImpl->iPosDirectControl);

                // Set the size of references joint positions vector
                pImpl->posDirectRefJointPosVector.resize(joints);
                //pImpl->posDirectRefJointPosVector.zero();

                // Set the size of input joint positions vector
                pImpl->posDirectInputJointPosVector.resize(joints);
                //pImpl->posDirectRefJointPosVector.zero();
            }

            //yInfo() << LogPrefix << "posDirectRefJointPosVector size : " << pImpl->posDirectRefJointPosVector.size();pImpl->minJerkTrajGeneratorVec.at(i)->computeNextValues(pImpl->posDirectRefJointPosVector);
            //yInfo() << LogPrefix << "posDirectInputJointPosVector size : " << pImpl->posDirectInputJointPosVector.size();
            //yInfo() << LogPrefix << "nJointsVectorFromConfig size :" << pImpl->nJointsVectorFromConfig.at(i);

            for (int j = 0; j < joints; j++) {

                if (pImpl->controlMode == "position") {
                    if (j < pImpl->nJointsVectorFromConfig.at(i)) {
                        pImpl->iPosControl->positionMove(j, jointPositionsArray[jointNumber]);
                        yInfo() << LogPrefix << "Joint (" << j << ") reference : " << jointPositionsArray[jointNumber];
                        jointNumber++;
                    }
                }

                if (pImpl->controlMode == "positionDirect") {
                    if (j < pImpl->nJointsVectorFromConfig.at(i))  {
                        pImpl->posDirectRefJointPosVector[j] = jointPositionsArray[jointNumber];
                        jointNumber++;
                    }
                    else {
                        pImpl->posDirectRefJointPosVector[j] = encoderJointPositions[j];
                    }
                    //pImpl->posDirectRefJointPosVector[j] = encoderJointPositions[j];
                }
            }

            if (pImpl->controlMode == "positionDirect") {

                yInfo() << LogPrefix << "posDirectRefJointPosVector size : " << pImpl->posDirectRefJointPosVector.size();
                yInfo() << LogPrefix << pImpl->posDirectRefJointPosVector.toString();

                yInfo() << LogPrefix << "min jerk trajectory smoothing for position direct control...";
                // Call min jerk trajecotry to smooth reference positions
                pImpl->minJerkTrajGeneratorVec.at(i)->computeNextValues(pImpl->posDirectRefJointPosVector);
                pImpl->posDirectInputJointPosVector = pImpl->minJerkTrajGeneratorVec.at(i)->getPos();

                yInfo() << LogPrefix << "posDirectInputJointPosVector size : " << pImpl->posDirectInputJointPosVector.size();
                yInfo() << LogPrefix << pImpl->posDirectInputJointPosVector.toString();

                //if (pImpl->positionDirectLoopCount.at(i) > 20) {
                    pImpl->iPosDirectControl->setPositions(pImpl->posDirectInputJointPosVector.data());
                //}
                yInfo() << LogPrefix << "Loop count : " << pImpl->positionDirectLoopCount.at(i);
                pImpl->positionDirectLoopCount.at(i)++;
            }
        }

        if (pImpl->controlMode == "position") {
            while(!pImpl->checkMotion) {
                pImpl->iPosControl->checkMotionDone(&pImpl->checkMotion);
            }

            pImpl->checkMotion = false;
        }
    }
}

bool RobotPositionController::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
        yError() << LogPrefix << "Failed to view the IHumanState interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    if (pImpl->iHumanState->getNumberOfJoints() == 0
        || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
        yError() << "The IHumanState interface might not be ready";
        return false;
    }

    // Check the joint numbers match
    /*if (pImpl->iHumanState->getNumberOfJoints() != pImpl->totalControlBoardJoints) {
     yError() << "Number of joints mismatch between the control boards and IHumanState interface";
     return false;
     }*/

    yDebug() << LogPrefix << "Read" << pImpl->iHumanState->getNumberOfJoints() << "joints";

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop";
        return false;
    }

    yInfo() << LogPrefix << "attach() successful";
    return true;
}

void RobotPositionController::threadRelease()
{}

bool RobotPositionController::detach()
{
    // Set the position control mode
    for (size_t boardCount = 0; boardCount < pImpl->remoteControlBoards.size(); boardCount++) {

        // Get encoder interface
        if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iEncoders) || !pImpl->iEncoders) {
            yError() << LogPrefix << "Failed to view the IEncoder interface from the (" << boardCount << ") remote control board device";
            return false;
        }

        // Get joint axes from encoder interface
        int remoteControlBoardJoints;
        pImpl->iEncoders->getAxes(&remoteControlBoardJoints);

        // Get control mode interface
        if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iControlMode) || !pImpl->iControlMode) {
            yError() << LogPrefix << "Failed to view the IControlMode interface from the (" << boardCount << ") remote control board device";
            return false;
        }

        // Set control mode
        for (unsigned joint = 0; joint < remoteControlBoardJoints; joint++) {
            pImpl->iControlMode->setControlMode(joint,VOCAB_CM_POSITION);
        }

    }

    while (isRunning()) {
        yarp::os::PeriodicThread::stop();
    }

    pImpl->iHumanState       = nullptr;
    pImpl->iEncoders         = nullptr;
    pImpl->iControlMode      = nullptr;
    pImpl->iPosControl       = nullptr;
    pImpl->iPosDirectControl = nullptr;

    return true;
}

bool RobotPositionController::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool RobotPositionController::detachAll()
{
    return detach();
}
-
