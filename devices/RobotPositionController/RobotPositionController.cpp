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
    std::vector<int> nJointsVector;
    int totalControlBoardJoints;
    std::vector<double> jointPositionsVector;

    std::vector<std::string> jointNameListFromControlBoards;
    std::vector<std::string> jointNameListFromHumanState;

    // Motion variable
    bool checkMotion = false;
};

// =============================
// ROBOPOSITIONCONTROLLER DEVICE
// =============================

RobotPositionController::RobotPositionController()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

RobotPositionController::~RobotPositionController()
{
    detachAll();
}

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
    yarp::os::Bottle* controlBoardsList = config.find("controlBoardsList").asList();
    const std::string remotePrefix  = config.find("remotePrefix").asString();
    const std::string localPrefix  = config.find("localPrefix").asString();

    yInfo() << LogPrefix << "*** ========================";
    yInfo() << LogPrefix << "*** Period                 :" << period;
    yInfo() << LogPrefix << "*** Control mode           :" << pImpl->controlMode;
    yInfo() << LogPrefix << "*** Reference speed        :" << pImpl->refSpeed;
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
    pImpl->nJointsVector.resize(controlBoards.size());

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

        // Get the number of joint from IEncoder interface
        pImpl->iEncoders->getAxes(&pImpl->nJointsVector.at(boardCount));

        if (pImpl->controlMode == "position") {

            // Check position control interface
            if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iPosControl) || !pImpl->iPosControl) {
                yError() << LogPrefix << "Failed to view the IPositionControl interface from the " << controlBoard << " remote control board device";
                return false;
            }

            // Set control mode
            for (unsigned i = 0; i < pImpl->nJointsVector.at(boardCount); i++) {
                pImpl->iControlMode->setControlMode(i,VOCAB_CM_POSITION);
                pImpl->iPosControl->setRefSpeed(i, pImpl->refSpeed);
            }

        }

        else if (pImpl->controlMode == "positionDirect") {

            // Check position control direct interface
            if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iPosDirectControl) || !pImpl->iPosDirectControl) {
                yError() << LogPrefix << "Failed to view the IPositionDirectControl interface from the " << controlBoard << " remote control board device";
                return false;
            }

            // Set control mode
            for (unsigned i = 0; i < pImpl->nJointsVector.at(boardCount); i++) {
                pImpl->iControlMode->setControlMode(i,VOCAB_CM_POSITION_DIRECT);
            }
        }

        // Get joint names of the control board from configuration
        if (!(config.check(controlBoard) && config.find(controlBoard).isList())) {
            yError() << LogPrefix << "joint list option not found or not valid for " << controlBoard << " control board";
            return false;
        }

        yarp::os::Bottle* jointsList = config.find(controlBoard).asList();

        for (unsigned index = 0; index < jointsList->size(); index++) {
            pImpl->jointNameListFromControlBoards.push_back(jointsList->get(index).asString());
        }

        pImpl->options.clear();
        boardCount++;
    }

    // Compute total joints from control boards
    pImpl->totalControlBoardJoints = std::accumulate(pImpl->nJointsVector.begin(), pImpl->nJointsVector.end(), 0);

    if (pImpl->totalControlBoardJoints != pImpl->jointNameListFromControlBoards.size()) {
        yError() << LogPrefix << "Control board joints number and names mismatch";
        return false;
    }

    yInfo() << LogPrefix << "Total number of joints from control boards : " << pImpl->totalControlBoardJoints;

    return true;
}

bool RobotPositionController::close()
{
    for(auto& board : pImpl->remoteControlBoards) {
        board->close();
    }
    yarp::os::PeriodicThread::stop();
    detachAll();
    return true;
}

void RobotPositionController::run()
{
    // Get joint positions from iHumanState interface
    pImpl->jointPositionsVector = pImpl->iHumanState->getJointPositions();
    pImpl->jointNameListFromHumanState = pImpl->iHumanState->getJointNames();

    // Initialize joint position array with a dummy value
    double jointPositionsArray[pImpl->totalControlBoardJoints];

    for (unsigned i = 0; i < pImpl->totalControlBoardJoints; i++) {
        jointPositionsArray[i] = -100000; //this is a dummy value
    }

    // Set the joint position values array for iPositionControl interface
    for (unsigned controlBoardJointIndex = 0; controlBoardJointIndex < pImpl->jointNameListFromControlBoards.size(); controlBoardJointIndex++) {
        for (unsigned humanStateJointIndex = 0; humanStateJointIndex < pImpl->jointNameListFromHumanState.size(); humanStateJointIndex++) {
            if (pImpl->jointNameListFromControlBoards.at(controlBoardJointIndex) == pImpl->jointNameListFromHumanState.at(humanStateJointIndex)) {
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

        if (pImpl->controlMode == "position") {
            pImpl->remoteControlBoards.at(i)->view(pImpl->iPosControl);
        }

        if (pImpl->controlMode == "positionDirect") {
            pImpl->remoteControlBoards.at(i)->view(pImpl->iPosDirectControl);
        }

        for (int j = 0; j < joints; j++) {

            if (pImpl->controlMode == "position") {
                if (jointPositionsArray[jointNumber] != -100000) {
                    pImpl->iPosControl->positionMove(j, jointPositionsArray[jointNumber]);
                }
                else {
                    pImpl->iPosControl->positionMove(j, encoderJointPositions[jointNumber]);
                }
            }

            if (pImpl->controlMode == "positionDirect") {
                if (jointPositionsArray[jointNumber] != -100000) {
                    pImpl->iPosDirectControl->setPosition(j, jointPositionsArray[jointNumber]);
                }
                else {
                    pImpl->iPosDirectControl->setPosition(j, encoderJointPositions[jointNumber]);
                }
            }

            jointNumber++;
        }

    }

    while(!pImpl->checkMotion) {
        pImpl->iPosControl->checkMotionDone(&pImpl->checkMotion);
    }

    pImpl->checkMotion = false;
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
    if (pImpl->iHumanState->getNumberOfJoints() != pImpl->totalControlBoardJoints) {
        yError() << "Number of joints mismatch between the control boards and IHumanState interface";
        return false;
    }

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

bool RobotPositionController::detach()
{
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
