// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanState_nws_yarp.h"
#include <hde/interfaces/IHumanState.h>
#include <hde/msgs/HumanState.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

const std::string DeviceName = "HumanState_nws_yarp";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::wrappers;

class HumanState_nws_yarp::impl
{
public:
    hde::interfaces::IHumanState* iHumanState = nullptr;
    yarp::os::BufferedPort<hde::msgs::HumanState> outputPort;

    // buffer variables
    std::array<double, 3> CoMPositionInterface;
    std::array<double, 3> CoMVelocityInterface;
    std::array<double, 3> basePositionInterface;
    std::array<double, 4> baseOrientationInterface;
    std::array<double, 6> baseVelocity;
    std::vector<double> jointPositionsInterface;
    std::vector<double> jointVelocitiesInterface;
    std::vector<std::string> jointNames;
    std::string baseName;
};

HumanState_nws_yarp::HumanState_nws_yarp()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanState_nws_yarp::~HumanState_nws_yarp() {}

bool HumanState_nws_yarp::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("outputPort") && config.find("outputPort").isString())) {
        yError() << LogPrefix << "outputPort option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    std::string outputPortName = config.find("outputPort").asString();

    // =============
    // OPEN THE PORT
    // =============

    if (!pImpl->outputPort.open(outputPortName)) {
        yError() << LogPrefix << "Failed to open port" << outputPortName;
        return false;
    }

    // ================
    // SETUP THE THREAD
    // ================

    setPeriod(period);

    return true;
}

#include <iostream>

bool HumanState_nws_yarp::close()
{
    return true;
}

void HumanState_nws_yarp::run()
{

    // Get data from the interface
    pImpl->CoMPositionInterface = pImpl->iHumanState->getCoMPosition();
    pImpl->CoMVelocityInterface = pImpl->iHumanState->getCoMVelocity();
    pImpl->basePositionInterface = pImpl->iHumanState->getBasePosition();
    pImpl->baseOrientationInterface = pImpl->iHumanState->getBaseOrientation();
    pImpl->baseVelocity = pImpl->iHumanState->getBaseVelocity();
    pImpl->jointPositionsInterface = pImpl->iHumanState->getJointPositions();
    pImpl->jointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();
    pImpl->jointNames = pImpl->iHumanState->getJointNames();
    pImpl->baseName = pImpl->iHumanState->getBaseName();

    // Prepare the message
    hde::msgs::HumanState& humanStateData = pImpl->outputPort.prepare();

    // Convert the COM position
    humanStateData.CoMPositionWRTGlobal = {
        pImpl->CoMPositionInterface[0], pImpl->CoMPositionInterface[1], pImpl->CoMPositionInterface[2]};

    // Convert the COM velocity
    humanStateData.CoMVelocityWRTGlobal = {
        pImpl->CoMVelocityInterface[0], pImpl->CoMVelocityInterface[1], pImpl->CoMVelocityInterface[2]};

    // Convert the base position
    humanStateData.baseOriginWRTGlobal = {
        pImpl->basePositionInterface[0], pImpl->basePositionInterface[1], pImpl->basePositionInterface[2]};

    // Convert the base orientation
    humanStateData.baseOrientationWRTGlobal = {
        pImpl->baseOrientationInterface[0], {pImpl->baseOrientationInterface[1], pImpl->baseOrientationInterface[2], pImpl->baseOrientationInterface[3]}};

    // Convert the base velocity
    humanStateData.baseVelocityWRTGlobal.resize(6);
    humanStateData.baseVelocityWRTGlobal[0] = pImpl->baseVelocity[0];
    humanStateData.baseVelocityWRTGlobal[1] = pImpl->baseVelocity[1];
    humanStateData.baseVelocityWRTGlobal[2] = pImpl->baseVelocity[2];
    humanStateData.baseVelocityWRTGlobal[3] = pImpl->baseVelocity[3];
    humanStateData.baseVelocityWRTGlobal[4] = pImpl->baseVelocity[4];
    humanStateData.baseVelocityWRTGlobal[5] = pImpl->baseVelocity[5];

    // Convert the joint names
    humanStateData.jointNames.resize(pImpl->jointPositionsInterface.size());
    for (unsigned i = 0; i < pImpl->jointPositionsInterface.size(); ++i) {
        humanStateData.jointNames[i] = pImpl->jointNames[i];
    }

    // Convert the joint positions
    humanStateData.positions.resize(pImpl->jointPositionsInterface.size());
    for (unsigned i = 0; i < pImpl->jointPositionsInterface.size(); ++i) {
        humanStateData.positions[i] = pImpl->jointPositionsInterface[i];
    }

    // Convert the joint velocities
    humanStateData.velocities.resize(pImpl->jointVelocitiesInterface.size());
    for (unsigned i = 0; i < pImpl->jointVelocitiesInterface.size(); ++i) {
        humanStateData.velocities[i] = pImpl->jointVelocitiesInterface[i];
    }

    // Store the name of the base link
    humanStateData.baseName = pImpl->baseName;

    // Send the data
    pImpl->outputPort.write(/*forceStrict=*/true);
}

bool HumanState_nws_yarp::attach(yarp::dev::PolyDriver* poly)
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

    yInfo() << pImpl->iHumanState->getNumberOfJoints() << " "
            << pImpl->iHumanState->getJointNames().size();

    // std::vector<std::string> jointNames=pImpl->humanState->getJointNames();
    for (int i = 0; i < pImpl->iHumanState->getJointNames().size(); i++) {
        yInfo() << "Joint name (" << i << "): " << pImpl->iHumanState->getJointNames()[i];
    }

    while (pImpl->iHumanState->getNumberOfJoints() == 0
        || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
        yInfo() << LogPrefix << "IHumanState interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
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

    return true;
}

void HumanState_nws_yarp::threadRelease() {}

bool HumanState_nws_yarp::detach()
{
    while (isRunning()) {
        stop();
    }

    while (!pImpl->outputPort.isClosed()) {
        pImpl->outputPort.close();
    }
    pImpl->iHumanState = nullptr;

    return true;
}

bool HumanState_nws_yarp::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool HumanState_nws_yarp::detachAll()
{
    return detach();
}
