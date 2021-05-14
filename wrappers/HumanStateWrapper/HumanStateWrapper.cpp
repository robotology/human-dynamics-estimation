/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStateWrapper.h"
#include <IHumanState.h>
#include <HumanDynamicsEstimation/HumanState.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

const std::string DeviceName = "HumanStateWrapper";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::wrappers;

class HumanStateWrapper::impl
{
public:
    hde::interfaces::IHumanState* iHumanState = nullptr;
    yarp::os::BufferedPort<human::HumanState> outputPort;

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

HumanStateWrapper::HumanStateWrapper()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanStateWrapper::~HumanStateWrapper() {}

bool HumanStateWrapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isDouble())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("outputPort") && config.find("outputPort").isString())) {
        yError() << LogPrefix << "outputPort option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asDouble();
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

bool HumanStateWrapper::close()
{
    return true;
}

void HumanStateWrapper::run()
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
    human::HumanState& humanStateData = pImpl->outputPort.prepare();

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

bool HumanStateWrapper::attach(yarp::dev::PolyDriver* poly)
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

    if (pImpl->iHumanState->getNumberOfJoints() == 0
        || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
        yError() << "The IHumanState interface might not be ready";
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

    return true;
}

void HumanStateWrapper::threadRelease() {}

bool HumanStateWrapper::detach()
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

bool HumanStateWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool HumanStateWrapper::detachAll()
{
    return detach();
}
