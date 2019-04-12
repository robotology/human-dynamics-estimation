/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStateWrapper.h"
#include "IHumanState.h"
#include "thrift/HumanState.h"

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
};

HumanStateWrapper::HumanStateWrapper()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanStateWrapper::~HumanStateWrapper()
{}

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
    yInfo() << logPrefix << "....................................... debug-1";

    return true;
}

#include <iostream>

bool HumanStateWrapper::close()
{
    return true;
}

void HumanStateWrapper::run()
{
    yInfo() << logPrefix << "....................................... debug-2";

    // Get data from the interface
    std::array<double, 3> basePositionInterface = pImpl->iHumanState->getBasePosition();
    std::array<double, 4> baseOrientationInterface = pImpl->iHumanState->getBaseOrientation();
    std::array<double, 6> baseVelocity = pImpl->iHumanState->getBaseVelocity();
    std::vector<double> jointPositionsInterface = pImpl->iHumanState->getJointPositions();
    std::vector<double> jointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();
    std::vector<std::string> jointNames = pImpl->iHumanState->getJointNames();
    std::string baseName = pImpl->iHumanState->getBaseName();

    // Prepare the message
    human::HumanState& humanStateData = pImpl->outputPort.prepare();

    // Convert the base position
    humanStateData.baseOriginWRTGlobal = {
        basePositionInterface[0], basePositionInterface[1], basePositionInterface[2]};

    // Convert the base orientation
    humanStateData.baseOrientationWRTGlobal = {
        baseOrientationInterface[0],
        {baseOrientationInterface[1], baseOrientationInterface[2], baseOrientationInterface[3]}};

    // Convert the base velocity
    humanStateData.baseVelocityWRTGlobal.resize(6);
    humanStateData.baseVelocityWRTGlobal[0] = baseVelocity[0];
    humanStateData.baseVelocityWRTGlobal[1] = baseVelocity[1];
    humanStateData.baseVelocityWRTGlobal[2] = baseVelocity[2];
    humanStateData.baseVelocityWRTGlobal[3] = baseVelocity[3];
    humanStateData.baseVelocityWRTGlobal[4] = baseVelocity[4];
    humanStateData.baseVelocityWRTGlobal[5] = baseVelocity[5];

    // Convert the joint names
    humanStateData.jointNames.resize(jointPositionsInterface.size());
    for (unsigned i = 0; i < jointPositionsInterface.size(); ++i) {
        humanStateData.jointNames[i] = jointNames[i];
    }

    // Convert the joint positions
    humanStateData.positions.resize(jointPositionsInterface.size());
    for (unsigned i = 0; i < jointPositionsInterface.size(); ++i) {
        humanStateData.positions[i] = jointPositionsInterface[i];
    }

    // Convert the joint velocities
    humanStateData.velocities.resize(jointVelocitiesInterface.size());
    for (unsigned i = 0; i < jointVelocitiesInterface.size(); ++i) {
        humanStateData.velocities[i] = jointVelocitiesInterface[i];
    }

    // Store the name of the base link
    humanStateData.baseName = baseName;

    // Send the data
    pImpl->outputPort.write(/*forceStrict=*/true);
    yInfo() << logPrefix << "....................................... debug-3";
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

    yInfo() << pImpl->ihumanState->getNumberOfJoints() << " "
            << pImpl->ihumanState->getJointNames().size();

    // std::vector<std::string> jointNames=pImpl->humanState->getJointNames();
    for (int i = 0; i < pImpl->ihumanState->getJointNames().size(); i++) {
        yInfo() << "Joint name (" << i << "): " << pImpl->ihumanState->getJointNames()[i];
    }

    if (pImpl->ihumanState->getNumberOfJoints() == 0
        || pImpl->ihumanState->getNumberOfJoints() != pImpl->ihumanState->getJointNames().size()) {
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

void HumanStateWrapper::threadRelease()
{}

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
