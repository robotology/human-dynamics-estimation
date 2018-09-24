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
const std::string logPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::wrappers;

class HumanStateWrapper::impl
{
public:
    hde::interfaces::IHumanState* humanState = nullptr;
    yarp::os::BufferedPort<human::HumanState> outputPort;
};

HumanStateWrapper::HumanStateWrapper()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanStateWrapper::~HumanStateWrapper()
{
    close();
    detachAll();
}

bool HumanStateWrapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isDouble())) {
        yInfo() << logPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("outputPort") && config.find("outputPort").isString())) {
        yError() << logPrefix << "outputPort option not found or not valid";
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
        yError() << logPrefix << "Failed to open port" << outputPortName;
        return false;
    }

    // ================
    // SETUP THE THREAD
    // ================

    setPeriod(period);

    return true;
}

bool HumanStateWrapper::close()
{
    pImpl->outputPort.close();
    return true;
}

void HumanStateWrapper::run()
{
    // Get data from the interface
    std::array<double, 3> basePositionInterface = pImpl->humanState->getBasePosition();
    std::array<double, 4> baseOrientationInterface = pImpl->humanState->getBaseOrientation();
    std::array<double, 6> baseVelocity = pImpl->humanState->getBaseVelocity();
    std::vector<double> jointPositionsInterface = pImpl->humanState->getJointPositions();
    std::vector<double> jointVelocitiesInterface = pImpl->humanState->getJointVelocities();

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

    // Send the data
    pImpl->outputPort.write(/*forceStrict=*/true);
}

bool HumanStateWrapper::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << logPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->humanState || !poly->view(pImpl->humanState) || !pImpl->humanState) {
        yError() << logPrefix << "Failed to view the IHumanState interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    if (pImpl->humanState->getNumberOfJoints() == 0
        || pImpl->humanState->getNumberOfJoints() != pImpl->humanState->getJointNames().size()) {
        yError() << "The IHumanState interface might not be ready";
        return false;
    }

    yDebug() << logPrefix << "Read" << pImpl->humanState->getNumberOfJoints() << "joints";

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << logPrefix << "Failed to start the loop";
        return false;
    }

    return true;
}

bool HumanStateWrapper::detach()
{
    pImpl->humanState = nullptr;
    return true;
}

bool HumanStateWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << logPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << logPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool HumanStateWrapper::detachAll()
{
    return detach();
}
