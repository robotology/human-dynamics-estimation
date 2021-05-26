/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStateRemapper.h"

#include <hde/msgs/HumanState.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <iostream>

const std::string RemapperName = "HumanStateRemapper";
const std::string logPrefix = RemapperName + " :";

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

class HumanStateRemapper::impl
{
public:
    yarp::os::Network network;
    yarp::os::BufferedPort<hde::msgs::HumanState> inputPort;
    bool terminationCall = false;

    // Buffer HumanState variables
    std::vector<std::string> jointNames;
    std::string baseName;
    std::vector<double>  jointPositions;
    std::vector<double>  jointVelocities;

    std::array<double, 3> basePosition;
    std::array<double, 4> baseOrientation;

    std::array<double, 6> baseVelocity;

    std::array<double, 3> CoMPosition;
    std::array<double, 3> CoMVelocity;
};

// ==============
// IWEAR REMAPPER
// ==============

HumanStateRemapper::HumanStateRemapper()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

HumanStateRemapper::~HumanStateRemapper() = default;

bool HumanStateRemapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    if (!(config.check("humanStateDataPort") && config.find("humanStateDataPort").isString())) {
        yError() << logPrefix << "humanStateData option does not exist or it is not a list";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    std::string humanStateDataPortName = config.find("humanStateDataPort").asString();

    // Initialize the network
    // TODO: is this required in every DeviceDriver?
    pImpl->network = yarp::os::Network();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << logPrefix << "YARP server wasn't found active.";
        return false;
    }

    // ==========================
    // CONFIGURE INPUT DATA PORTS
    // ==========================
    yDebug() << logPrefix << "Configuring input data ports";

    pImpl->inputPort.useCallback(*this);
    if (!pImpl->inputPort.open("...")) {
        yError() << logPrefix << "Failed to open port" << humanStateDataPortName;
        return false;
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << logPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(humanStateDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << logPrefix << "Failed to connect " << humanStateDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << logPrefix << "Opened correctly";
    return true;
}

void HumanStateRemapper::threadRelease()
{}

bool HumanStateRemapper::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void HumanStateRemapper::run()
{
    return;
}

void HumanStateRemapper::onRead(hde::msgs::HumanState& humanStateData)
{
    if(!pImpl->terminationCall) {
        pImpl->jointNames = humanStateData.jointNames;
        pImpl->baseName = humanStateData.baseName;

        pImpl->jointPositions = humanStateData.positions;
        pImpl->jointVelocities = humanStateData.velocities;

        pImpl->basePosition = {humanStateData.baseOriginWRTGlobal.x, humanStateData.baseOriginWRTGlobal.y, humanStateData.baseOriginWRTGlobal.z};
        pImpl->baseOrientation = {humanStateData.baseOrientationWRTGlobal.w, humanStateData.baseOrientationWRTGlobal.imaginary.x, humanStateData.baseOrientationWRTGlobal.imaginary.y, humanStateData.baseOrientationWRTGlobal.imaginary.z};

        pImpl->baseVelocity = {humanStateData.baseVelocityWRTGlobal[0], humanStateData.baseVelocityWRTGlobal[1], humanStateData.baseVelocityWRTGlobal[2],
                               humanStateData.baseVelocityWRTGlobal[3], humanStateData.baseVelocityWRTGlobal[4], humanStateData.baseVelocityWRTGlobal[5]};

        pImpl->CoMPosition = {humanStateData.CoMPositionWRTGlobal.x, humanStateData.CoMPositionWRTGlobal.y, humanStateData.CoMPositionWRTGlobal.z};
        pImpl->CoMVelocity = {humanStateData.CoMVelocityWRTGlobal.x, humanStateData.CoMVelocityWRTGlobal.y, humanStateData.CoMVelocityWRTGlobal.z};
    }
}

std::vector<std::string> HumanStateRemapper::getJointNames() const
{
    return pImpl->jointNames;
}

std::string HumanStateRemapper::getBaseName() const
{
    return pImpl->baseName;
}
size_t HumanStateRemapper::getNumberOfJoints() const
{
    return pImpl->jointPositions.size();
}

std::vector<double> HumanStateRemapper::getJointPositions() const
{
    return pImpl->jointPositions;
}

std::vector<double> HumanStateRemapper::getJointVelocities() const
{
    return pImpl->jointVelocities;
}

std::array<double, 3> HumanStateRemapper::getBasePosition() const
{
    return pImpl->basePosition;
}

std::array<double, 4> HumanStateRemapper::getBaseOrientation() const
{
    return pImpl->baseOrientation;
}

std::array<double, 6> HumanStateRemapper::getBaseVelocity() const
{
    return pImpl->baseVelocity;
}

std::array<double, 3> HumanStateRemapper::getCoMPosition() const
{
    return pImpl->CoMPosition;
}

std::array<double, 3> HumanStateRemapper::getCoMVelocity() const
{
    return pImpl->CoMVelocity;
}
