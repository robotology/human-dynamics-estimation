// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanState_nwc_yarp.h"

#include <hde/msgs/HumanState.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <iostream>
#include <mutex>

const std::string ClientName = "HumanState_nwc_yarp";
const std::string LogPrefix = ClientName + " :";

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

class HumanState_nwc_yarp::impl
{
public:
    std::mutex mtx;
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

// ====================
// IHUMANSTATE CLIENT
// ====================

HumanState_nwc_yarp::HumanState_nwc_yarp()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

HumanState_nwc_yarp::~HumanState_nwc_yarp() = default;

bool HumanState_nwc_yarp::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    if (!(config.check("humanStateDataPort") && config.find("humanStateDataPort").isString())) {
        yError() << LogPrefix << "humanStateData option does not exist or it is not a list";
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
        yError() << LogPrefix << "YARP server wasn't found active.";
        return false;
    }

    // ==========================
    // CONFIGURE INPUT DATA PORTS
    // ==========================
    yDebug() << LogPrefix << "Configuring input data ports";

    pImpl->inputPort.useCallback(*this);
    if (!pImpl->inputPort.open("...")) {
        yError() << LogPrefix << "Failed to open port" << humanStateDataPortName;
        return false;
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << LogPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(humanStateDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << LogPrefix << "Failed to connect " << humanStateDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << LogPrefix << "Opened correctly";
    return true;
}

void HumanState_nwc_yarp::threadRelease()
{}

bool HumanState_nwc_yarp::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void HumanState_nwc_yarp::run()
{
    return;
}

void HumanState_nwc_yarp::onRead(hde::msgs::HumanState& humanStateData)
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
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

std::vector<std::string> HumanState_nwc_yarp::getJointNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);

    return pImpl->jointNames;
}

std::string HumanState_nwc_yarp::getBaseName() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->baseName;
}
size_t HumanState_nwc_yarp::getNumberOfJoints() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->jointPositions.size();
}

std::vector<double> HumanState_nwc_yarp::getJointPositions() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->jointPositions;
}

std::vector<double> HumanState_nwc_yarp::getJointVelocities() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->jointVelocities;
}

std::array<double, 3> HumanState_nwc_yarp::getBasePosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->basePosition;
}

std::array<double, 4> HumanState_nwc_yarp::getBaseOrientation() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->baseOrientation;
}

std::array<double, 6> HumanState_nwc_yarp::getBaseVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->baseVelocity;
}

std::array<double, 3> HumanState_nwc_yarp::getCoMPosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->CoMPosition;
}

std::array<double, 3> HumanState_nwc_yarp::getCoMVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->CoMVelocity;
}
