/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanDynamicsRemapper.h"

#include <hde/msgs/HumanDynamics.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <iostream>

const std::string RemapperName = "HumanDynamicsRemapper";
const std::string logPrefix = RemapperName + " :";

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

class HumanDynamicsRemapper::impl
{
public:
    yarp::os::Network network;
    yarp::os::BufferedPort<hde::msgs::HumanDynamics> inputPort;
    bool terminationCall = false;

    // Buffer HumanDynamics variables
    std::vector<std::string> jointNames;
    std::vector<double>  jointTorques;
};

// ==============
// IWEAR REMAPPER
// ==============

HumanDynamicsRemapper::HumanDynamicsRemapper()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

HumanDynamicsRemapper::~HumanDynamicsRemapper() = default;

// parsing the configuration file and connect ports
bool HumanDynamicsRemapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    // TODO: where to check this port?
    if (!(config.check("humanDynamicsDataPort") && config.find("humanDynamicsDataPort").isString())) {
        yError() << logPrefix << "humanDynamicsData option does not exist or it is not a list";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    std::string humanDynamicsDataPortName = config.find("humanDynamicsDataPort").asString();

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
        yError() << logPrefix << "Failed to open port" << humanDynamicsDataPortName;
        return false;
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << logPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(humanDynamicsDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << logPrefix << "Failed to connect " << humanDynamicsDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << logPrefix << "Opened correctly";
    return true;
}

void HumanDynamicsRemapper::threadRelease()
{}

bool HumanDynamicsRemapper::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void HumanDynamicsRemapper::run()
{
    return;
}

// data are read from the port and saved in buffer variables
void HumanDynamicsRemapper::onRead(hde::msgs::HumanDynamics& humanDynamicsData)
{
    if(!pImpl->terminationCall) {
        pImpl->jointNames = humanDynamicsData.jointNames;

        pImpl->jointTorques = humanDynamicsData.torques;
    }
}

// method of IHumanDynamics interface expose the buffer variables data
std::vector<std::string> HumanDynamicsRemapper::getJointNames() const
{
    return pImpl->jointNames;
}

size_t HumanDynamicsRemapper::getNumberOfJoints() const
{
    return pImpl->jointTorques.size();
}

std::vector<double> HumanDynamicsRemapper::getJointTorques() const
{
    return pImpl->jointTorques;
}
