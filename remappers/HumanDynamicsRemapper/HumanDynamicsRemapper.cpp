// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanDynamicsRemapper.h"

#include <hde/msgs/HumanDynamics.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <iostream>
#include <mutex>

const std::string RemapperName = "HumanDynamicsRemapper";
const std::string logPrefix = RemapperName + " :";

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

class HumanDynamicsRemapper::impl
{
public:
    std::mutex mtx;
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
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    if(!pImpl->terminationCall) {
        pImpl->jointNames = humanDynamicsData.jointNames;

        pImpl->jointTorques = humanDynamicsData.torques;
    }
}

// method of IHumanDynamics interface expose the buffer variables data
std::vector<std::string> HumanDynamicsRemapper::getJointNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->jointNames;
}

size_t HumanDynamicsRemapper::getNumberOfJoints() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->jointTorques.size();
}

std::vector<double> HumanDynamicsRemapper::getJointTorques() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->jointTorques;
}
