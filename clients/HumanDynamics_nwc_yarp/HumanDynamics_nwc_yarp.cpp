// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanDynamics_nwc_yarp.h"

#include <hde/msgs/HumanDynamics.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <iostream>
#include <mutex>

const std::string ClientName = "HumanDynamics_nwc_yarp";
const std::string LogPrefix = ClientName + " :";

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

class HumanDynamics_nwc_yarp::impl
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

// =======================
// IHUMANDYNAMICS CLIENT
// =======================

HumanDynamics_nwc_yarp::HumanDynamics_nwc_yarp()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

HumanDynamics_nwc_yarp::~HumanDynamics_nwc_yarp() = default;

// parsing the configuration file and connect ports
bool HumanDynamics_nwc_yarp::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    // TODO: where to check this port?
    if (!(config.check("humanDynamicsDataPort") && config.find("humanDynamicsDataPort").isString())) {
        yError() << LogPrefix << "humanDynamicsData option does not exist or it is not a list";
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
        yError() << LogPrefix << "YARP server wasn't found active.";
        return false;
    }

    // ==========================
    // CONFIGURE INPUT DATA PORTS
    // ==========================
    yDebug() << LogPrefix << "Configuring input data ports";

    pImpl->inputPort.useCallback(*this);
    if (!pImpl->inputPort.open("...")) {
        yError() << LogPrefix << "Failed to open port" << humanDynamicsDataPortName;
        return false;
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << LogPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(humanDynamicsDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << LogPrefix << "Failed to connect " << humanDynamicsDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << LogPrefix << "Opened correctly";
    return true;
}

void HumanDynamics_nwc_yarp::threadRelease()
{}

bool HumanDynamics_nwc_yarp::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void HumanDynamics_nwc_yarp::run()
{
    return;
}

// data are read from the port and saved in buffer variables
void HumanDynamics_nwc_yarp::onRead(hde::msgs::HumanDynamics& humanDynamicsData)
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    if(!pImpl->terminationCall) {
        pImpl->jointNames = humanDynamicsData.jointNames;

        pImpl->jointTorques = humanDynamicsData.torques;
    }
}

// method of IHumanDynamics interface expose the buffer variables data
std::vector<std::string> HumanDynamics_nwc_yarp::getJointNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->jointNames;
}

size_t HumanDynamics_nwc_yarp::getNumberOfJoints() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->jointTorques.size();
}

std::vector<double> HumanDynamics_nwc_yarp::getJointTorques() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->jointTorques;
}
