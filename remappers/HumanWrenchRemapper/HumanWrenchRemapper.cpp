// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanWrenchRemapper.h"

#include <hde/msgs/HumanWrench.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <iostream>
#include <mutex>

const std::string RemapperName = "HumanWrenchRemapper";
const std::string LogPrefix = RemapperName + " :";

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

class HumanWrenchRemapper::impl
{
public:
    std::mutex mtx;
    yarp::os::Network network;
    yarp::os::BufferedPort<hde::msgs::HumanWrench> inputPort;
    bool terminationCall = false;

    // Buffer HumanWrench variables
    std::vector<std::string> wrenchSourceNames;
    std::vector<double>  wrenches;
};

// ====================
// HUMANWRENCH REMAPPER
// ====================

HumanWrenchRemapper::HumanWrenchRemapper()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

HumanWrenchRemapper::~HumanWrenchRemapper() = default;

// parsing the configuration file and connect ports
bool HumanWrenchRemapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    // TODO: where to check this port?
    if (!(config.check("humanWrenchDataPort") && config.find("humanWrenchDataPort").isString())) {
        yError() << LogPrefix << "humanWrenchData option does not exist or it is not a list";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    std::string humanWrenchDataPortName = config.find("humanWrenchDataPort").asString();

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
        yError() << LogPrefix << "Failed to open port" << humanWrenchDataPortName;
        return false;
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << LogPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(humanWrenchDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << LogPrefix << "Failed to connect " << humanWrenchDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << LogPrefix << "Opened correctly";
    return true;
}

void HumanWrenchRemapper::threadRelease()
{}

bool HumanWrenchRemapper::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void HumanWrenchRemapper::run()
{
    return;
}

// data are read from the port and saved in buffer variables
void HumanWrenchRemapper::onRead(hde::msgs::HumanWrench& humanWrenchData)
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    if(!pImpl->terminationCall) {
        pImpl->wrenchSourceNames = humanWrenchData.wrenchSourceNames;

        pImpl->wrenches = humanWrenchData.wrenches;
    }
}

// method of IHumanWrench interface expose the buffer variables data
std::vector<std::string> HumanWrenchRemapper::getWrenchSourceNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->wrenchSourceNames;
}

size_t HumanWrenchRemapper::getNumberOfWrenchSources() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->wrenchSourceNames.size();
}

std::vector<double> HumanWrenchRemapper::getWrenches() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->wrenches;
}
