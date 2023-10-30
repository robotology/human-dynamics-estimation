// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanWrench_nws_yarp.h"
#include <hde/interfaces/IHumanWrench.h>
#include <hde/msgs/HumanWrench.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <mutex>

const std::string DeviceName = "HumanWrench_nws_yarp";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::servers;

class HumanWrench_nws_yarp::impl
{
public:
    mutable std::mutex mutex;
    hde::interfaces::IHumanWrench* humanWrench = nullptr;
    yarp::os::BufferedPort<hde::msgs::HumanWrench> outputPort;
};

HumanWrench_nws_yarp::HumanWrench_nws_yarp()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanWrench_nws_yarp::~HumanWrench_nws_yarp() {}

bool HumanWrench_nws_yarp::open(yarp::os::Searchable& config)
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

bool HumanWrench_nws_yarp::close()
{
    pImpl->outputPort.close();
    return true;
}

void HumanWrench_nws_yarp::run()
{
    // Get data from the interface
    std::vector<std::string> wrenchSourceNames = pImpl->humanWrench->getWrenchSourceNames();
    std::vector<double> wrenches = pImpl->humanWrench->getWrenches();

    // Prepare the message
    hde::msgs::HumanWrench& humanWrenchData = pImpl->outputPort.prepare();


    // Convert the wrench siurce names
    humanWrenchData.wrenchSourceNames.resize(wrenchSourceNames.size());
    for (unsigned i = 0; i < wrenchSourceNames.size(); ++i) {
        humanWrenchData.wrenchSourceNames[i] = wrenchSourceNames[i];
    }

    // Convert the joint torques
    humanWrenchData.wrenches.resize(wrenches.size());
    for (unsigned i = 0; i < wrenches.size(); ++i) {
        humanWrenchData.wrenches[i] = wrenches[i];
    }

    // Send the data
    pImpl->outputPort.write();
}

bool HumanWrench_nws_yarp::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->humanWrench || !poly->view(pImpl->humanWrench) || !pImpl->humanWrench) {
        yError() << LogPrefix << "Failed to view the IHumanWrench interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    auto numberOfWrenchSources = pImpl->humanWrench->getNumberOfWrenchSources();
    while ( numberOfWrenchSources == 0 ||
            numberOfWrenchSources != pImpl->humanWrench->getWrenchSourceNames().size()) {
        yInfo() << LogPrefix << "IHumanWrench interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
        numberOfWrenchSources = pImpl->humanWrench->getNumberOfWrenchSources();
    }

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

void HumanWrench_nws_yarp::threadRelease() {}

bool HumanWrench_nws_yarp::detach()
{
    while (isRunning()) {
        stop();
    }

    pImpl->humanWrench = nullptr;
    return true;
}

bool HumanWrench_nws_yarp::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This server accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool HumanWrench_nws_yarp::detachAll()
{
    return detach();
}
