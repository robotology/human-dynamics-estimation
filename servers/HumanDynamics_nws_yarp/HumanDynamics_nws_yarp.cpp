// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanDynamics_nws_yarp.h"
#include <hde/interfaces/IHumanDynamics.h>
#include <hde/msgs/HumanDynamics.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <mutex>

const std::string DeviceName = "HumanDynamics_nws_yarp";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::wrappers;

class HumanDynamics_nws_yarp::impl
{
public:
    mutable std::mutex mutex;
    hde::interfaces::IHumanDynamics* humanDynamics = nullptr;
    yarp::os::BufferedPort<hde::msgs::HumanDynamics> outputPort;
};

HumanDynamics_nws_yarp::HumanDynamics_nws_yarp()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanDynamics_nws_yarp::~HumanDynamics_nws_yarp() {}

bool HumanDynamics_nws_yarp::open(yarp::os::Searchable& config)
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

bool HumanDynamics_nws_yarp::close()
{
    pImpl->outputPort.close();
    return true;
}

void HumanDynamics_nws_yarp::run()
{
    // Get data from the interface
    std::vector<double> jointTorques = pImpl->humanDynamics->getJointTorques();
    std::vector<std::string> jointNames = pImpl->humanDynamics->getJointNames();

    // Prepare the message
    hde::msgs::HumanDynamics& humanDynamicsData = pImpl->outputPort.prepare();

    // Convert the joint names
    humanDynamicsData.jointNames.resize(jointTorques.size());
    for (unsigned i = 0; i < jointTorques.size(); ++i) {
        humanDynamicsData.jointNames[i] = jointNames[i];
    }

    // Convert the joint torques
    humanDynamicsData.torques.resize(jointTorques.size());
    for (unsigned i = 0; i < jointTorques.size(); ++i) {
        humanDynamicsData.torques[i] = jointTorques[i];
    }

    // Send the data
    pImpl->outputPort.write();
}

bool HumanDynamics_nws_yarp::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->humanDynamics || !poly->view(pImpl->humanDynamics) || !pImpl->humanDynamics) {
        yError() << LogPrefix << "Failed to view the IHumanDynamics interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    while (pImpl->humanDynamics->getNumberOfJoints() == 0
        || pImpl->humanDynamics->getNumberOfJoints()
               != pImpl->humanDynamics->getJointNames().size()) {
        yInfo() << LogPrefix << "IHumanDynamics interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
    }

    yDebug() << LogPrefix << "Read" << pImpl->humanDynamics->getNumberOfJoints() << "joints";

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

void HumanDynamics_nws_yarp::threadRelease() {}

bool HumanDynamics_nws_yarp::detach()
{
    while (isRunning()) {
        stop();
    }

    pImpl->humanDynamics = nullptr;
    return true;
}

bool HumanDynamics_nws_yarp::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool HumanDynamics_nws_yarp::detachAll()
{
    return detach();
}
