// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanDynamicsWrapper.h"
#include <hde/interfaces/IHumanDynamics.h>
#include <hde/msgs/HumanDynamics.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <mutex>

const std::string DeviceName = "HumanDynamicsWrapper";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::wrappers;

class HumanDynamicsWrapper::impl
{
public:
    mutable std::mutex mutex;
    hde::interfaces::IHumanDynamics* humanDynamics = nullptr;
    yarp::os::BufferedPort<hde::msgs::HumanDynamics> outputPort;
};

HumanDynamicsWrapper::HumanDynamicsWrapper()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanDynamicsWrapper::~HumanDynamicsWrapper() {}

bool HumanDynamicsWrapper::open(yarp::os::Searchable& config)
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

bool HumanDynamicsWrapper::close()
{
    pImpl->outputPort.close();
    return true;
}

void HumanDynamicsWrapper::run()
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

bool HumanDynamicsWrapper::attach(yarp::dev::PolyDriver* poly)
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

void HumanDynamicsWrapper::threadRelease() {}

bool HumanDynamicsWrapper::detach()
{
    while (isRunning()) {
        stop();
    }

    pImpl->humanDynamics = nullptr;
    return true;
}

bool HumanDynamicsWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool HumanDynamicsWrapper::detachAll()
{
    return detach();
}
