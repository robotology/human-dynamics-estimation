/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanDynamicsWrapper.h"
#include "IHumanDynamics.h"

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

    // TODO: Currently as only joint torques are streamed,
    // yarp bottle is sent on the output port
    // An ideal is to stream the human::HumanDynamics thrift
    // format message
    yarp::os::BufferedPort<yarp::os::Bottle> outputPort;
};

HumanDynamicsWrapper::HumanDynamicsWrapper()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanDynamicsWrapper::~HumanDynamicsWrapper()
{
    close();
    detachAll();
}

bool HumanDynamicsWrapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isDouble())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("outputPort") && config.find("outputPort").isString())) {
        yError() << LogPrefix << "outputPort option not found or not valid";
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

    // Prepare the human dynamics data bottle
    yarp::os::Bottle& humanDynamicsData = pImpl->outputPort.prepare();

    if (humanDynamicsData.size() != 0) {
        // Clear the human dynamics data bottle
        humanDynamicsData.clear();
    }

    // Add joint torques to human dynamics data bottle
    for (size_t index = 0; index < jointTorques.size(); index++) {
        humanDynamicsData.addDouble(jointTorques.at(index));
    }

    // Send the data
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    pImpl->outputPort.write(/*forceStrict=*/true);
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

    if (pImpl->humanDynamics->getNumberOfJoints() == 0
        || pImpl->humanDynamics->getNumberOfJoints() != pImpl->humanDynamics->getJointNames().size()) {
        yError() << "The IHumanDynamics interface might not be ready";
        return false;
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

bool HumanDynamicsWrapper::detach()
{
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


















