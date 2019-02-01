/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanControlBoard.h"
#include "IHumanState.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

const std::string DeviceName = "HumanControlBoard";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

class HumanControlBoard::impl
{
public:
    // Attached interface
    hde::interfaces::IHumanState* humanState = nullptr;

    // Buffered ports
    yarp::os::BufferedPort<yarp::os::Bottle> dynamicsPort;

    // Data variables
    int nJoints;
    std::vector<std::string> jointNameList;

    yarp::sig::Vector jointPositions;
    yarp::sig::Vector jointVelocities;
    yarp::sig::Vector jointAccelerations;

    yarp::sig::Vector jointPositionsRad;
    yarp::sig::Vector jointVelocitiesRad;
    yarp::sig::Vector jointAccelerationsRad;

    yarp::sig::Vector jointTorques;
};

// =========================
// HUMANCONTROLBOARD DEVICE
// =========================

HumanControlBoard::HumanControlBoard()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanControlBoard::~HumanControlBoard()
{
    detachAll();
}

bool HumanControlBoard::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isDouble())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("dynamicsDataPortName") && config.find("dynamicsDataPortName").isString())) {
        yError() << LogPrefix << "dynamicsDataPortName option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asDouble();
    std::string dynamicsDataPortName = config.find("dynamicsDataPortName").asString();

    // =============
    // OPEN THE PORT
    // =============

    if (!(pImpl->dynamicsPort.open("/HumanControlBoard/dynamics:i")
          && yarp::os::Network::connect(dynamicsDataPortName,
                                        pImpl->dynamicsPort.getName().c_str()))) {
        yError() << LogPrefix << "Failed to open or connect to "
                 << pImpl->dynamicsPort.getName().c_str();
        return false;
    }

    // ================
    // SETUP THE THREAD
    // ================

    setPeriod(period);

    return true;
}

bool HumanControlBoard::close()
{
    pImpl->dynamicsPort.close();
    yarp::os::PeriodicThread::stop();
    detachAll();
    return true;
}

void HumanControlBoard::run() {}

bool HumanControlBoard::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->humanState || !poly->view(pImpl->humanState) || !pImpl->humanState) {
        yError() << LogPrefix << "Failed to view the IHumanState interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    if (pImpl->humanState->getNumberOfJoints() == 0
        || pImpl->humanState->getNumberOfJoints() != pImpl->humanState->getJointNames().size()) {
        yError() << "The IHumanState interface might not be ready";
        return false;
    }

    yDebug() << LogPrefix << "Read" << pImpl->humanState->getNumberOfJoints() << "joints";

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

bool HumanControlBoard::detach()
{
    askToStop();
    pImpl->humanState = nullptr;
    return true;
}

bool HumanControlBoard::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool HumanControlBoard::detachAll()
{
    return detach();
}

// IAxisInfo interface
bool HumanControlBoard::getAxisName(int axis, std::string& name)
{
    if (axis < 0 || static_cast<size_t>(axis) >= pImpl->nJoints)
        return false;

    name = std::string(pImpl->jointNameList.at(axis));
    return true;
}

// IEncoders interface
bool HumanControlBoard::getEncoder(int j, double* v)
{
    if (v && j >= 0 && static_cast<std::size_t>(j) < pImpl->nJoints) {
        *v = pImpl->jointPositions[j];
    }
    return true;
}

bool HumanControlBoard::getEncoders(double* encs)
{
    if (!encs)
        return false;
    for (std::size_t i = 0; i < pImpl->nJoints; i++) {
        encs[i] = pImpl->jointPositions[i];
    }
    return true;
}

bool HumanControlBoard::getEncoderSpeed(int j, double* sp)
{
    if (sp && j >= 0 && static_cast<size_t>(j) < pImpl->nJoints) {
        *sp = pImpl->jointVelocities[j];
        return true;
    }
    return false;
}

bool HumanControlBoard::getEncoderSpeeds(double* spds)
{
    if (!spds)
        return false;
    for (size_t i = 0; i < pImpl->nJoints; ++i) {
        spds[i] = pImpl->jointVelocities[i];
    }
    return true;
}

bool HumanControlBoard::getEncoderAcceleration(int j, double* spds)
{
    if (spds && j >= 0 && static_cast<size_t>(j) < pImpl->nJoints) {
        *spds = pImpl->jointAccelerations[j];
        return true;
    }
    return false;
}

bool HumanControlBoard::getEncoderAccelerations(double* accs)
{
    if (!accs)
        return false;
    for (size_t i = 0; i < pImpl->nJoints; ++i) {
        accs[i] = pImpl->jointAccelerations[i];
    }
    return true;
}

// IPositionControl interface
bool HumanControlBoard::getAxes(int* ax)
{
    if (!ax)
        return false;
    *ax = pImpl->nJoints;
    return true;
}

// ITorqueControl interface
bool HumanControlBoard::getTorque(int j, double* t)
{
    if (t && j >= 0 && static_cast<size_t>(j) < pImpl->nJoints) {
        *t = pImpl->jointTorques[j];
        return true;
    }
    else
        return false;
}

bool HumanControlBoard::getTorques(double* t)
{
    if (!t)
        return false;
    for (size_t j = 0; j < pImpl->nJoints; ++j) {
        t[j] = pImpl->jointTorques[j];
    }
    return true;
}
