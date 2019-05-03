/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanControlBoard.h"
#include "IHumanState.h"
#include "IHumanDynamics.h"

#include <mutex>
#include <cmath>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>

const std::string DeviceName = "HumanControlBoard";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

class HumanControlBoard::impl
{
public:
    // Attached interface
    hde::interfaces::IHumanState* iHumanState = nullptr;
    hde::interfaces::IHumanDynamics* iHumanDynamics = nullptr;

    mutable std::mutex mtx;

    // Human model
    iDynTree::Model humanModel;

    // Buffered ports
    yarp::os::BufferedPort<yarp::os::Bottle> dynamicsPort;

    // Joint torques port

    // Data variables
    int nJoints;
    std::vector<std::string> jointNameList;

    yarp::sig::Vector jointPositions;
    yarp::sig::Vector jointVelocities;
    yarp::sig::Vector jointAccelerations;

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

    if (!(config.check("urdf") && config.find("urdf").isString())) {
        yError() << LogPrefix << "urdf option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asDouble();
    const std::string urdfFileName = config.find("urdf").asString();

    // ==========================
    // INITIALIZE THE HUMAN MODEL
    // ==========================

    auto& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string urdfFilePath = rf.findFile(urdfFileName);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("urdf").asString();
        return false;
    }

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << urdfFilePath;
        return false;
    }

    // Get the model from the loader
    pImpl->humanModel = modelLoader.model();

    // Initialize data variables
    pImpl->nJoints = pImpl->humanModel.getNrOfJoints();
    pImpl->jointNameList.resize(pImpl->nJoints);

    pImpl->jointPositions.resize(pImpl->nJoints);
    pImpl->jointVelocities.resize(pImpl->nJoints);
    pImpl->jointAccelerations.resize(pImpl->nJoints);
    pImpl->jointTorques.resize(pImpl->nJoints);

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

void HumanControlBoard::run()
{
    // Get data from IHumanState interface
    pImpl->nJoints = pImpl->iHumanState->getNumberOfJoints();
    pImpl->jointNameList = pImpl->iHumanState->getJointNames();

    std::vector<double> jointPositionsInterface = pImpl->iHumanState->getJointPositions();
    std::vector<double> jointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();

    // Get data from IHumanDynamics interface
    // TODO: Acceleration is currently not given from IHumanDynamics interface
    std::vector<double> jointTorquesInterface = pImpl->iHumanDynamics->getJointTorques();

    for (size_t j = 0; j < jointPositionsInterface.size(); j++) {

        std::unique_lock<std::mutex> lock(pImpl->mtx);
        pImpl->jointPositions[j] = jointPositionsInterface.at(j)*(180/M_PI);
        pImpl->jointVelocities[j] = jointVelocitiesInterface.at(j)*(180/M_PI);

        //TODO: setting to zero
        pImpl->jointAccelerations[j] = 0;

        // The joint order from the two interfaces
        // IHumanState and IHumanDynamics are the same
        // Set the joint torques
        pImpl->jointTorques[j] = jointTorquesInterface.at(j);
    }

}

bool HumanControlBoard::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    // Get the device name from the driver
    const std::string deviceName = poly->getValue("device").asString();

    if (deviceName == "human_state_provider" || deviceName == "xsens_human_state_provider") {

        // Attach IHumanState interface from HumanStateProvider
        if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
            yError() << LogPrefix << "Failed to view IHumanState interface from the polydriver";
            return false;
        }

        // Check the interface
        if (pImpl->iHumanState->getNumberOfJoints() == 0
                || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
            yError() << "The IHumanState interface might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
    }

    if (deviceName == "human_dynamics_estimator") {
        // Attach IHumanDynamics interfaces coming from HumanDynamicsEstimator
        if (pImpl->iHumanDynamics || !poly->view(pImpl->iHumanDynamics) || !pImpl->iHumanDynamics) {
            yError() << LogPrefix << "Failed to view IHumanDynamics interface from the polydriver";
            return false;
        }

        // Check the interface
        if (pImpl->iHumanDynamics->getNumberOfJoints() != 0
            && (pImpl->iHumanDynamics->getNumberOfJoints() != pImpl->iHumanDynamics->getJointNames().size())) {
            yError() << LogPrefix << "The IHumanDynamics interface is not valid."
                     << "The number of joints should match the number of joint names.";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
    }

    return true;
}

bool HumanControlBoard::detach()
{
    askToStop();
    pImpl->iHumanState = nullptr;
    pImpl->iHumanDynamics = nullptr;
    return true;
}

bool HumanControlBoard::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    bool attachStatus = true;
    if (driverList.size() > 2) {
        yError() << LogPrefix << "This wrapper accepts only two attached PolyDriver";
        return false;
    }

    for (size_t i = 0; i < driverList.size(); i++) {
        const yarp::dev::PolyDriverDescriptor* driver = driverList[i];

        if (!driver) {
            yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
            return false;
        }

        attachStatus = attachStatus && attach(driver->poly);
    }

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (attachStatus && !start()) {
        yError() << LogPrefix << "Failed to start the loop";
        return false;
    }

    return attachStatus;
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
bool HumanControlBoard::getAxes(int* ax)
{
    if (!ax)
        return false;
    *ax = pImpl->nJoints;
    return true;
}

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
