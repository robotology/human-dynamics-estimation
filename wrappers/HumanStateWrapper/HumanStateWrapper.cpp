/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStateWrapper.h"
#include <hde/interfaces/IHumanState.h>
#include <hde/msgs/HumanState.h>

#include <algorithm>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

const std::string DeviceName = "HumanStateWrapper";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::wrappers;

class HumanStateWrapper::impl
{
public:
    hde::interfaces::IHumanState* iHumanState = nullptr;
    yarp::os::BufferedPort<hde::msgs::HumanState> outputPort;

    yarp::os::BufferedPort<yarp::sig::Vector> jointPositionPort;
    yarp::os::BufferedPort<yarp::sig::Vector> jointVelocityPort;
    yarp::os::BufferedPort<yarp::sig::Vector> basePositionPort;
    yarp::os::BufferedPort<yarp::sig::Vector> baseVelocityPort;
    yarp::os::BufferedPort<yarp::sig::Vector> CoMPositionPort;
    yarp::os::BufferedPort<yarp::sig::Vector> CoMVelocityPort;


    // buffer variables
    std::array<double, 3> CoMPositionInterface;
    std::array<double, 3> CoMVelocityInterface;
    std::array<double, 3> basePositionInterface;
    std::array<double, 4> baseOrientationInterface;
    std::array<double, 6> baseVelocity;
    std::vector<double> jointPositionsInterface;
    std::vector<double> jointVelocitiesInterface;
    std::vector<std::string> jointNames;
    std::string baseName;

};

HumanStateWrapper::HumanStateWrapper()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
    , publishJointPositionVector(true)
    , publishJointVelocityVector(true)
    , publishBasePositionVector(true)
    , publishBaseVelocityVector(true)
    , publishCoMPositionVector(true)
    , publishCoMVelocityVector(true)
    , changeJointsOrder(true)
{}

HumanStateWrapper::~HumanStateWrapper() {}

bool HumanStateWrapper::open(yarp::os::Searchable& config)
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

    if (!(config.check("jointPositionPortName")
          && config.find("jointPositionPortName").isString())) {
        yInfo() << LogPrefix << "Not Opening  pure Vector port for publishing robot information";
        publishJointPositionVector = false;
    }

    if (!(config.check("jointVelocityPortName")
          && config.find("jointVelocityPortName").isString())) {
        yInfo() << LogPrefix << "Not Opening  pure Vector port for publishing robot information";
        publishJointVelocityVector = false;
    }

    if (!(config.check("basePositionPortName") && config.find("basePositionPortName").isString())) {
        yInfo() << LogPrefix << "Not Opening  pure Vector port for publishing robot information";
        publishBasePositionVector = false;
    }

    if (!(config.check("baseVelocityPortName") && config.find("baseVelocityPortName").isString())) {
        yInfo() << LogPrefix << "Not Opening  pure Vector port for publishing robot information";
        publishBaseVelocityVector = false;
    }

    if (!(config.check("CoMPositionPortName") && config.find("CoMPositionPortName").isString())) {
        yInfo() << LogPrefix << "Not Opening  pure Vector port for publishing robot information";
        publishCoMPositionVector = false;
    }

    if (!(config.check("CoMVelocityPortName") && config.find("CoMVelocityPortName").isString())) {
        yInfo() << LogPrefix << "Not Opening  pure Vector port for publishing robot information";
        publishCoMVelocityVector = false;
    }
    if (!(config.check("JointsDesiredOrder") && config.find("JointsDesiredOrder").asList())) {
        yInfo() << LogPrefix << "Not Found Desired Joints Order, using default IK Output";
        changeJointsOrder = false;
    }

    // =============
    // OPEN THE PORT
    // =============

    if (!pImpl->outputPort.open(outputPortName)) {
        yError() << LogPrefix << "Failed to open port" << outputPortName;
        return false;
    }

    // =============
    // OPEN THE VECTOR PORT
    // =============
    if (publishJointPositionVector) {

        // Joint position port
        std::string jointPositionPortName = config.find("jointPositionPortName").asString();
        if (!pImpl->jointPositionPort.open(jointPositionPortName)) {
            yError() << LogPrefix << "Failed to open port" << jointPositionPortName;
            return false;
        }
    }

    if (publishJointVelocityVector) {

        // Joint velocity port
        std::string jointVelocityPortName = config.find("jointVelocityPortName").asString();
        if (!pImpl->jointVelocityPort.open(jointVelocityPortName)) {
            yError() << LogPrefix << "Failed to open port" << jointVelocityPortName;
            return false;
        }
    }

    if (publishBasePositionVector) {

        // Base Position port
        std::string basePositionPortName = config.find("basePositionPortName").asString();
        if (!pImpl->basePositionPort.open(basePositionPortName)) {
            yError() << LogPrefix << "Failed to open port" << basePositionPortName;
            return false;
        }
    }

    if (publishBaseVelocityVector) {

        // Base Velocity port
        std::string baseVelocityPortName = config.find("baseVelocityPortName").asString();
        if (!pImpl->baseVelocityPort.open(baseVelocityPortName)) {
            yError() << LogPrefix << "Failed to open port" << baseVelocityPortName;
            return false;
        }
    }

    if (publishCoMPositionVector) {

        // CoM Position port
        std::string CoMPositionPortName = config.find("CoMPositionPortName").asString();
        if (!pImpl->CoMPositionPort.open(CoMPositionPortName)) {
            yError() << LogPrefix << "Failed to open port" << CoMPositionPortName;
            return false;
        }
    }

    if (publishCoMVelocityVector) {

        // CoM Velocity port
        std::string CoMVelocityPortName = config.find("CoMVelocityPortName").asString();
        if (!pImpl->CoMVelocityPort.open(CoMVelocityPortName)) {
            yError() << LogPrefix << "Failed to open port" << CoMVelocityPortName;
            return false;
        }
    }

    // Reading the change joints order list
    if (changeJointsOrder) {
        parseJointsOrder(config.find("JointsDesiredOrder").asList(), jointsNameDesiredOrder);
    }

    // ================
    // SETUP THE THREAD
    // ================

    setPeriod(period);

    return true;
}

#include <iostream>

bool HumanStateWrapper::close()
{
    return true;
}

void HumanStateWrapper::run()
{

    // Get data from the interface
    pImpl->CoMPositionInterface = pImpl->iHumanState->getCoMPosition();
    pImpl->CoMVelocityInterface = pImpl->iHumanState->getCoMVelocity();
    pImpl->basePositionInterface = pImpl->iHumanState->getBasePosition();
    pImpl->baseOrientationInterface = pImpl->iHumanState->getBaseOrientation();
    pImpl->baseVelocity = pImpl->iHumanState->getBaseVelocity();
    pImpl->jointPositionsInterface = pImpl->iHumanState->getJointPositions();
    pImpl->jointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();
    pImpl->jointNames = pImpl->iHumanState->getJointNames();
    pImpl->baseName = pImpl->iHumanState->getBaseName();

    // Prepare the message
    hde::msgs::HumanState& humanStateData = pImpl->outputPort.prepare();

    // Convert the COM position
    humanStateData.CoMPositionWRTGlobal = {
        pImpl->CoMPositionInterface[0], pImpl->CoMPositionInterface[1], pImpl->CoMPositionInterface[2]};

    // Convert the COM velocity
    humanStateData.CoMVelocityWRTGlobal = {
        pImpl->CoMVelocityInterface[0], pImpl->CoMVelocityInterface[1], pImpl->CoMVelocityInterface[2]};

    // Convert the base position
    humanStateData.baseOriginWRTGlobal = {
        pImpl->basePositionInterface[0], pImpl->basePositionInterface[1], pImpl->basePositionInterface[2]};

    // Convert the base orientation
    humanStateData.baseOrientationWRTGlobal = {
        pImpl->baseOrientationInterface[0], {pImpl->baseOrientationInterface[1], pImpl->baseOrientationInterface[2], pImpl->baseOrientationInterface[3]}};

    // Convert the base velocity
    humanStateData.baseVelocityWRTGlobal.resize(6);
    humanStateData.baseVelocityWRTGlobal[0] = pImpl->baseVelocity[0];
    humanStateData.baseVelocityWRTGlobal[1] = pImpl->baseVelocity[1];
    humanStateData.baseVelocityWRTGlobal[2] = pImpl->baseVelocity[2];
    humanStateData.baseVelocityWRTGlobal[3] = pImpl->baseVelocity[3];
    humanStateData.baseVelocityWRTGlobal[4] = pImpl->baseVelocity[4];
    humanStateData.baseVelocityWRTGlobal[5] = pImpl->baseVelocity[5];

    // Convert the joint names
    humanStateData.jointNames.resize(pImpl->jointPositionsInterface.size());
    for (unsigned i = 0; i < pImpl->jointPositionsInterface.size(); ++i) {
        humanStateData.jointNames[i] = pImpl->jointNames[i];
    }

    // Convert the joint positions
    humanStateData.positions.resize(pImpl->jointPositionsInterface.size());
    for (unsigned i = 0; i < pImpl->jointPositionsInterface.size(); ++i) {
        humanStateData.positions[i] = pImpl->jointPositionsInterface[i];
    }

    // Convert the joint velocities
    humanStateData.velocities.resize(pImpl->jointVelocitiesInterface.size());
    for (unsigned i = 0; i < pImpl->jointVelocitiesInterface.size(); ++i) {
        humanStateData.velocities[i] = pImpl->jointVelocitiesInterface[i];
    }

    // Store the name of the base link
    humanStateData.baseName = pImpl->baseName;

    // Send the data

    pImpl->outputPort.write(/*forceStrict=*/true);

    // joint position
    if (publishJointPositionVector) {
        int size_joint = pImpl->jointPositionsInterface.size();
        yarp::sig::Vector& jointPositionsOut = pImpl->jointPositionPort.prepare();
        jointPositionsOut.resize(size_joint);
        std::vector<int> jointsOrderIndex;

        if (changeJointsOrder) {
            //TODO this should be replaced by an unordered map, to decrease the complexity
            if (computeJointsOrderIndex(pImpl->jointNames, jointsNameDesiredOrder, jointsOrderIndex)) {
            }
            else {
                yError() << LogPrefix << "Given Wrong Joints order";
                changeJointsOrder = false;
            }
        }

        for (int j = 0; j < size_joint; j++) {
            if (changeJointsOrder) {
            //TODO this should be replaced by an unordered map, to decrease the complexity
                jointPositionsOut[j] = pImpl->jointPositionsInterface[jointsOrderIndex.at(j)];
            }
            else {
                jointPositionsOut[j] = pImpl->jointPositionsInterface[j];
            }
        }

        pImpl->jointPositionPort.write(/*forceStrict=*/true);
    }

    // joint velocities
    if (publishJointVelocityVector) {

        int size_joint = pImpl->jointPositionsInterface.size();
        yarp::sig::Vector& jointVelocityOut = pImpl->jointVelocityPort.prepare();

        jointVelocityOut.resize(size_joint);
        std::vector<int> jointsOrderIndex;

        if (changeJointsOrder) {
            if (computeJointsOrderIndex(pImpl->jointNames, jointsNameDesiredOrder, jointsOrderIndex)) {
            }
            else {
                yError() << LogPrefix << "Given Wrong Joints order";
                changeJointsOrder = false;
            }
        }

        for (int j = 0; j < size_joint; j++) {
            if (changeJointsOrder) {

                jointVelocityOut[j] = pImpl->jointVelocitiesInterface[jointsOrderIndex.at(j)];
            }
            else {
                jointVelocityOut[j] = pImpl->jointVelocitiesInterface[j];
            }
        }

        pImpl->jointVelocityPort.write(/*forceStrict=*/true);
    }

    // base position
    if (publishBasePositionVector) {

        yarp::sig::Vector& basePositionOut = pImpl->basePositionPort.prepare();
        basePositionOut.resize(7);

        for (int j = 0; j < 3; j++) {
            basePositionOut[j] = pImpl->basePositionInterface[j];
        }

        for (int j = 0; j < 4; j++) {
            basePositionOut[3 + j] = pImpl->baseOrientationInterface[j];
        }

        pImpl->basePositionPort.write(/*forceStrict=*/true);
    }

    // base velocity
    if (publishBaseVelocityVector) {
        yarp::sig::Vector& baseVelocityOut = pImpl->baseVelocityPort.prepare();
        baseVelocityOut.resize(6);

        for (int j = 0; j < 6; j++) {
            baseVelocityOut[j] = pImpl->baseVelocity[j];
        }

        pImpl->baseVelocityPort.write(/*forceStrict=*/true);
    }

    // CoM position
    if (publishCoMPositionVector) {

        yarp::sig::Vector& CoMPositionOut = pImpl->CoMPositionPort.prepare();
        CoMPositionOut.resize(3);

        for (int j = 0; j < 3; j++) {
            CoMPositionOut[j] = pImpl->CoMPositionInterface[j];
        }

        pImpl->CoMPositionPort.write(/*forceStrict=*/true);
    }

    // CoM Velocity
    if (publishCoMVelocityVector) {

        yarp::sig::Vector& CoMVelocityOut = pImpl->CoMVelocityPort.prepare();
        CoMVelocityOut.resize(3);

        for (int j = 0; j < 3; j++) {
            CoMVelocityOut[j] = pImpl->CoMVelocityInterface[j];
        }

        pImpl->CoMVelocityPort.write(/*forceStrict=*/true);
    }
}

bool HumanStateWrapper::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
        yError() << LogPrefix << "Failed to view the IHumanState interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    yInfo() << pImpl->iHumanState->getNumberOfJoints() << " "
            << pImpl->iHumanState->getJointNames().size();

    // std::vector<std::string> jointNames=pImpl->humanState->getJointNames();
    for (int i = 0; i < pImpl->iHumanState->getJointNames().size(); i++) {
        yInfo() << "Joint name (" << i << "): " << pImpl->iHumanState->getJointNames()[i];
    }

    if (pImpl->iHumanState->getNumberOfJoints() == 0
        || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
        yError() << "The IHumanState interface might not be ready";
        return false;
    }

    yDebug() << LogPrefix << "Read" << pImpl->iHumanState->getNumberOfJoints() << "joints";

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

void HumanStateWrapper::threadRelease() {}

bool HumanStateWrapper::detach()
{
    while (isRunning()) {
        stop();
    }

    while (!pImpl->outputPort.isClosed()) {
        pImpl->outputPort.close();
    }
    pImpl->iHumanState = nullptr;

    return true;
}

bool HumanStateWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool HumanStateWrapper::detachAll()
{
    return detach();
}

bool HumanStateWrapper::parseJointsOrder(yarp::os::Bottle* list,
                                         std::vector<std::string>& jointsOrderOut)
{

    for (auto i = 0; i < list->size(); i++) {
        jointsOrderOut.push_back(list->get(i).asString());
    }
    return true;
}

bool HumanStateWrapper::computeJointsOrderIndex(const std::vector<std::string> &jointsOrderIn,
                                                const std::vector<std::string> &jointsOrderDesired,
                                                std::vector<int>& indexJointsOrder)
{

    if (jointsOrderIn.size() == jointsOrderDesired.size()) {
        for (auto i = 0; i < jointsOrderDesired.size(); i++) {
            auto index = find(jointsOrderIn.begin(), jointsOrderIn.end(), jointsOrderDesired.at(i));
            int index_int = index - jointsOrderIn.begin();
            indexJointsOrder.push_back(index_int);
        }
        return true;
    }
    else {
        return false;
    }
}
