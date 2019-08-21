/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStatePublisher.h"
#include "IHumanState.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Publisher.h>
#include <yarp/rosmsg/TickTime.h>
#include <yarp/rosmsg/sensor_msgs/JointState.h>
#include <yarp/rosmsg/tf2_msgs/TFMessage.h>
#include <yarp/dev/IFrameTransform.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/Transform.h>

#include <array>
#include <string>
#include <vector>

const std::string DeviceName = "HumanStatePublisher";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::publishers;

yarp::rosmsg::TickTime getTimeStampFromYarp();

struct HumanJointStatePublisherResources
{
    size_t counter = 0;
    yarp::rosmsg::sensor_msgs::JointState message;
    yarp::os::Publisher<yarp::rosmsg::sensor_msgs::JointState> publisher;
};

struct BasePosePublisherResources
{
    size_t counter = 0;
    yarp::rosmsg::tf2_msgs::TFMessage message;
    yarp::os::Publisher<yarp::rosmsg::tf2_msgs::TFMessage> publisher;
};

struct HumanStateBuffers
{
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<std::string> names;
    std::array<double, 3> basePosition;
    std::array<double, 4> baseOrientation;
};

class HumanStatePublisher::impl
{
public:
    yarp::dev::PolyDriver transformClientDevice;
    yarp::dev::IFrameTransform* iFrameTransform = nullptr;

    yarp::sig::Matrix humanBase_H_ground;

    hde::interfaces::IHumanState* humanState = nullptr;

    bool firstRun = true;
    bool fixBasePosition = false;
    bool fixBaseOrientation = false;

    // Base offset variables
    yarp::os::Bottle *basePositionOffset = nullptr;
    yarp::os::Bottle *baseOrientationOffset = nullptr;

    std::string baseTFName;

    // Buffers
    HumanStateBuffers humanStateBuffers;

    // ROS Publishers
    yarp::os::Node* node = nullptr;
    BasePosePublisherResources humanBasePoseROS;
    HumanJointStatePublisherResources humanJointStateROS;
};

HumanStatePublisher::HumanStatePublisher()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanStatePublisher::~HumanStatePublisher() = default;

bool HumanStatePublisher::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // GENERAL OPTIONS
    // ===============

    bool useDefaultPeriod = false;
    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period: " << DefaultPeriod << "s";
        useDefaultPeriod = true;
    }

    yarp::os::Bottle* fixedBasePosition;
    if (config.check("fixBasePosition")) {
        if (config.find("fixBasePosition").isList() &&
                config.find("fixBasePosition").asList()->size() == 3) {
            pImpl->fixBasePosition = true;
            fixedBasePosition = config.find("fixBasePosition").asList();
            yInfo() << LogPrefix << "Using a fixed position for the base frame: "
                    << fixedBasePosition;
        }
        else {
            yError() << LogPrefix << "Parameter 'fixBasePosition' invalid";
            return false;
        }
    }

    yarp::os::Bottle* fixedBaseOrientation;
    if (config.check("fixBaseOrientation")) {
        if (config.find("fixBaseOrientation").isList() &&
                config.find("fixBaseOrientation").asList()->size() == 4) {
            pImpl->fixBaseOrientation = true;
            fixedBaseOrientation = config.find("fixBaseOrientation").asList();
            yInfo() << LogPrefix << "Using a fixed orientation for the base frame: "
                    << fixedBaseOrientation;
        }
        else {
            yError() << LogPrefix << "Parameter 'fixBaseOrientation' invalid (required quaternion)";
            return false;
        }
    }

    // Check and parse base position offset
    if (config.check("basePositionOffset")) {
        if (config.find("basePositionOffset").isList() &&
                config.find("basePositionOffset").asList()->size() == 3) {
            pImpl->basePositionOffset = new yarp::os::Bottle(*config.find("basePositionOffset").asList());
            yInfo() << LogPrefix << "Using base position offset: " << pImpl->basePositionOffset->toString().c_str();
        }
        else {
            yError() << LogPrefix << "Parameter 'basePositionOffset' invalid";
            return false;
        }
    }
    else {
        // Set default base position offset
        pImpl->basePositionOffset = new yarp::os::Bottle("0.0 0.0 0.0");
    }

    // Check and parse base orientation offset
    if (config.check("baseOrientationOffset")) {
        if (config.find("baseOrientationOffset").isList() &&
                config.find("baseOrientationOffset").asList()->size() == 4) {
            pImpl->baseOrientationOffset = new yarp::os::Bottle(*config.find("baseOrientationOffset").asList());
            yInfo() << LogPrefix << "Using base orientation offset: " << pImpl->baseOrientationOffset->toString().c_str();
        }
        else {
            yError() << LogPrefix << "Parameter 'baseOrientationOffset' invalid (required quaternion)";
            return false;
        }
    }
    else {
        // Set default base orientation
        pImpl->baseOrientationOffset = new yarp::os::Bottle("0.0 0.0 0.0 0.0");
    }

    // ROS TOPICS
    // ==========

    if (!(config.check("baseTFName") && config.find("baseTFName").isString())) {
        yError() << LogPrefix << "Parameter 'baseTFName' missing or invalid";
        return false;
    }

    if (!(config.check("humanJointsTopic") && config.find("humanJointsTopic").isString())) {
        yError() << LogPrefix << "Parameter 'humanJointsTopic' missing or invalid";
        return false;
    }

    // PORT PREFIX
    // ==========
    bool hasPortPrefix = false;
    if (config.check("portprefix")) {
        if (!config.find("portprefix").isString()) {
            yError() << LogPrefix << "Parameter 'portprefix' is invalid";
            return false;
        }
        hasPortPrefix = true;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    float period = DefaultPeriod;
    if (!useDefaultPeriod) {
        period = config.find("period").asFloat64();
    }

    pImpl->baseTFName = config.find("baseTFName").asString(); // e.g. /Human/Pelvis
    std::string humanJointsTopicName = config.find("humanJointsTopic").asString();

    std::string portPrefix = "";
    if (hasPortPrefix) {
        portPrefix = config.find("portprefix").asString();
        pImpl->node = new yarp::os::Node({"/" + portPrefix + "/" + DeviceName});
    }
    else {
        pImpl->node = new yarp::os::Node({"/" + DeviceName});
    }

    if (pImpl->fixBasePosition) {
        pImpl->humanStateBuffers.basePosition[0] = fixedBasePosition->get(0).asFloat64();
        pImpl->humanStateBuffers.basePosition[1] = fixedBasePosition->get(1).asFloat64();
        pImpl->humanStateBuffers.basePosition[2] = fixedBasePosition->get(2).asFloat64();
    }

    if (pImpl->fixBaseOrientation) {
        pImpl->humanStateBuffers.baseOrientation[0] = fixedBaseOrientation->get(0).asFloat64();
        pImpl->humanStateBuffers.baseOrientation[1] = fixedBaseOrientation->get(1).asFloat64();
        pImpl->humanStateBuffers.baseOrientation[2] = fixedBaseOrientation->get(2).asFloat64();
        pImpl->humanStateBuffers.baseOrientation[3] = fixedBaseOrientation->get(3).asFloat64();
    }

    yInfo() << LogPrefix << "*** =====================";
    yInfo() << LogPrefix << "*** Period              :" << period;
    if (hasPortPrefix) {
        yInfo() << LogPrefix << "*** Prefix              :" << portPrefix;
    }
    yInfo() << LogPrefix << "*** Base transform name :" << pImpl->baseTFName;
    yInfo() << LogPrefix << "*** Joint topic name    :" << humanJointsTopicName;
    yInfo() << LogPrefix << "*** =====================";

    // =========================
    // INITIALIZE ROS PUBLISHERS
    // =========================

    // Initialize ROS resource for human base pose
    pImpl->humanBasePoseROS.counter = 0;
    pImpl->humanBasePoseROS.message.transforms.resize(1);
    pImpl->humanBasePoseROS.message.transforms[0].header.frame_id = "ground";
    pImpl->humanBasePoseROS.message.transforms[0].child_frame_id = pImpl->baseTFName;

    if (!pImpl->humanBasePoseROS.publisher.topic("/tf")) {
        yError() << LogPrefix << "Failed to create topic /tf";
        return false;
    }

    // Initialize ROS resource for human joint positions
    pImpl->humanJointStateROS.message.header.seq = 0;
    pImpl->humanJointStateROS.message.header.frame_id = "ground";

    if (!pImpl->humanJointStateROS.publisher.topic(humanJointsTopicName)) {
        yError() << LogPrefix << "Failed to create topic" << humanJointsTopicName;
        return false;
    }

    // =========================
    // OPEN THE TRANSFORM CLIENT
    // =========================

    yarp::os::Property options;
    options.put("device", "transformClient");
    options.put("local", "/" + DeviceName + "/transformClient");
    options.put("remote", "/transformServer");

    if (!pImpl->transformClientDevice.open(options)) {
        yError() << LogPrefix << "Failed to open the transformClient device";
        return false;
    }

    if (!pImpl->transformClientDevice.view(pImpl->iFrameTransform)) {
        yError() << "The IFrameTransform is not implemented by the opened device";
        return false;
    }

    setPeriod(period);
    return true;
}

bool HumanStatePublisher::close()
{
    pImpl->humanBasePoseROS.publisher.close();
    pImpl->humanJointStateROS.publisher.close();
    pImpl->node->interrupt();

    return true;
}

void HumanStatePublisher::run()
{
    if (pImpl->firstRun) {
        if (pImpl->humanState->getNumberOfJoints() == 0) {
            // Interface not ready
            return;
        }

        size_t dofs = pImpl->humanState->getNumberOfJoints();
        if (pImpl->humanState->getJointNames().size() != dofs) {
            yError() << LogPrefix << "The IHumanState interface is not valid."
                     << "The number of joints do not match the number of joint names.";
            askToStop();
            return;
        }

        if ((pImpl->humanState->getJointPositions().size() != dofs)
            || (pImpl->humanState->getJointVelocities().size() != dofs)) {
            yError() << LogPrefix << "Read joint positions or velocities do not have a length that"
                     << "matches the number of joints";
            askToStop();
            return;
        }

        // Initialize the buffers of the joint publisher
        pImpl->humanStateBuffers.names.resize(dofs);
        pImpl->humanStateBuffers.positions.resize(dofs, 0);
        pImpl->humanStateBuffers.velocities.resize(dofs, 0);

        // Initialize the containers of the message buffer
        pImpl->humanJointStateROS.message.name.resize(dofs);
        pImpl->humanJointStateROS.message.position.resize(dofs, 0);
        pImpl->humanJointStateROS.message.velocity.resize(dofs, 0);

        yInfo() << LogPrefix << "Run properly initialized";
        pImpl->firstRun = false;
    }

    // ===============================
    // PREPARE JOINT POSITIONS MESSAGE
    // ===============================

    // Get the data from the interface
    pImpl->humanStateBuffers.positions = pImpl->humanState->getJointPositions();
    pImpl->humanStateBuffers.velocities = pImpl->humanState->getJointVelocities();
    pImpl->humanStateBuffers.names = pImpl->humanState->getJointNames();

    // This is the buffer of the message with joint data which will be sent
    auto& jointsMessageBuffer = pImpl->humanJointStateROS.message;

    // Update metadata
    jointsMessageBuffer.header.seq = pImpl->humanJointStateROS.counter++;
    jointsMessageBuffer.header.stamp = getTimeStampFromYarp();

    // Store the data into the message buffer
    for (unsigned i = 0; i < pImpl->humanState->getNumberOfJoints(); ++i) {
        jointsMessageBuffer.name[i] = pImpl->humanStateBuffers.names[i];
        jointsMessageBuffer.position[i] = pImpl->humanStateBuffers.positions[i];
        jointsMessageBuffer.velocity[i] = pImpl->humanStateBuffers.velocities[i];
    }

    // Update the publisher message with the new one
    auto& jointStateMsg = pImpl->humanJointStateROS.publisher.prepare();
    jointStateMsg = jointsMessageBuffer;

    // =================================
    // PREPARE THE BASE POSITION MESSAGE
    // =================================

    // Get the data from the interface if not using fixed values
    if (!pImpl->fixBasePosition) {
        pImpl->humanStateBuffers.basePosition = pImpl->humanState->getBasePosition();
    }
    if (!pImpl->fixBaseOrientation) {
        pImpl->humanStateBuffers.baseOrientation = pImpl->humanState->getBaseOrientation();
    }

    // Add base position offset
    pImpl->humanStateBuffers.basePosition[0] = pImpl->humanStateBuffers.basePosition[0] + pImpl->basePositionOffset->get(0).asFloat64();
    pImpl->humanStateBuffers.basePosition[1] = pImpl->humanStateBuffers.basePosition[1] + pImpl->basePositionOffset->get(1).asFloat64();
    pImpl->humanStateBuffers.basePosition[2] = pImpl->humanStateBuffers.basePosition[2] + pImpl->basePositionOffset->get(2).asFloat64();

    // Add base orientation offset
    pImpl->humanStateBuffers.baseOrientation[0] = pImpl->humanStateBuffers.baseOrientation[0] + pImpl->baseOrientationOffset->get(0).asFloat64();
    pImpl->humanStateBuffers.baseOrientation[1] = pImpl->humanStateBuffers.baseOrientation[1] + pImpl->baseOrientationOffset->get(1).asFloat64();
    pImpl->humanStateBuffers.baseOrientation[2] = pImpl->humanStateBuffers.baseOrientation[2] + pImpl->baseOrientationOffset->get(2).asFloat64();
    pImpl->humanStateBuffers.baseOrientation[3] = pImpl->humanStateBuffers.baseOrientation[3] + pImpl->baseOrientationOffset->get(3).asFloat64();

    // This is the buffer of the message with base data which will be sent.
    // Here we get the handlt to the first (and only) tf which is sent.
    auto& baseMessageBufferTransform = pImpl->humanBasePoseROS.message.transforms[0];

    // Update metadata
    baseMessageBufferTransform.header.seq = pImpl->humanBasePoseROS.counter++;
    baseMessageBufferTransform.header.stamp = getTimeStampFromYarp();

    // Store the data into the message buffer
    baseMessageBufferTransform.transform.translation.x = pImpl->humanStateBuffers.basePosition[0];
    baseMessageBufferTransform.transform.translation.y = pImpl->humanStateBuffers.basePosition[1];
    baseMessageBufferTransform.transform.translation.z = pImpl->humanStateBuffers.basePosition[2];

    baseMessageBufferTransform.transform.rotation.w = pImpl->humanStateBuffers.baseOrientation[0];
    baseMessageBufferTransform.transform.rotation.x = pImpl->humanStateBuffers.baseOrientation[1];
    baseMessageBufferTransform.transform.rotation.y = pImpl->humanStateBuffers.baseOrientation[2];
    baseMessageBufferTransform.transform.rotation.z = pImpl->humanStateBuffers.baseOrientation[3];

    // Update the publisher message with the new one
    auto& baseStateMsg = pImpl->humanBasePoseROS.publisher.prepare();
    baseStateMsg = pImpl->humanBasePoseROS.message;

    // ==================
    // WRITE THE MESSAGES
    // ==================

    pImpl->humanBasePoseROS.publisher.write(/*forceStrict=*/true);
    pImpl->humanJointStateROS.publisher.write(/*forceStrict=*/true);

    // Publish base tf to transform server
    iDynTree::Position basePosition(pImpl->humanStateBuffers.basePosition[0],
                                    pImpl->humanStateBuffers.basePosition[1],
                                    pImpl->humanStateBuffers.basePosition[2]);

    iDynTree::Vector4  quaternion;
    quaternion.setVal(0, pImpl->humanStateBuffers.baseOrientation[0]);
    quaternion.setVal(1, pImpl->humanStateBuffers.baseOrientation[1]);
    quaternion.setVal(2, pImpl->humanStateBuffers.baseOrientation[2]);
    quaternion.setVal(3, pImpl->humanStateBuffers.baseOrientation[3]);

    iDynTree::Rotation baseRotation(iDynTree::Rotation::RotationFromQuaternion(quaternion));

    iDynTree::Transform humanBase_H_ground_transform;
    humanBase_H_ground_transform.setPosition(basePosition);
    humanBase_H_ground_transform.setRotation(baseRotation);

    iDynTree::toYarp(humanBase_H_ground_transform.asHomogeneousTransform(),
                     pImpl->humanBase_H_ground);

    pImpl->iFrameTransform->setTransform(pImpl->baseTFName, "ground", pImpl->humanBase_H_ground);
}

bool HumanStatePublisher::attach(yarp::dev::PolyDriver* poly)
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

    // If it is ready, check that is valid
    if (pImpl->humanState->getNumberOfJoints() != 0
        && (pImpl->humanState->getNumberOfJoints() != pImpl->humanState->getJointNames().size())) {
        yError() << LogPrefix << "The IHumanState interface is not valid."
                 << "The number of joints should match the number of joint names.";
        return false;
    }

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    return true;
}

void HumanStatePublisher::threadRelease()
{}

bool HumanStatePublisher::detach()
{
    while (isRunning()) {
        stop();
    }

    pImpl->humanBasePoseROS.publisher.interrupt();
    pImpl->humanJointStateROS.publisher.interrupt();
    pImpl->humanState = nullptr;
    pImpl->basePositionOffset = nullptr;
    pImpl->baseOrientationOffset = nullptr;
    return true;
}

bool HumanStatePublisher::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool HumanStatePublisher::detachAll()
{
    return detach();
}

yarp::rosmsg::TickTime getTimeStampFromYarp()
{
    yarp::rosmsg::TickTime rosTickTime;
    double yarpTimeStamp = yarp::os::Time::now();

    uint64_t time = static_cast<uint64_t>(yarpTimeStamp * 1000000000UL);
    uint64_t nsec_part = time % 1000000000UL;
    uint64_t sec_part = time / 1000000000UL;

    if (sec_part > UINT_MAX) {
        yWarning() << LogPrefix
                   << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    rosTickTime.sec = static_cast<unsigned>(sec_part);
    rosTickTime.nsec = static_cast<unsigned>(nsec_part);

    return rosTickTime;
}
