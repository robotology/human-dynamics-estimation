/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanRobotPosePublisher.h"

#include <iDynTree/Core/Transform.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <string>
#include <mutex>

const std::string DeviceName = "HumanRobotPosePublisher";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::publishers;

class HumanRobotPosePublisher::impl
{
public:

    mutable std::mutex mutex;

    yarp::dev::PolyDriver transformClientDevice;
    yarp::dev::IFrameTransform* iFrameTransform = nullptr;

    // Human Robot fixed transform
    iDynTree::Transform humanRobotFixedTransform;
    yarp::sig::Matrix robotLeftFoot_H_humanLeftFoot;

    std::string robotTFPrefix;
    std::string robotFloatingBaseFrame;
    std::string robotLeftFootFrame;

    std::string humanFloatingBaseFrame;
    std::string humanLeftFootFrame;

    yarp::os::Node node = {"/" + DeviceName};
};

HumanRobotPosePublisher::HumanRobotPosePublisher()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanRobotPosePublisher::~HumanRobotPosePublisher() = default;

// TODO: This function is also used in HumanWrenchProvider.
//       This should be moved to some single file like utils
bool parseRotation(yarp::os::Bottle* list, iDynTree::Rotation& rotation)
{
    if (list->size() != 9) {
        yError() << LogPrefix << "The list with rotation data does not contain 9 elements";
        return false;
    }

    rotation = iDynTree::Rotation(list->get(0).asDouble(),
                                  list->get(1).asDouble(),
                                  list->get(2).asDouble(),
                                  list->get(3).asDouble(),
                                  list->get(4).asDouble(),
                                  list->get(5).asDouble(),
                                  list->get(6).asDouble(),
                                  list->get(7).asDouble(),
                                  list->get(8).asDouble());
    return true;
}

bool parsePosition(yarp::os::Bottle* list, iDynTree::Position& position)
{
    if (list->size() != 3) {
        yError() << LogPrefix << "The list with position data does not contain 9 elements";
        return false;
    }

    position = iDynTree::Position(
        list->get(0).asDouble(), list->get(1).asDouble(), list->get(2).asDouble());
    return true;
}

bool HumanRobotPosePublisher::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("robotURDFFileName") && config.find("robotURDFFileName").isString())) {
        yError() << LogPrefix << "robotURDFFileName option not found or not valid";
        return false;
    }

    if (!(config.check("robotFloatingBaseFrame") && config.find("robotFloatingBaseFrame").isString())) {
        yError() << LogPrefix << "robotFloatingBaseFrame option not found or not valid";
        return false;
    }

    if (!(config.check("robotLeftFootFrame") && config.find("robotLeftFootFrame").isString())) {
        yError() << LogPrefix << "robotLeftFootFrame option not found or not valid";
        return false;
    }

    if (!(config.check("robotTFPrefix") && config.find("robotTFPrefix").isString())) {
        yError() << LogPrefix << "robotTFPrefix option not found or not valid";
        return false;
    }

    if (!(config.check("humanFloatingBaseFrame") && config.find("humanFloatingBaseFrame").isString())) {
        yError() << LogPrefix << "humanFloatingBaseFrame option not found or not valid";
        return false;
    }

    if (!(config.check("humanLeftFootFrame") && config.find("humanLeftFootFrame").isString())) {
        yError() << LogPrefix << "humanLeftFootFrame option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    const std::string robotURDFFileName = config.find("robotURDFFileName").asString();

    pImpl->robotFloatingBaseFrame = config.find("robotFloatingBaseFrame").asString();
    pImpl->robotLeftFootFrame = config.find("robotLeftFootFrame").asString();
    pImpl->robotTFPrefix = config.find("robotTFPrefix").asString();

    pImpl->humanFloatingBaseFrame = config.find("humanFloatingBaseFrame").asString();
    pImpl->humanLeftFootFrame = config.find("humanLeftFootFrame").asString();

    yarp::os::Bottle& fixedTransformGroup = config.findGroup("HumanRobotLeftFootFixedTransform");

    if (fixedTransformGroup.isNull()) {
        yError() << LogPrefix << "Failed to find HumanRobotLeftFootFixedTransform group";
        return false;
    }

    if (!(fixedTransformGroup.check("rotation") && fixedTransformGroup.find("rotation").isList())) {
        yError() << LogPrefix << "rotation not found or not a valid list";
        return false;
    }

    if (!(fixedTransformGroup.check("position") && fixedTransformGroup.find("position").isList())) {
        yError() << LogPrefix << "position not found or not a valid list";
        return false;
    }

    iDynTree::Rotation rotation;
    iDynTree::Position position;
    if (!parseRotation(fixedTransformGroup.find("rotation").asList(), rotation)
        || !parsePosition(fixedTransformGroup.find("position").asList(), position)) {
        yError() << LogPrefix << "Failed to parse position or rotation";
        return false;
    }

    // Store the transform in the temporary object
    pImpl->humanRobotFixedTransform = {rotation, position};
    iDynTree::toYarp(pImpl->humanRobotFixedTransform.asHomogeneousTransform(), pImpl->robotLeftFoot_H_humanLeftFoot);

    yInfo() << LogPrefix << "*** ========================================";
    yInfo() << LogPrefix << "*** Period                                 :" << period;
    yInfo() << LogPrefix << "*** Robot TF prefix                        :" << pImpl->robotTFPrefix;
    yInfo() << LogPrefix << "*** Robot floating base frame              :" << pImpl->robotFloatingBaseFrame;
    yInfo() << LogPrefix << "*** Robot left foot frame                  :" << pImpl->robotLeftFootFrame;
    yInfo() << LogPrefix << "*** Human floating base frame              :" << pImpl->humanFloatingBaseFrame;
    yInfo() << LogPrefix << "*** Human left foot frame                  :" << pImpl->humanLeftFootFrame;
    yInfo() << LogPrefix << "*** Human robot left foot fixed transform  :";
    yInfo() << pImpl->humanRobotFixedTransform.toString();
    yInfo() << LogPrefix << "*** ========================================";

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

    // Check if transforms are available in the transformServer for the input frames from the config file
    bool ok = pImpl->iFrameTransform->canTransform(pImpl->robotTFPrefix + "/" + pImpl->robotFloatingBaseFrame,
                                                   pImpl->robotTFPrefix + "/" + pImpl->robotLeftFootFrame);
    ok = ok && pImpl->iFrameTransform->canTransform(pImpl->humanLeftFootFrame, pImpl->humanFloatingBaseFrame);

    if (!ok) {
        yError() << LogPrefix << "tranforms do not exist for the given frames in the transformServer";
        return false;
    }

    // ===============
    // Periodic Thread
    // ===============

    setPeriod(period);

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    yInfo() << LogPrefix << "Configured correctly, running thread";
    return true;
}

bool HumanRobotPosePublisher::close()
{
    pImpl->node.interrupt();

    return true;
}

void HumanRobotPosePublisher::run()
{
    // Read the homogeneous tf from ground to human left foot using IFrameTransform interface
    yarp::sig::Matrix humanLeftFoot_H_humanBase;
    pImpl->iFrameTransform->getTransform(pImpl->humanLeftFootFrame, pImpl->humanFloatingBaseFrame, humanLeftFoot_H_humanBase);

    // Read the homogenous tf from robot left foot to robot base using IFrameTransform interface
    yarp::sig::Matrix robotBase_H_robotLeftFoot;
    pImpl->iFrameTransform->getTransform(pImpl->robotTFPrefix + "/" + pImpl->robotFloatingBaseFrame,
                                         pImpl->robotTFPrefix + "/" + pImpl->robotLeftFootFrame, robotBase_H_robotLeftFoot);

    // Compute groud to robot base frame transform
    yarp::sig::Matrix robotBase_H_humanBase;
    robotBase_H_humanBase = robotBase_H_robotLeftFoot * pImpl->robotLeftFoot_H_humanLeftFoot * humanLeftFoot_H_humanBase;

    // Send the final transform to transformServer
    pImpl->iFrameTransform->setTransform(pImpl->robotTFPrefix + "/" + pImpl->robotFloatingBaseFrame,
                                         pImpl->humanFloatingBaseFrame, robotBase_H_humanBase);
}


