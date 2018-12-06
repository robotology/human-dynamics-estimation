/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanRobotPosePublisher.h"

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/KinDynComputations.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>

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
    yarp::dev::IFrameTransform* iHumanTransform = nullptr;

    // Human Robot fixed transform
    iDynTree::Transform humanRobotFixedTransform;

    // Model variables
    iDynTree::Model robotModel;

    std::string robotTFPrefix;
    std::string robotFloatingBaseFrame;
    std::string robotLeftFootFrame;
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

    if (!(config.check("urdfFileName") && config.find("urdfFileName").isString())) {
        yError() << LogPrefix << "urdfFileName option not found or not valid";
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

    if (!(config.check("humanLeftFootFrame") && config.find("humanLeftFootFrame").isString())) {
        yError() << LogPrefix << "humanLeftFootFrame option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    const std::string urdfModelName = config.find("urdfModelName").asString();
    const std::string urdfFileName = config.find("urdfFileName").asString();

    pImpl->robotFloatingBaseFrame = config.find("robotFloatingBaseFrame").asString();
    pImpl->robotLeftFootFrame = config.find("robotLeftFootFrame").asString();
    pImpl->robotTFPrefix = config.find("robotTFPrefix").asString();
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

    yInfo() << LogPrefix << "*** ========================================";
    yInfo() << LogPrefix << "*** Period                                 :" << period;
    yInfo() << LogPrefix << "*** Urdf file name                         :" << urdfFileName;
    yInfo() << LogPrefix << "*** Robot TF prefix                        :" << pImpl->robotTFPrefix;
    yInfo() << LogPrefix << "*** Robot floating base frame              :" << pImpl->robotFloatingBaseFrame;
    yInfo() << LogPrefix << "*** Robot left foot frame                  :" << pImpl->robotLeftFootFrame;
    yInfo() << LogPrefix << "*** Human left foot frame                  :" << pImpl->humanLeftFootFrame;
    yInfo() << LogPrefix << "*** Human robot left foot fixed transform  :";
    yInfo() << pImpl->humanRobotFixedTransform.toString();
    yInfo() << LogPrefix << "*** ========================================";

    // ==========================
    // INITIALIZE THE ROBOT MODEL
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

    // Get the model from the model loader
    pImpl->robotModel = modelLoader.model();

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

    if (!pImpl->transformClientDevice.view(pImpl->iHumanTransform)) {
        yError() << "The IFrameTransform is not implemented by the opened device";
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
    yInfo() << LogPrefix << "Inside run";
    // Read the tf from ground to human left foot
    yarp::sig::Matrix humanGroundToHumanBaseTF;
    pImpl->iHumanTransform->getTransform(pImpl->humanLeftFootFrame, "ground", humanGroundToHumanBaseTF);

    // Store the received tf in iDynTree transform format
    iDynTree::Transform humanGroundToHumanBaseTransform(iDynTree::Rotation(humanGroundToHumanBaseTF(1,1),
                                                                           humanGroundToHumanBaseTF(1,2),
                                                                           humanGroundToHumanBaseTF(1,3),
                                                                           humanGroundToHumanBaseTF(2,1),
                                                                           humanGroundToHumanBaseTF(2,2),
                                                                           humanGroundToHumanBaseTF(2,3),
                                                                           humanGroundToHumanBaseTF(3,1),
                                                                           humanGroundToHumanBaseTF(3,2),
                                                                           humanGroundToHumanBaseTF(3,4)),
                                                        iDynTree::Position(humanGroundToHumanBaseTF(1,4),
                                                                           humanGroundToHumanBaseTF(2,4),
                                                                           humanGroundToHumanBaseTF(3,4)));

    // Get the transform from robot left foot to root link
    iDynTree::KinDynComputations robotKinDynComp;
    iDynTree::Transform robotLeftFootToBaseTransform = robotKinDynComp.getRelativeTransform(pImpl->robotLeftFootFrame,
                                                                                            pImpl->robotFloatingBaseFrame);

    // Compute groud to robot base frame transform
    iDynTree::Transform humanGroundToRobotBaseTransform;
    humanGroundToRobotBaseTransform = humanGroundToHumanBaseTransform * pImpl->humanRobotFixedTransform * robotLeftFootToBaseTransform;

    yInfo() << LogPrefix << humanGroundToRobotBaseTransform.toString();
    iDynTree::Position finalPosition = humanGroundToRobotBaseTransform.getPosition();
    iDynTree::Rotation finalRotation = humanGroundToRobotBaseTransform.getRotation();

    // Convert iDynTree Transform to yarp Matrix
    yarp::sig::Matrix humanGroundToRobotBaseTF(4,4);

    // Initialize to Identity
    humanGroundToHumanBaseTF.eye();

    // Store position
    humanGroundToHumanBaseTF(1,4) = finalPosition.getVal(0);
    humanGroundToHumanBaseTF(2,4) = finalPosition.getVal(1);
    humanGroundToHumanBaseTF(3,4) = finalPosition.getVal(2);

    // Store rotation
    humanGroundToHumanBaseTF(1,1) = finalRotation.getVal(1,1);
    humanGroundToHumanBaseTF(1,2) = finalRotation.getVal(1,2);
    humanGroundToHumanBaseTF(1,3) = finalRotation.getVal(1,3);

    humanGroundToHumanBaseTF(2,1) = finalRotation.getVal(2,1);
    humanGroundToHumanBaseTF(2,2) = finalRotation.getVal(2,2);
    humanGroundToHumanBaseTF(2,3) = finalRotation.getVal(2,3);

    humanGroundToHumanBaseTF(3,1) = finalRotation.getVal(3,1);
    humanGroundToHumanBaseTF(3,2) = finalRotation.getVal(3,2);
    humanGroundToHumanBaseTF(3,3) = finalRotation.getVal(3,3);

    // Send the final transform to transformServer
    pImpl->iHumanTransform->setTransform(pImpl->robotTFPrefix + "/" + pImpl->robotLeftFootFrame,
                                         "ground", humanGroundToRobotBaseTF);
}


