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
#include <yarp/os/RpcServer.h>
#include <yarp/os/PortReader.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <string>
#include <mutex>

const std::string DeviceName = "HumanRobotPosePublisher";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::publishers;

class cmdProcessor : public yarp::os::PortReader {

protected:
    std::mutex mtx;

public:
    bool cmdStatus = false;

    virtual bool read(yarp::os::ConnectionReader& connection) {
        yarp::os::Bottle command, response;
        if (command.read(connection)) {

            if (command.get(0).asString() == "help") {
                response.addString("Enter <set> to set the robot pose correctly");
            }
            else if (command.get(0).asString() == "set") {
                response.addString("Entered command is correct");
                this->mtx.lock();
                this->cmdStatus = true;
                this->mtx.unlock();
            }
            else {
                response.addString("Entered command is incorrect, Enter help to know available commands");
            }
        }
        else
        {
            this->mtx.lock();
            this->cmdStatus = false;
            this->mtx.unlock();
            return false;
        }

        yarp::os::ConnectionWriter *reply = connection.getWriter();

        if (reply != NULL) {
            response.write(*reply);
        }
        else return false;

        return true;

    }

    bool getCommandStatus() {
        std::lock_guard<std::mutex> lock(this->mtx);
        return this->cmdStatus;
    }

    void setCommandStatus(bool status) {
        std::lock_guard<std::mutex> lock(this->mtx);
        this->cmdStatus = status;
    }
};

class HumanRobotPosePublisher::impl
{
public:

    mutable std::mutex mutex;
    bool first_data = false;

    // Timeout
    double tfTimeout;

    yarp::dev::PolyDriver transformClientDevice;
    yarp::dev::IFrameTransform* iFrameTransform = nullptr;

    // Final ground to robot base transform
    yarp::sig::Matrix robotBase_H_ground;

    // Human Robot fixed transform
    iDynTree::Transform robotHumanFixedTransform;
    yarp::sig::Matrix robotLeftFoot_H_humanLeftFoot;

    std::string robotTFPrefix;
    std::string robotFloatingBaseFrame;
    std::string robotLeftFootFrame;

    std::string humanFloatingBaseFrame;
    std::string humanLeftFootFrame;

    yarp::os::Node node = {"/" + DeviceName};

    // Rpc
    cmdProcessor commandPro;
    yarp::os::RpcServer rpcPort;
    bool cmdReceived;
    bool setRobotBasePose();
    bool robotBaseTF;
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

    if (!(config.check("tfTimeout") && config.find("tfTimeout").isFloat64())) {
        yInfo() << LogPrefix << "Using default tfTimeout:" << DefaultPeriod << "s";
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
    pImpl->tfTimeout = config.check("tfTimeout", yarp::os::Value(DefaultPeriod)).asFloat64();
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
    pImpl->robotHumanFixedTransform = {rotation, position};
    iDynTree::toYarp(pImpl->robotHumanFixedTransform.inverse().asHomogeneousTransform(), pImpl->robotLeftFoot_H_humanLeftFoot);

    // Set default ground to robot base transform to be Identity
    iDynTree::toYarp(iDynTree::Transform::Identity().asHomogeneousTransform(), pImpl->robotBase_H_ground);

    // ===================
    // INITIALIZE RPC PORT
    // ===================

    std::string rpcPortName = "/" + DeviceName + "/rcp:i";
    if (!pImpl->rpcPort.open(rpcPortName)) {
        yError() << LogPrefix << "Unable to open rpc port " << rpcPortName;
        return false;
    }

    // Set rpc port reader
    pImpl->rpcPort.setReader(pImpl->commandPro);

    pImpl->cmdReceived = false;
    pImpl->robotBaseTF = false;

    yInfo() << LogPrefix << "*** ========================================";
    yInfo() << LogPrefix << "*** Period                                 :" << period;
    yInfo() << LogPrefix << "*** TF Timeout                             :" << pImpl->tfTimeout;
    yInfo() << LogPrefix << "*** Robot TF prefix                        :" << pImpl->robotTFPrefix;
    yInfo() << LogPrefix << "*** Robot floating base frame              :" << pImpl->robotFloatingBaseFrame;
    yInfo() << LogPrefix << "*** Robot left foot frame                  :" << pImpl->robotLeftFootFrame;
    yInfo() << LogPrefix << "*** Human floating base frame              :" << pImpl->humanFloatingBaseFrame;
    yInfo() << LogPrefix << "*** Human left foot frame                  :" << pImpl->humanLeftFootFrame;
    yInfo() << LogPrefix << "*** Rpc port                               :" << rpcPortName;
    yInfo() << LogPrefix << "*** Human robot left foot fixed transform  :";
    yInfo() << pImpl->robotHumanFixedTransform.toString();
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


    // Check for first data
    pImpl->first_data = false;
    std::string frames;
    while (!pImpl->first_data) {
        if(pImpl->iFrameTransform->allFramesAsString(frames)) {
            pImpl->first_data = true;
            yInfo() << LogPrefix << "first data received";
        }
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
    // Check command status
    if (pImpl->commandPro.getCommandStatus()) {
        pImpl->robotBaseTF = false;
        if (pImpl->setRobotBasePose()) {
            pImpl->robotBaseTF = true;
            pImpl->commandPro.setCommandStatus(false);
            yInfo() << LogPrefix << "Robot ground to base transform set correctly";
        }
    }

    // Stream robot base tf
    if (pImpl->robotBaseTF) {
        if(!pImpl->iFrameTransform->setTransform(pImpl->robotTFPrefix + "/" + pImpl->robotFloatingBaseFrame,
                                                "ground", pImpl->robotBase_H_ground)) {
            yWarning() << LogPrefix << "Failed to set ground to robot base transform";
        }
    }
}

bool HumanRobotPosePublisher::impl::setRobotBasePose()
{

    //iDynTree::toYarp(iDynTree::Transform::Identity().asHomogeneousTransform(), this->robotBase_H_ground);
    //return true;

    //Read the homogeneous tf from ground to human base using IFrameTransform interface
    yarp::sig::Matrix humanBase_H_ground;

    // Read the homogeneous tf from human base to human left foot using IFrameTransform interface
    yarp::sig::Matrix humanLeftFoot_H_humanBase;

    // Read the homogenous tf from robot left foot to robot base using IFrameTransform interface
    yarp::sig::Matrix robotBase_H_robotLeftFoot;

    bool ok;

    /*ok = this->iFrameTransform->canTransform(this->humanFloatingBaseFrame, "ground") &&
         this->iFrameTransform->canTransform(this->humanLeftFootFrame, this->humanFloatingBaseFrame);
         this->iFrameTransform->canTransform(this->robotTFPrefix + "/" + this->robotFloatingBaseFrame,
                                             this->robotTFPrefix + "/" + this->robotLeftFootFrame);*/

    ok = this->iFrameTransform->waitForTransform(this->humanFloatingBaseFrame, "ground", this->tfTimeout) &&
         this->iFrameTransform->waitForTransform(this->humanLeftFootFrame, this->humanFloatingBaseFrame, this->tfTimeout);
         this->iFrameTransform->waitForTransform(this->robotTFPrefix + "/" + this->robotFloatingBaseFrame,
                                                 this->robotTFPrefix + "/" + this->robotLeftFootFrame, this->tfTimeout);

    if (ok) {

        this->iFrameTransform->getTransform(this->humanFloatingBaseFrame, "ground", humanBase_H_ground);

        this->iFrameTransform->getTransform(this->humanLeftFootFrame, this->humanFloatingBaseFrame, humanLeftFoot_H_humanBase);

        this->iFrameTransform->getTransform(this->robotTFPrefix + "/" + this->robotFloatingBaseFrame,
                                             this->robotTFPrefix + "/" + this->robotLeftFootFrame, robotBase_H_robotLeftFoot);

        // Compute groud to robot base frame transform
        robotBase_H_ground = robotBase_H_robotLeftFoot * this->robotLeftFoot_H_humanLeftFoot * humanLeftFoot_H_humanBase * humanBase_H_ground;

        //yInfo() << LogPrefix << "Transformations :";
        //yInfo() << "robotBase_H_robotLeftFoot : ";
        //yInfo() << robotBase_H_robotLeftFoot.toString().c_str();
        //yInfo() << "pImpl->robotLeftFoot_H_humanLeftFoot";
        //yInfo() << pImpl->robotLeftFoot_H_humanLeftFoot.toString().c_str();
        //yInfo() << "humanLeftFoot_H_humanBase";
        //yInfo() << humanLeftFoot_H_humanBase.toString().c_str();
        return true;
    }
    else {
        yWarning() << "Failed to get human or robot transforms from transform server";
        return false;
    }
}

