/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanDynamicsPublisher.h"
#include "IHumanDynamics.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/rosmsg/TickTime.h>
#include <yarp/rosmsg/sensor_msgs/Temperature.h>

#include <iDynTree/ModelIO/ModelLoader.h>

#include <array>
#include <string>
#include <vector>

const std::string DeviceName = "HumanDynamicsPublisher";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::publishers;

yarp::rosmsg::TickTime getTimeStampFromYarp();

struct HumanJointTorquesPublisherResources
{
    size_t counter = 0;
    yarp::rosmsg::sensor_msgs::Temperature message;
    yarp::os::Publisher<yarp::rosmsg::sensor_msgs::Temperature> publisher;
};

class HumanDynamicsPublisher::impl
{
public:
    hde::interfaces::IHumanDynamics* humanDynamics = nullptr;

    bool firstRun = true;

    // ROS Publisher
    yarp::os::Node node = {"/" + DeviceName};
    HumanJointTorquesPublisherResources humanJointTorquesROS;

    // Model variables
    iDynTree::Model humanModel;
};

HumanDynamicsPublisher::HumanDynamicsPublisher()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanDynamicsPublisher::~HumanDynamicsPublisher() = default;

bool HumanDynamicsPublisher::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // GENERAL OPTIONS
    // ===============

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period: " << DefaultPeriod << "s";
    }

    if (!(config.check("urdf") && config.find("urdf").isString())) {
        yError() << LogPrefix << "Parameter 'urdf' missing or invalid";
        return false;
    }

    // ROS TOPICS
    // ==========

    if (!(config.check("humanJointTorquesTopic") && config.find("humanJointTorquesTopic").isString())) {
        yError() << LogPrefix << "Parameter 'humanJointTorquesTopic' missing or invalid";
        return false;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    double period = config.find("period").asFloat64();
    std::string urdfFileName = config.find("urdf").asString();
    std::string humanJointTorquesTopicName = config.find("humanJointTorquesTopic").asString();

    yInfo() << LogPrefix << "*** =====================";
    yInfo() << LogPrefix << "*** Period              :" << period;
    yInfo() << LogPrefix << "*** Urdf file name         :" << urdfFileName;
    yInfo() << LogPrefix << "*** Joint topic name    :" << humanJointTorquesTopicName;
    yInfo() << LogPrefix << "*** =====================";

    // Find the URDF file
    auto& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string urdfFilePath = rf.findFile(urdfFileName);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("urdf").asString();
        return false;
    }

    // Load the model
    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << urdfFilePath;
        return false;
    }

    // Get the model from the loader
    pImpl->humanModel = modelLoader.model();

    // =========================
    // INITIALIZE ROS PUBLISHERS
    // =========================

    // TODO: Initialize ROS resource for human joint torques

    setPeriod(period);
    return true;
}

bool HumanDynamicsPublisher::close()
{
    detach();
    pImpl->humanJointTorquesROS.publisher.close();
    pImpl->node.interrupt();

    return true;
}

void HumanDynamicsPublisher::run()
{
    // TODO: Implement how the joint torques should be published
}

bool HumanDynamicsPublisher::attach(yarp::dev::PolyDriver* poly)
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

    // If it is ready, check that is valid
    if (pImpl->humanDynamics->getNumberOfJoints() != 0
        && (pImpl->humanDynamics->getNumberOfJoints() != pImpl->humanDynamics->getJointNames().size())) {
        yError() << LogPrefix << "The IHumanDynamics interface is not valid."
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

bool HumanDynamicsPublisher::detach()
{
    stop();
    pImpl->humanJointTorquesROS.publisher.interrupt();
    pImpl->humanDynamics = nullptr;
    return true;
}

bool HumanDynamicsPublisher::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool HumanDynamicsPublisher::detachAll()
{
    return detach();
}

// TODO: This function is needed for the ros publishers
// It can be moved to a separate header file
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
