// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanDynamicsPublisher.h"

#include <hde/interfaces/IHumanDynamics.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/rosmsg/TickTime.h>
#include <yarp/rosmsg/sensor_msgs/Temperature.h>

#include <iDynTree/ModelIO/ModelLoader.h>

#include <string>
#include <vector>
#include <cmath>

const std::string DeviceName = "HumanDynamicsPublisher";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

typedef iDynTree::JointIndex JointIndex;
typedef std::string LinkName;

struct JointEffortData
{
    LinkName parentLinkName;
    std::string sphericalJointName;
    std::vector<JointIndex> fakeJointsIndices;

    yarp::rosmsg::sensor_msgs::Temperature message;
    std::shared_ptr<yarp::os::Publisher<yarp::rosmsg::sensor_msgs::Temperature>> publisher;
};

struct ModelEffortData
{
  std::vector<JointEffortData> efforts;
};

typedef std::string SphericalLinkName;
typedef std::string SphericalJointName;
typedef std::pair<SphericalLinkName, SphericalJointName> SphericalJointDataPair;
bool parseFrameListOption(const yarp::os::Value& option, std::vector<std::string>& parsedSegments);

using namespace hde::publishers;

yarp::rosmsg::TickTime getTimeStampFromYarp();

class HumanDynamicsPublisher::impl
{
public:
    bool firstRun = true;

    hde::interfaces::IHumanDynamics* humanDynamics = nullptr;
    std::vector<JointEffortData> modelEffortData;

    std::vector<double> jointTorques;

    // ROS Publisher
    yarp::os::Node node = {"/" + DeviceName};
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

    // MODEL OPTIONS
    // =============

    if (!(config.check("parentLinkNames") && config.find("parentLinkNames").isList())) {
        yError() << LogPrefix << "Parameter 'parentLinkNames' list missing or invalid";
        return false;
    }

    if (!(config.check("sphericalJointNames") && config.find("sphericalJointNames").isList())) {
        yError() << LogPrefix << "Parameter 'sphericalJointNames' list missing or invalid";
        return false;
    }

    // ROS OPTIONS
    // =============

    if (!(config.check("topicPrefix") && config.find("topicPrefix").isString())) {
        yError() << LogPrefix << "Parameter 'topicPrefix' missing or invalid";
        return false;
    }

    if (!(config.check("tfPrefix") && config.find("tfPrefix").isString())) {
        yError() << LogPrefix << "Parameter 'tfPrefix' missing or invalid";
        return false;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    double period = config.find("period").asFloat64();

    yarp::os::Bottle* listOfLinkNames = config.find("parentLinkNames").asList();
    yarp::os::Bottle* listOfJointNames = config.find("sphericalJointNames").asList();

    if ( listOfLinkNames->size() != listOfJointNames->size()) {
        yError() << LogPrefix << "Number of `parentLinkNames` and 'sphericalJointNames' do not match";
        return false;
    }

    std::string topicPrefix = config.find("topicPrefix").asString();
    std::string tfPrefix = config.find("tfPrefix").asString();

    yInfo() << LogPrefix << "*** =============================";
    yInfo() << LogPrefix << "*** Period                      :" << period;
    yInfo() << LogPrefix << "*** parentLinkNames             :" << listOfLinkNames->toString();
    yInfo() << LogPrefix << "*** sphericalJointNames         :" << listOfJointNames->toString();
    yInfo() << LogPrefix << "*** topicPrefix                 :" << topicPrefix;
    yInfo() << LogPrefix << "*** tfPrefix                    :" << tfPrefix;
    yInfo() << LogPrefix << "*** =============================";


    std::vector<SphericalJointDataPair> SphericalJointData;
    for (size_t joint = 0; joint < listOfJointNames->size(); joint++) {
        SphericalJointData.emplace_back(listOfLinkNames->get(joint).asString(),
                                        listOfJointNames->get(joint).asString());
    }

    // Initialize JointEffortData
    for (const auto& sphericalJoint : SphericalJointData) {
        JointEffortData jointEffortData;

        jointEffortData.parentLinkName = sphericalJoint.first;
        jointEffortData.sphericalJointName = sphericalJoint.second;

        // ROS message
        jointEffortData.message.header.frame_id =
                tfPrefix + "/" + jointEffortData.parentLinkName;
        jointEffortData.message.header.seq = 0;
        jointEffortData.message.variance = 0;

        // ROS publisher
        jointEffortData.publisher =
                std::make_shared<yarp::os::Publisher<yarp::rosmsg::sensor_msgs::Temperature>>();

        if (!jointEffortData.publisher->topic(topicPrefix + "/"
                                              + jointEffortData.sphericalJointName)) {
            yError() << LogPrefix << "ROS publishers initialization failed for " << jointEffortData.sphericalJointName;
            return false;
        }

        // Populate ModelEffortData
        pImpl->modelEffortData.push_back(jointEffortData);
    }

    setPeriod(period);
    return true;
}

bool HumanDynamicsPublisher::close()
{
    // Close ROS publishers
    for (auto& jointEffortData : pImpl->modelEffortData) {
        jointEffortData.publisher->close();
    }

    pImpl->node.interrupt();

    return true;
}

void HumanDynamicsPublisher::run()
{
    if (pImpl->firstRun) {

        // Get the fake joint indices
        std::vector<std::string> URDFjoints = pImpl->humanDynamics->getJointNames();

        // Check all the occurences of sphericalJointName* in the urdf model joints
        for (unsigned jointIdx = 0; jointIdx < URDFjoints.size(); ++jointIdx) {
            // Name of the processed urdf joint
            const std::string urdfJointName = URDFjoints[jointIdx];
            // Find if one of the sphericalJointNames from the conf is a substring
            for (auto& effortData : pImpl->modelEffortData) {
                if (urdfJointName.find(effortData.sphericalJointName) != std::string::npos) {
                    // Store the index
                    effortData.fakeJointsIndices.push_back(jointIdx);
                }
            }
        }

        pImpl->firstRun = false;
    }

    // Get the jointTorques
    pImpl->jointTorques = pImpl->humanDynamics->getJointTorques();

    for (auto& jointEffortData : pImpl->modelEffortData) {
        // Update metadata
        jointEffortData.message.header.seq++;
        jointEffortData.message.header.stamp = getTimeStampFromYarp();

        double effortTmp = 0;

        for (const auto& modelFakeJointIdx : jointEffortData.fakeJointsIndices) {
            effortTmp += pow(pImpl->jointTorques.at(modelFakeJointIdx), 2);
        }
        effortTmp = sqrt(effortTmp);

        jointEffortData.message.temperature = effortTmp;

        // Store the message into the publisher
        auto& effortMsg = jointEffortData.publisher->prepare();
        effortMsg = jointEffortData.message;

        // Publish the effort for this joint
        jointEffortData.publisher->write();
    }
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
    while (pImpl->humanDynamics->getNumberOfJoints() != 0
        && (pImpl->humanDynamics->getNumberOfJoints() != pImpl->humanDynamics->getJointNames().size())) {
        yInfo() << LogPrefix << "IHumanDynamics interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
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

void HumanDynamicsPublisher::threadRelease()
{}

bool HumanDynamicsPublisher::detach()
{
    while (isRunning()) {
        stop();
    }

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
