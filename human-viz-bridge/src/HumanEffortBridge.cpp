#include "TickTime.h"
#include "sensor_msgs_Temperature.h"
#include "std_msgs_Header.h"
#include "thrifts/HumanDynamics.h"

#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <cmath>
#include <limits.h>
#include <memory>
#include <string>
#include <vector>

const std::string LogPrefix = "human-effort-bridge : ";

typedef iDynTree::JointIndex JointIndex;
typedef std::string LinkName;

struct JointEffortData
{
    LinkName parentLinkName;
    std::string sphericalJointName;
    std::vector<JointIndex> fakeJointsIndices;
    sensor_msgs_Temperature message;
    std::shared_ptr<yarp::os::Publisher<sensor_msgs_Temperature>> publisher;
};

struct ModelEffortData
{
    std::vector<JointEffortData> efforts;
};

typedef std::string SphericalJointName;
typedef std::string SphericalJointChildLink;
typedef std::pair<SphericalJointName, SphericalJointChildLink> SphericalJointData;
bool parseSphericalJointsData(const yarp::os::Value& option,
                              std::vector<SphericalJointData>& sphericalJointsData);
TickTime normalizeSecNSec(double yarpTimeStamp);
bool parseFrameListOption(const yarp::os::Value& option, std::vector<std::string>& parsedSegments);

class HumanEffortBridge
    : public yarp::os::RFModule
    , public yarp::os::TypedReaderCallback<human::HumanDynamics>
{
private:
    std::vector<JointEffortData> m_modelEffortData;
    yarp::os::BufferedPort<human::HumanDynamics> m_humanDynamicsDataPort;
    std::unique_ptr<yarp::os::Node> m_node;

public:
    double getPeriod() override { return 100; }

    // The HumanEffortBridge runs using a BufferedPort callback function.
    // See onRead().
    bool updateModule() override { return true; }

    // Configure function.
    bool configure(yarp::os::ResourceFinder& rf) override
    {
        // ================
        // CHECK PARAMETERS
        // ================

        if (!rf.check("name")) {
            yError() << LogPrefix << "Module name is wrong or missing";
        }
        const std::string moduleName = rf.find("name").asString();
        setName(moduleName.c_str());
        // Check that the name matches the module name in order to avoid
        // passing a wrong configuration file
        if (moduleName != "human-effort-bridge") {
            yError() << LogPrefix << "The moduleName parameter of the passed configuration"
                     << " is not human-effort-bridge";
            return false;
        }

        if (!(rf.check("serverName") && rf.find("serverName").isString())) {
            yError() << LogPrefix << "Parameter 'serverName' missing or invalid";
            return false;
        }

        // URDF MODEL PARAMETERS
        // =====================

        if (!(rf.check("urdf_model") && rf.find("urdf_model").isString())) {
            yError() << LogPrefix << "Parameter 'urdf_model' missing or invalid";
            return false;
        }

        if (!(rf.check("sphericalJointsEfforts") && rf.find("sphericalJointsEfforts").isList())) {
            yError() << LogPrefix << "Parameter 'sphericalJointsEfforts' missing or invalid";
            return false;
        }

        // ROS / YARP PARAMETERS
        // =====================

        if (!(rf.check("autoconnect") && rf.find("autoconnect").isBool())) {
            yError() << LogPrefix << "Parameter 'autoconnect' missing or invalid";
            return false;
        }

        if (!(rf.check("nodeName") && rf.find("nodeName").isString())) {
            yError() << LogPrefix << "Parameter 'nodeName' missing or invalid";
            return false;
        }

        if (!(rf.check("topicPrefix") && rf.find("topicPrefix").isString())) {
            yError() << LogPrefix << "Parameter 'topicPrefix' missing or invalid";
            return false;
        }

        if (!(rf.check("tfPrefix") && rf.find("tfPrefix").isString())) {
            yError() << LogPrefix << "Parameter 'tfPrefix' missing or invalid";
            return false;
        }

        // ===============
        // READ PARAMETERS
        // ===============

        const std::string humanStateRemotePortName = rf.find("serverName").asString();
        const std::string urdfModelName = rf.find("urdf_model").asString();
        const bool autoconnect = rf.find("autoconnect").asBool();
        const std::string nodeName = rf.find("nodeName").asString();
        const std::string topicPrefix = rf.find("topicPrefix").asString();
        const std::string tfPrefix = rf.find("tfPrefix").asString();

        std::vector<SphericalJointData> sphericalJointData;
        if (!parseSphericalJointsData(rf.find("sphericalJointsEfforts"), sphericalJointData)) {
            yError() << LogPrefix << "Failed to parse effortKinematicData parameter";
            return false;
        }

        // ======================
        // INITIALIZE HUMAN MODEL
        // ======================

        // Find the urdf file
        std::string humanModelAbsPath = rf.findFile(urdfModelName.c_str());
        if (humanModelAbsPath.empty()) {
            yError() << LogPrefix << "ResourceFinder couldn't find urdf file " + urdfModelName;
            return false;
        }

        // Load the urdf model
        iDynTree::ModelLoader modelLoader;
        if (!modelLoader.loadModelFromFile(humanModelAbsPath)) {
            yError() << LogPrefix << "Failed to load URDF model from file " << urdfModelName;
            return false;
        }
        const iDynTree::Model& humanModel = modelLoader.model();

        // INITIALIZE THE MODELEFFORTDATA
        // ==============================

        // Initialize ROS node
        m_node = std::unique_ptr<yarp::os::Node>(new yarp::os::Node(nodeName));

        m_modelEffortData.reserve(sphericalJointData.size());
        bool ok = true;

        // Initialize JointEffortData for all the published joints with effort
        for (const auto& sphericalJoint : sphericalJointData) {
            JointEffortData jointEffortData;
            // Kinematics
            jointEffortData.parentLinkName = sphericalJoint.first;
            jointEffortData.sphericalJointName = sphericalJoint.second;
            // Ros publisher
            jointEffortData.publisher =
                std::make_shared<yarp::os::Publisher<sensor_msgs_Temperature>>();
            ok = ok
                 && jointEffortData.publisher->topic(topicPrefix + "/"
                                                     + jointEffortData.sphericalJointName);
            // Ros message
            jointEffortData.message.header.frame_id =
                tfPrefix + "/" + jointEffortData.parentLinkName;
            jointEffortData.message.header.seq = 0;
            jointEffortData.message.variance = 0;
            // Populate the ModelEffortData
            m_modelEffortData.push_back(jointEffortData);
        }

        if (!ok) {
            yError() << LogPrefix << "ROS publishers initialization failed";
            return false;
        }

        // Get the indices of the fake model joints that compose the spherical joint specified in
        // the configuration. The logic is that the fake joints name are:
        //      sphericalJointName + postfix
        //
        // e.g: RightUpperArm + _f1

        // Get all joints from the urdf model
        std::vector<std::string> URDFjoints(humanModel.getNrOfJoints());
        for (iDynTree::JointIndex jointIdx = 0; jointIdx < URDFjoints.size(); ++jointIdx) {
            URDFjoints[jointIdx] = humanModel.getJointName(jointIdx);
        }

        // TODO: for some reason the joints are serialized into the human::HumanDynamics class
        //       in alphabetical order, not following iDynTree indices
        sort(URDFjoints.begin(), URDFjoints.end());

        // Check all the occurences of sphericalJointName* in the urdf model joints
        for (unsigned jointIdx = 0; jointIdx < URDFjoints.size(); ++jointIdx) {
            // Name of the processed urdf joint
            const std::string urdfJointName = URDFjoints[jointIdx];
            // Find if one of the sphericalJointNames from the conf is a substring
            for (auto& effortData : m_modelEffortData) {
                if (urdfJointName.find(effortData.sphericalJointName) != std::string::npos) {
                    // Store the index
                    effortData.fakeJointsIndices.push_back(jointIdx);
                }
            }
        }

        // ====================
        // INITIALIZE YARP PORT
        // ====================

        // Open ports
        const std::string humanStateInputPortName = "/" + getName() + "/humanState:i";
        if (!m_humanDynamicsDataPort.open(humanStateInputPortName)) {
            yError() << LogPrefix << "Failed to open port " << humanStateInputPortName;
            return false;
        }

        if (autoconnect) {
            if (!yarp::os::Network::connect(humanStateRemotePortName, humanStateInputPortName)) {
                yError() << LogPrefix << "Failed to connect " << humanStateRemotePortName << " to "
                         << humanStateInputPortName;
                return false;
            }
        }

        // Configure the port callback
        m_humanDynamicsDataPort.useCallback(*this);

        return true;
    }

    bool interruptModule() override
    {
        // Interrupt yarp ports
        m_humanDynamicsDataPort.interrupt();

        // Interrupt ROS publishers
        for (auto& jointEffortData : m_modelEffortData) {
            jointEffortData.publisher->interrupt();
        }

        return true;
    }

    bool close() override
    {
        // Close yarp ports
        m_humanDynamicsDataPort.close();

        // Close ROS publishers
        for (auto& jointEffortData : m_modelEffortData) {
            jointEffortData.publisher->close();
        }

        // Close the node
        m_node->interrupt();

        return true;
    }

    void onRead(human::HumanDynamics& humanDynamicsData) override
    {
        // Get the timestamp
        TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());

        for (auto& jointEffortData : m_modelEffortData) {
            jointEffortData.message.header.stamp = currentTime;
            jointEffortData.message.header.seq++;

            // Read the efforts of the joints (included the fake ones) the form
            // the effortData.jointName revolute joint
            double effortTmp = 0;
            for (const auto& modelFakeJointIdx : jointEffortData.fakeJointsIndices) {
                effortTmp += pow(humanDynamicsData.jointVariables[modelFakeJointIdx].torque[0], 2);
            }
            effortTmp = sqrt(effortTmp);

            jointEffortData.message.temperature = effortTmp;

            // Store the message into the publisher
            sensor_msgs_Temperature& effortMsg = jointEffortData.publisher->prepare();
            effortMsg = jointEffortData.message;

            // Publish the effort for this joint
            jointEffortData.publisher->write(/*forceStrict=*/false);
        }
    }
};

int main(int argc, char* argv[])
{
    // Initialize YARP network
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(5.0)) {
        yError() << LogPrefix << " YARP server not available";
        return EXIT_FAILURE;
    }

    // Configure ResourceFinder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setVerbose(true);
    rf.setDefaultContext("human-dynamic-estimation");
    rf.setDefaultConfigFile("human-effort-bridge.ini");
    rf.configure(argc, argv);
    rf.setVerbose(true);

    // Configure the module
    HumanEffortBridge module;
    module.runModule(rf);

    // Terminate YARP network
    yarp::os::Network::fini();

    return EXIT_SUCCESS;
}

bool parseSphericalJointsData(const yarp::os::Value& option,
                              std::vector<SphericalJointData>& sphericalJointsData)
{
    if (option.isNull() || !option.isList() || !option.asList()) {
        return false;
    }

    yarp::os::Bottle* sphericalJointsDataList = option.asList();

    for (int sphericalJoint = 0; sphericalJoint < sphericalJointsDataList->size();
         ++sphericalJoint) {
        if (!sphericalJointsDataList->get(sphericalJoint).isList()
            || !(sphericalJointsDataList->get(sphericalJoint).asList()->size() == 2)) {
            return false;
        }

        yarp::os::Bottle* sphericalJointDataList =
            sphericalJointsDataList->get(sphericalJoint).asList();

        if (!sphericalJointDataList->get(0).isString()
            || !sphericalJointDataList->get(1).isString()) {
            return false;
        }

        sphericalJointsData.emplace_back(sphericalJointDataList->get(0).asString(),
                                         sphericalJointDataList->get(1).asString());
    }

    return true;
}

TickTime normalizeSecNSec(double yarpTimeStamp)
{
    uint64_t time = (uint64_t)(yarpTimeStamp * 1000000000UL);
    uint64_t nsec_part = (time % 1000000000UL);
    uint64_t sec_part = (time / 1000000000UL);
    TickTime ret;

    if (sec_part > UINT_MAX) {
        yWarning() << LogPrefix
                   << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    ret.sec = sec_part;
    ret.nsec = nsec_part;
    return ret;
}

bool parseFrameListOption(const yarp::os::Value& option, std::vector<std::string>& parsedSegments)
{
    if (option.isNull() || !option.isList() || !option.asList()) {
        return false;
    }

    yarp::os::Bottle* frames = option.asList();
    parsedSegments.reserve(static_cast<size_t>(frames->size()));

    for (int i = 0; i < frames->size(); ++i) {
        if (!frames->get(i).isString()) {
            return false;
        }
        parsedSegments.push_back(frames->get(i).asString());
    }
    return true;
}
