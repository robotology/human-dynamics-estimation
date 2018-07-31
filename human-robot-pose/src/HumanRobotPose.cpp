#include "HumanRobotPose.h"

#include "TickTime.h"
#include "sensor_msgs_JointState.h"
#include "sensor_msgs_Temperature.h"
#include "tf2_msgs_TFMessage.h"
#include "thrifts/HumanState.h"

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Property.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>

#include <algorithm>
#include <limits.h>
#include <memory>
#include <vector>

const std::string LogPrefix = "human-robot-pose : ";
inline TickTime normalizeSecNSec(double yarpTimeStamp);
inline bool parseRotationMatrix(const yarp::os::Value& ini, iDynTree::Rotation& rotation);
inline bool parsePositionVector(const yarp::os::Value& ini, iDynTree::Position& position);
inline bool parseFrameListOption(const yarp::os::Value& option,
                                 std::vector<std::string>& parsedSegments);

enum TF
{
    HumanEntry = 0,
    RobotEntry = 1
};

struct HumanJointStatePublisherResources
{
    size_t counter = 0;
    sensor_msgs_JointState message;
    yarp::os::Publisher<sensor_msgs_JointState> publisher;
};

struct BasePosePublisherResources
{
    size_t counter = 0;
    tf2_msgs_TFMessage message;
    std::shared_ptr<yarp::os::Publisher<tf2_msgs_TFMessage>> publisher = nullptr;
};

class HumanRobotPose::impl
{
public:
    double period = 0.1;
    yarp::os::Mutex mutex; // TODO: check if we should put it somewhere else
    bool autoconnect = false;

    // Human
    human::HumanState* humanStateData = nullptr;
    yarp::os::BufferedPort<human::HumanState> humanStatePort;
    std::string humanStateRemotePort;
    iDynTree::JointPosDoubleArray humanJointsPosition;
    iDynTree::Transform world_H_humanBase;
    iDynTree::JointPosDoubleArray humanJointsVelocity;
    iDynTree::KinDynComputations humanKinDynComp;

    // Robot
    bool enableRobot = false;
    iDynTree::JointPosDoubleArray robotJointsPosition;
    iDynTree::JointPosDoubleArray robotJointsVelocity;
    yarp::dev::PolyDriver remoteControlBoardRemapper;
    yarp::dev::IEncoders* iEncoders = nullptr;
    iDynTree::KinDynComputations robotKinDynComp;

    const double gravityRaw[3] = {0, 0, -9.81};
    const iDynTree::Vector3 gravity = {gravityRaw, 3};

    // Contacts
    typedef std::pair<std::string, std::string> Contact;
    Contact contact1;
    Contact contact2;

    // Method #1, #2, #3 selection
    bool useSkin = false;
    bool useFixedTransform = false;

    // Method #2
    std::string fromHumanFrame;
    std::string toRobotFrame;
    iDynTree::Transform humanFrame_H_robotFrame;

    // ROS Publishers
    std::string tfPrefix;
    std::unique_ptr<yarp::os::Node> node;
    HumanJointStatePublisherResources humanJointStateROS;
    BasePosePublisherResources basePoseROS;

    // Methods for computing the robot pose
    iDynTree::Transform world_H_humanBase_forRobotPose;
    iDynTree::Transform humanBase_H_humanFrame_forRobotPose;
    iDynTree::Transform computeRobotPose_withSkin();
    iDynTree::Transform computeRobotPose_withFixedTransform();
    iDynTree::Transform computeRobotPose_withKnownContacts();

    // RPC
    bool setRobotPose();
    std::string handlerPortName;
    yarp::os::Port handlerPort;
    bool readHumanTransform = true;
};

HumanRobotPose::HumanRobotPose()
    : pImpl{new HumanRobotPose::impl()}
{}

HumanRobotPose::~HumanRobotPose() {}

double HumanRobotPose::getPeriod()
{
    return pImpl->period;
}

bool HumanRobotPose::configure(yarp::os::ResourceFinder& rf)
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
    if (moduleName != "human-robot-pose") {
        yError() << LogPrefix
                 << "The moduleName parameter of the passed configuration is not human-robot-pose";
        return false;
    }

    // MODULE PARAMETERS
    // =================

    if (!(rf.check("period") && rf.find("period").isInt())) {
        yError() << LogPrefix << "Parameter 'period' missing or invalid";
        return false;
    }

    if (!(rf.check("autoconnect") && rf.find("autoconnect").isBool())) {
        yError() << LogPrefix << "Parameter 'autoconnect' missing or invalid";
        return false;
    }

    // HUMAN PARAMETERS
    // ================

    if (!(rf.check("humanModel") && rf.find("humanModel").isString())) {
        yError() << LogPrefix << "Parameter 'humanModel' missing or invalid";
        return false;
    }

    if (!(rf.check("humanJointsListIni") && rf.find("humanJointsListIni").isString())) {
        yError() << LogPrefix << "Parameter 'humanJointsListIni' missing or invalid";
        return false;
    }

    if (!(rf.check("humanStatePort") && rf.find("humanStatePort").isString())) {
        yError() << LogPrefix << "Parameter 'humanStatePort' missing or invalid";
        return false;
    }

    // ROBOT PARAMETERS
    // ================

    if (!(rf.check("enableRobot") && rf.find("enableRobot").isBool())) {
        yError() << LogPrefix << "Parameter 'enableRobot' missing or invalid";
        return false;
    }

    if (!(rf.check("robotName") && rf.find("robotName").isString())) {
        yError() << LogPrefix << "Parameter 'robotName' missing or invalid";
        return false;
    }
    if (!(rf.check("robotModel") && rf.find("robotModel").isString())) {
        yError() << LogPrefix << "Parameter 'robotModel' missing or invalid";
        return false;
    }

    // MODE #1 PARAMETERS (SKIN)
    // =========================

    if (!(rf.check("useSkin") && rf.find("useSkin").isBool())) {
        yError() << LogPrefix << "Parameter 'useSkin' missing or invalid";
        return false;
    }

    if (!(rf.check("skinManagerPort") && rf.find("skinManagerPort").isString())) {
        yError() << LogPrefix << "Parameter 'skinManagerPort' missing or invalid";
        return false;
    }

    // MODE #2 PARAMETERS (NO SKIN, HARDCODED TRANSFORM)
    // =================================================

    if (!(rf.check("useFixedTransform") && rf.find("useFixedTransform").isBool())) {
        yError() << LogPrefix << "Parameter 'useFixedTransform' missing or invalid";
        return false;
    }

    if (!(rf.check("fixedTransformPos") && rf.find("fixedTransformPos").isList())) {
        yError() << LogPrefix << "Parameter 'fixedTransformPos' missing or invalid";
        return false;
    }

    if (!(rf.check("fixedTransformRot") && rf.find("fixedTransformRot").isList())) {
        yError() << LogPrefix << "Parameter 'fixedTransformRot' missing or invalid";
        return false;
    }

    if (!(rf.check("fromHumanFrame") && rf.find("fromHumanFrame").isString())) {
        yError() << LogPrefix << "Parameter 'fromHumanFrame' missing or invalid";
        return false;
    }

    if (!(rf.check("toRobotFrame") && rf.find("toRobotFrame").isString())) {
        yError() << LogPrefix << "Parameter 'toRobotFrame' missing or invalid";
        return false;
    }

    // MODE #3 PARAMETERS (NO SKIN, KNOWN CONTACT FRAMES)
    // ==================================================

    if (!(rf.check("humanContactFrames") && rf.find("humanContactFrames").isList()
          && (rf.find("humanContactFrames").asList()->size() == 2))) {
        yError() << LogPrefix << "Parameter 'humanContactFrames' missing or invalid";
        return false;
    }

    if (!(rf.check("robotContactFrames") && rf.find("robotContactFrames").isList()
          && rf.find("robotContactFrames").asList()->size() == 2)) {
        yError() << LogPrefix << "Parameter 'robotContactFrames' missing or invalid";
        return false;
    }

    // ROS TOPICS
    // ==========
    if (!(rf.check("tfPrefix") && rf.find("tfPrefix").isString())) {
        yError() << LogPrefix << "Parameter 'tfPrefix' missing or invalid";
        return false;
    }

    if (!(rf.check("nodeName") && rf.find("nodeName").isString())) {
        yError() << LogPrefix << "Parameter 'nodeName' missing or invalid";
        return false;
    }

    if (!(rf.check("humanJointsTopic") && rf.find("humanJointsTopic").isString())) {
        yError() << LogPrefix << "Parameter 'humanJointsTopic' missing or invalid";
        return false;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    // MODULE PARAMETERS
    pImpl->period = rf.find("period").asInt() / 1000.0;
    pImpl->autoconnect = rf.find("autoconnect").asBool();

    // HUMAN PARAMETERS
    const std::string humanModelName = rf.find("humanModel").asString();
    const std::string humanJointsListIni = rf.find("humanJointsListIni").asString();
    pImpl->humanStateRemotePort = rf.find("humanStatePort").asString();

    // ROBOT PARAMETERS
    pImpl->enableRobot = rf.find("enableRobot").asBool();
    const std::string robotName = rf.find("robotName").asString();
    const std::string robotModelName = rf.find("robotModel").asString();

    // MODE #1 PARAMETERS
    pImpl->useSkin = rf.find("useSkin").asBool();
    if (pImpl->useSkin) {
        const std::string skinManagerPort = rf.find("skinManagerPort").asString();
    }

    // MODE #2 PARAMETERS
    pImpl->useFixedTransform = rf.find("useFixedTransform").asBool();
    if (pImpl->useFixedTransform) {
        iDynTree::Position pos;
        iDynTree::Rotation rot;
        if (!parsePositionVector(rf.find("fixedTransformPos"), pos)) {
            yError() << "Failed to parse fixedTransformPos parameter";
            return false;
        }
        if (!parseRotationMatrix(rf.find("fixedTransformRot"), rot)) {
            yError() << "Failed to parse fixedTransformRot parameter";
            return false;
        }
        pImpl->humanFrame_H_robotFrame.setPosition(pos);
        pImpl->humanFrame_H_robotFrame.setRotation(rot);

        pImpl->fromHumanFrame = rf.find("fromHumanFrame").asString();
        pImpl->toRobotFrame = rf.find("toRobotFrame").asString();
    }

    // MODE #3 PARAMETERS
    const yarp::os::Bottle* humanContactFrames = rf.find("humanContactFrames").asList();
    const yarp::os::Bottle* robotContactFrames = rf.find("robotContactFrames").asList();

    pImpl->contact1 = {humanContactFrames->get(0).asString(),
                       robotContactFrames->get(0).asString()};
    pImpl->contact2 = {humanContactFrames->get(1).asString(),
                       robotContactFrames->get(1).asString()};

    // ROS TOPICS
    pImpl->tfPrefix = rf.find("tfPrefix").asString();
    const std::string nodeName = rf.find("nodeName").asString();
    const std::string humanJointsTopic = rf.find("humanJointsTopic").asString();

    // =====================
    // INITIALIZE YARP PORTS
    // =====================

    // Open ports
    const std::string humanStateInputPortName = "/" + getName() + "/humanState:i";
    if (!pImpl->humanStatePort.open(humanStateInputPortName)) {
        yError() << LogPrefix << "Failed to open port " << humanStateInputPortName;
        return false;
    }

    if (pImpl->autoconnect) {
        if (!yarp::os::Network::connect(pImpl->humanStateRemotePort, humanStateInputPortName)) {
            yError() << LogPrefix << "Failed to connect " << pImpl->humanStateRemotePort << " to "
                     << humanStateInputPortName;
            return false;
        }
    }

    // ==============================
    // INITIALIZE IENCODERS INTERFACE
    // ==============================

    const std::vector<std::string> robotControlledJoints = {
        // torso
        "torso_pitch",
        "torso_roll",
        "torso_yaw",
        // left arm
        "l_shoulder_pitch",
        "l_shoulder_roll",
        "l_shoulder_yaw",
        "l_elbow",
        "l_wrist_prosup",
        "l_wrist_pitch",
        "l_wrist_yaw",
        // right arm
        "r_shoulder_pitch",
        "r_shoulder_roll",
        "r_shoulder_yaw",
        "r_elbow",
        "r_wrist_prosup",
        "r_wrist_pitch",
        "r_wrist_yaw",
        // left leg
        "l_hip_pitch",
        "l_hip_roll",
        "l_hip_yaw",
        "l_knee",
        "l_ankle_pitch",
        "l_ankle_roll",
        // right leg
        "r_hip_pitch",
        "r_hip_roll",
        "r_hip_yaw",
        "r_knee",
        "r_ankle_pitch",
        "r_ankle_roll",
    };

    if (pImpl->enableRobot) {
        // Convert to Bottle
        yarp::os::Bottle axesNames;
        yarp::os::Bottle& axesList = axesNames.addList();
        for (const auto& axes : robotControlledJoints) {
            axesList.addString(axes);
        }

        yarp::os::Bottle remoteControlBoards;
        yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
        const std::string cbPrefix = "/" + robotName + "/";
        remoteControlBoardsList.addString(cbPrefix + "torso");
        remoteControlBoardsList.addString(cbPrefix + "left_arm");
        remoteControlBoardsList.addString(cbPrefix + "right_arm");
        remoteControlBoardsList.addString(cbPrefix + "left_leg");
        remoteControlBoardsList.addString(cbPrefix + "right_leg");

        // Open the remotecontrolboardremapper device
        yarp::os::Property options;
        options.put("device", "remotecontrolboardremapper");
        options.put("axesNames", axesNames.get(0));
        options.put("remoteControlBoards", remoteControlBoards.get(0));
        options.put("localPortPrefix", "/" + getName());
        yarp::os::Property& remoteCBOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
        remoteCBOpts.put("writeStrict", "on");

        if (!pImpl->remoteControlBoardRemapper.open(options)
            && !pImpl->remoteControlBoardRemapper.isValid()) {
            yError() << LogPrefix
                     << "Failed to open the RemoteControlBoardRemapper with the options passed";
            return false;
        }

        // Access the interface
        if (!pImpl->remoteControlBoardRemapper.view(pImpl->iEncoders)) {
            yError() << LogPrefix
                     << "Failed to get the iEncoders interface from the RemoteControlBoardRemapper";
            return false;
        }
    }

    // ======================
    // INITIALIZE HUMAN MODEL
    // ======================

    // Load the ini file containing the human joints
    std::string humanJointsListIniPath = rf.findFile(humanJointsListIni.c_str());
    if (humanJointsListIniPath.empty()) {
        yError() << LogPrefix << "ResourceFinder couldn't find ini file " + humanJointsListIni;
        return false;
    }

    yarp::os::Property config;
    if (!config.fromConfigFile(humanJointsListIniPath, /*wipe=*/true)) {
        yError() << LogPrefix << "Failed to read " << humanJointsListIniPath << " file";
        return false;
    }

    // Parse the ini file containing the human joints
    std::vector<std::string> humanJointListFromConf;
    if (!parseFrameListOption(config.find("jointList"), humanJointListFromConf)) {
        yError() << LogPrefix << "Failed parsing the joint list read from "
                 << humanJointsListIniPath;
        return false;
    }

    // Find the urdf file
    std::string humanModelPath = rf.findFile(humanModelName.c_str());
    if (humanModelPath.empty()) {
        yError() << LogPrefix << "ResourceFinder couldn't find urdf file " + humanModelName;
        return false;
    }

    // Load the urdf model
    iDynTree::ModelLoader humanMdlLoader;
    if (!humanMdlLoader.loadReducedModelFromFile(humanModelPath, humanJointListFromConf)) {
        yError() << LogPrefix << "Failed to load reduced human model from file";
        return false;
    }

    const iDynTree::Model& humanModel = humanMdlLoader.model();
    pImpl->humanKinDynComp.loadRobotModel(humanModel);
    pImpl->humanKinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    // Read the joints from the URDF
    std::vector<std::string> humanJointListURDF;
    humanJointListURDF.reserve(humanModel.getNrOfJoints());
    for (iDynTree::JointIndex jointIdx = 0; jointIdx < humanModel.getNrOfJoints(); ++jointIdx) {

        // Get the joint name from the index
        std::string jointNameFromUrdf = humanModel.getJointName(jointIdx);

        // If it is not present in the configuration, raise an error.
        // This logic supports reduced models.
        if (std::find(
                humanJointListFromConf.begin(), humanJointListFromConf.end(), jointNameFromUrdf)
            == humanJointListFromConf.end()) {
            yError() << LogPrefix << "URDF joints and received joints do not match";
            close();
            return false;
        }

        // Store it in the vector
        humanJointListURDF.push_back(jointNameFromUrdf);

        // Check if the joint order of the configuration file matches the order of the urdf.
        // TODO: why do we need this? Serialization?
        if ((humanJointListURDF[jointIdx].compare(humanJointListFromConf[jointIdx]))) {
            yError() << LogPrefix
                     << "URDF joints is different from the order of the received joints";
            close();
            return false;
        }
    }

    // Set the joints names already here
    pImpl->humanJointStateROS.message.name.resize(humanJointListURDF.size());
    pImpl->humanJointStateROS.message.header.frame_id = "ground";
    for (unsigned jointIdx = 0; jointIdx < humanJointListURDF.size(); ++jointIdx) {
        pImpl->humanJointStateROS.message.name[jointIdx] = humanJointListURDF[jointIdx];
    }

    // ======================
    // INITIALIZE ROBOT MODEL
    // ======================

    if (pImpl->enableRobot) {
        std::string robotModelPath = rf.findFile(robotModelName.c_str());
        if (robotModelName.empty()) {
            yError() << LogPrefix << "ResourceFinder couldn't find urdf file " + robotModelName;
            return false;
        }

        iDynTree::ModelLoader robotMdlLoader;
        if (!robotMdlLoader.loadReducedModelFromFile(robotModelPath, robotControlledJoints)) {
            yError() << LogPrefix << "Failed to load reduced robot model from file";
            return false;
        }

        pImpl->robotKinDynComp.loadRobotModel(robotMdlLoader.model());
        pImpl->robotKinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
    }

    // =========================
    // INITIALIZE ROS PUBLISHERS
    // =========================

    pImpl->node = std::unique_ptr<yarp::os::Node>(new yarp::os::Node(nodeName));

    // Initialize ROS resource for human base pose
    pImpl->basePoseROS.publisher = std::make_shared<yarp::os::Publisher<tf2_msgs_TFMessage>>("/tf");
    pImpl->basePoseROS.message.transforms.resize(1);

    // Initialize ROS resource for robot base pose
    if (pImpl->enableRobot) {
        pImpl->basePoseROS.message.transforms.resize(2);
    }

    // Initialize ROS resource for human joints position
    const unsigned humanDofs = pImpl->humanKinDynComp.getNrOfDegreesOfFreedom();
    pImpl->humanJointStateROS.publisher.topic(humanJointsTopic);
    pImpl->humanJointStateROS.message.position.resize(humanDofs, 0);
    pImpl->humanJointStateROS.message.velocity.resize(humanDofs, 0);
    pImpl->humanJointStateROS.message.effort.resize(humanDofs, 0);

    // ===================
    // INITIALIZE RPC PORT
    // ===================

    pImpl->handlerPortName = "/" + getName() + "/rcp:i";
    if (!pImpl->handlerPort.open(pImpl->handlerPortName.c_str())) {
        yError() << LogPrefix << "Unable to open port " << pImpl->handlerPortName;
        return false;
    }

    attach(pImpl->handlerPort);

    // ==========================
    // INITIALIZE OTHER RESOURCES
    // ==========================

    // Buffer for human position
    pImpl->humanJointsPosition.resize(humanDofs);
    pImpl->humanJointsPosition.zero();

    // Buffer for human velocity
    pImpl->humanJointsVelocity.resize(humanDofs);
    pImpl->humanJointsVelocity.zero();

    if (pImpl->enableRobot) {
        const unsigned robotDofs = pImpl->robotKinDynComp.getNrOfDegreesOfFreedom();
        // Buffer for robot position
        pImpl->robotJointsPosition.resize(robotDofs);
        pImpl->robotJointsPosition.zero();
        // Buffer for robot velocity
        pImpl->robotJointsVelocity.resize(robotDofs);
        pImpl->robotJointsVelocity.zero();
    }

    // Initialize cached resources for setting the robot pose only when the
    // rpc command setRobotPose is executed
    pImpl->world_H_humanBase_forRobotPose = iDynTree::Transform::Identity();
    pImpl->humanBase_H_humanFrame_forRobotPose = iDynTree::Transform::Identity();

    return true;
}

bool HumanRobotPose::updateModule()
{
    if (!pImpl->autoconnect) {
        if (!yarp::os::Network::isConnected(pImpl->humanStateRemotePort,
                                            pImpl->humanStatePort.getName())) {
            yInfo() << LogPrefix << "Ports " << pImpl->humanStateRemotePort << " and "
                    << pImpl->humanStatePort.getName()
                    << " are not connected. Waiting manual connection.";
            return true;
        }
        if (pImpl->enableRobot) {
        }
    }

    // ===================================================
    // READ AND UPDATE HUMAN JOINTS CONFIGURATION AND POSE
    // ===================================================

    // The State provider is slower than this module. Use old data
    // if the reading is not available.
    // TODO: switch to a callback on humanStatePort
    auto newHumanStateData = pImpl->humanStatePort.read(/*shouldWait=*/false);
    if (newHumanStateData) {
        pImpl->humanStateData = newHumanStateData;
    }

    if (!pImpl->humanStateData) {
        yError() << LogPrefix << "Failed to read the human state data";
        return false;
    }

    if (!iDynTree::toiDynTree(pImpl->humanStateData->positions, pImpl->humanJointsPosition)) {
        yError() << LogPrefix << "Failed to parse the human state data";
        return false;
    }

    // Update KinDynKinematics
    pImpl->humanKinDynComp.setRobotState(
        pImpl->humanJointsPosition, pImpl->humanJointsVelocity, pImpl->gravity);

    // Store the world_H_humanBase transform. It is required later.
    // Rotation
    iDynTree::Vector4 pelvisQuaternion;
    pelvisQuaternion(0) = pImpl->humanStateData->baseOrientationWRTGlobal.w;
    pelvisQuaternion(1) = pImpl->humanStateData->baseOrientationWRTGlobal.imaginary.x;
    pelvisQuaternion(2) = pImpl->humanStateData->baseOrientationWRTGlobal.imaginary.y;
    pelvisQuaternion(3) = pImpl->humanStateData->baseOrientationWRTGlobal.imaginary.z;
    pImpl->world_H_humanBase.setRotation(
        iDynTree::Rotation::RotationFromQuaternion(pelvisQuaternion));
    // Translation
    pImpl->world_H_humanBase.setPosition({pImpl->humanStateData->baseOriginWRTGlobal.x,
                                          pImpl->humanStateData->baseOriginWRTGlobal.y,
                                          pImpl->humanStateData->baseOriginWRTGlobal.z});

    // ==========================================
    // READ AND UPDATE ROBOT JOINTS CONFIGURATION
    // ==========================================

    if (pImpl->enableRobot) {
        if (!pImpl->iEncoders) {
            yError() << LogPrefix << "iEncoders interface was not configured properly";
            return false;
        }

        if (!pImpl->iEncoders->getEncoders(pImpl->robotJointsPosition.data())) {
            yError() << LogPrefix << "Failed to read robot joints positions from the interface";
            return false;
        }

        // Convert deg to rad
        iDynTree::toEigen(pImpl->robotJointsPosition) =
            M_PI / 180.0 * iDynTree::toEigen(pImpl->robotJointsPosition);

        pImpl->robotKinDynComp.setRobotState(
            pImpl->robotJointsPosition, pImpl->robotJointsVelocity, pImpl->gravity);
    }

    // =============================
    // COMPUTE FRAME TRANSFORMATIONS
    // =============================

    if (pImpl->readHumanTransform) {

        pImpl->readHumanTransform = false;
        pImpl->humanBase_H_humanFrame_forRobotPose =
            pImpl->humanKinDynComp.getRelativeTransform("Pelvis", pImpl->fromHumanFrame);
        pImpl->world_H_humanBase_forRobotPose = pImpl->world_H_humanBase;
    }

    iDynTree::Transform robotPose;
    if (pImpl->enableRobot) {

        // Contacts detected with the skin
        if (pImpl->useSkin) {
            robotPose = pImpl->computeRobotPose_withSkin();
        }
        else {
            // Relative transform read from the configuration
            if (pImpl->useFixedTransform) {
                robotPose = pImpl->computeRobotPose_withFixedTransform();
            }
            // Estimate transformation but read contacts from the configuration
            else {
                robotPose = pImpl->computeRobotPose_withKnownContacts();
            }
        }
    }

    // ===================
    // WRITE TO ROS TOPICS
    // ===================

    // HUMAN JOINTS POSITION
    // =====================

    // Get the message buffer to send
    sensor_msgs_JointState& jointStateMsg = pImpl->humanJointStateROS.publisher.prepare();

    pImpl->humanJointStateROS.message.header.seq = pImpl->humanJointStateROS.counter++;
    pImpl->humanJointStateROS.message.header.stamp = normalizeSecNSec(yarp::os::Time::now());
    for (size_t index = 0; index < jointStateMsg.position.size(); ++index) {
        pImpl->humanJointStateROS.message.position[index] = pImpl->humanStateData->positions[index];
    }

    jointStateMsg = pImpl->humanJointStateROS.message;
    pImpl->humanJointStateROS.publisher.write();

    // HUMAN TRANSFORM world_HH_base
    // =============================

    // Get the message to send
    tf2_msgs_TFMessage& tfMsgHuman = pImpl->basePoseROS.publisher->prepare();

    // Get the buffer and the original data
    auto& humanTransform = pImpl->basePoseROS.message.transforms[HumanEntry];
    const auto& humanStateData = pImpl->humanStateData;

    // Metadata
    humanTransform.header.seq = pImpl->basePoseROS.counter++;
    humanTransform.header.stamp = normalizeSecNSec(yarp::os::Time::now());
    humanTransform.header.frame_id = "ground";
    humanTransform.child_frame_id = pImpl->tfPrefix + "/Pelvis";

    // Translation
    humanTransform.transform.translation.x = humanStateData->baseOriginWRTGlobal.x;
    humanTransform.transform.translation.y = humanStateData->baseOriginWRTGlobal.y;
    humanTransform.transform.translation.z = humanStateData->baseOriginWRTGlobal.z;

    // Rotation
    humanTransform.transform.rotation.x = humanStateData->baseOrientationWRTGlobal.imaginary.x;
    humanTransform.transform.rotation.y = humanStateData->baseOrientationWRTGlobal.imaginary.y;
    humanTransform.transform.rotation.z = humanStateData->baseOrientationWRTGlobal.imaginary.z;
    humanTransform.transform.rotation.w = humanStateData->baseOrientationWRTGlobal.w;

    // Store the transform
    tfMsgHuman = pImpl->basePoseROS.message;

    // ROBOT TRANSFORM world_HR_base
    // =============================

    if (pImpl->enableRobot) {

        // Get the message to send
        tf2_msgs_TFMessage& tfMsgRobot = pImpl->basePoseROS.publisher->prepare();

        // Get the buffer
        auto& robotTransform = pImpl->basePoseROS.message.transforms[RobotEntry];

        iDynTree::Position world_P_robotBase = robotPose.getPosition();
        iDynTree::Vector4 world_R_robotBase = robotPose.getRotation().asQuaternion();

        // Metadata
        robotTransform.header.seq = pImpl->basePoseROS.counter++;
        robotTransform.header.stamp = normalizeSecNSec(yarp::os::Time::now());
        robotTransform.header.frame_id = "ground";
        robotTransform.child_frame_id = "icub02/base_link"; // TODO: transform to a parameter

        // Translation
        robotTransform.transform.translation.x = world_P_robotBase(0);
        robotTransform.transform.translation.y = world_P_robotBase(1);
        robotTransform.transform.translation.z = world_P_robotBase(2);

        // Rotation
        robotTransform.transform.rotation.x = world_R_robotBase(1);
        robotTransform.transform.rotation.y = world_R_robotBase(2);
        robotTransform.transform.rotation.z = world_R_robotBase(3);
        robotTransform.transform.rotation.w = world_R_robotBase(0);

        // Store the transform
        tfMsgRobot = pImpl->basePoseROS.message;
    }

    // Send the transform(s)
    pImpl->basePoseROS.publisher->write();

    return true;
}

bool HumanRobotPose::interruptModule()
{
    // Interrupt yarp ports
    pImpl->humanStatePort.interrupt();
    pImpl->handlerPort.interrupt();

    // Interrupt ROS publishers
    pImpl->basePoseROS.publisher->interrupt();
    pImpl->humanJointStateROS.publisher.interrupt();

    return true;
}

iDynTree::Transform HumanRobotPose::impl::computeRobotPose_withSkin()
{
    // TODO
    return {};
}

iDynTree::Transform HumanRobotPose::impl::computeRobotPose_withFixedTransform()
{
    iDynTree::Transform robotFrame_H_robotBase =
        robotKinDynComp.getRelativeTransform(toRobotFrame, "base_link");

    return world_H_humanBase_forRobotPose * humanBase_H_humanFrame_forRobotPose
           * humanFrame_H_robotFrame * robotFrame_H_robotBase;
}

bool HumanRobotPose::impl::setRobotPose()
{
    readHumanTransform = true;
    return true;
}

iDynTree::Transform HumanRobotPose::impl::computeRobotPose_withKnownContacts()
{
    // TODO
    return {};
}

bool HumanRobotPose::close()
{
    // Close yarp ports
    pImpl->humanStatePort.close();
    pImpl->handlerPort.close();

    // Close ROS publishers
    pImpl->basePoseROS.publisher->close();
    pImpl->humanJointStateROS.publisher.close();

    // Stop the node
    pImpl->node->interrupt();

    return true;
}

bool HumanRobotPose::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    if (command.size() == 1 && command.get(0).isString()) {
        std::string cmd = command.get(0).asString();

        if (cmd == "help") {
            reply.addString("commands: setRobotPose\n");
            return true;
        }
        else if (cmd == "setRobotPose") {
            if (!pImpl->setRobotPose()) {
                reply.addString("Fail");
                return true;
            }
            reply.addString("Ok");
        }
        else {
            reply.addString("Fail");
            return true;
        }
    }
    else {
        reply.addString("Fail");
        return true;
    }
    return true;
}

inline TickTime normalizeSecNSec(double yarpTimeStamp)
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

inline bool parseRotationMatrix(const yarp::os::Value& ini, iDynTree::Rotation& rotation)
{
    if (ini.isNull() || !ini.isList()) {
        return false;
    }
    yarp::os::Bottle* outerList = ini.asList();
    if (!outerList || outerList->size() != 3) {
        return false;
    }
    for (int row = 0; row < outerList->size(); ++row) {
        yarp::os::Value& innerValue = outerList->get(row);
        if (innerValue.isNull() || !innerValue.isList()) {
            return false;
        }
        yarp::os::Bottle* innerList = innerValue.asList();
        if (!innerList || innerList->size() != 3) {
            return false;
        }
        for (int column = 0; column < innerList->size(); ++column) {
            rotation.setVal(row, column, innerList->get(column).asDouble());
        }
    }
    return true;
}

inline bool parsePositionVector(const yarp::os::Value& ini, iDynTree::Position& position)
{
    if (ini.isNull() || !ini.isList()) {
        return false;
    }
    yarp::os::Bottle* list = ini.asList();
    if (!list || list->size() != 3) {
        return false;
    }
    for (int i = 0; i < list->size(); ++i) {
        position.setVal(i, list->get(i).asDouble());
    }
    return true;
}

inline bool parseFrameListOption(const yarp::os::Value& option,
                                 std::vector<std::string>& parsedSegments)
{
    if (option.isNull() || !option.isList() || !option.asList())
        return false;
    yarp::os::Bottle* frames = option.asList();
    parsedSegments.reserve(static_cast<size_t>(frames->size()));

    for (int i = 0; i < frames->size(); ++i) {
        if (frames->get(i).isString()) {
            parsedSegments.push_back(frames->get(i).asString());
        }
    }
    return true;
}
