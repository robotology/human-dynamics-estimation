#include <cmath>

#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_TransformStamped.h"
#include "geometry_msgs_Vector3.h"
#include "msgs/String.h"
#include "sensor_msgs_JointState.h"
#include "std_msgs_Header.h"
#include "tf2_msgs_TFMessage.h"
#include "thrifts/HumanState.h"

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>

#include <TickTime.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Property.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>

#include <algorithm>
#include <iostream>
#include <limits.h>
#include <string>
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace human;

inline TickTime normalizeSecNSec(double yarpTimeStamp)
{
    uint64_t time = (uint64_t) (yarpTimeStamp * 1000000000UL);
    uint64_t nsec_part = (time % 1000000000UL);
    uint64_t sec_part = (time / 1000000000UL);
    TickTime ret;

    if (sec_part > UINT_MAX)
    {
        yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    ret.sec  = sec_part;
    ret.nsec = nsec_part;
    return ret;
}

static bool parseFrameListOption(const Value &option, vector<string> &parsedSegments)
{
    if (option.isNull() || !option.isList() || !option.asList()) return false;
    Bottle *frames = option.asList();
    parsedSegments.reserve(static_cast<size_t>(frames->size()));

    for (int i = 0; i < frames->size(); ++i) {
        if (frames->get(i).isString()) {
            parsedSegments.push_back(frames->get(i).asString());
        }
    }
    return true;
}

static bool parseRotationMatrix(const yarp::os::Value& ini, iDynTree::Rotation& rotation)
{
    if (ini.isNull() || !ini.isList())
    {
        return false;
    }
    yarp::os::Bottle *outerList = ini.asList();
    if (!outerList || outerList->size() != 3)
    {
        return false;
    }
    for (int row = 0; row < outerList->size(); ++row)
    {
        yarp::os::Value& innerValue = outerList->get(row);
        if (innerValue.isNull() || !innerValue.isList())
        {
            return false;
        }
        yarp::os::Bottle *innerList = innerValue.asList();
        if (!innerList || innerList->size() != 3)
        {
            return false;
        }
        for (int column = 0; column < innerList->size(); ++column)
        {
            rotation.setVal(row, column, innerList->get(column).asDouble());
        }
    }
    return true;
}


static bool parsePositionVector(const yarp::os::Value& ini, iDynTree::Position& position)
{
    if (ini.isNull() || !ini.isList())
    {
        return false;
    }
    yarp::os::Bottle *list = ini.asList();
    if (!list || list->size() != 3)
    {
        return false;
    }
    for (int i = 0; i < list->size(); ++i)
    {
        position.setVal(i, list->get(i).asDouble());
    }
    return true;
}


class RobotBasePosePublisher : public RFModule
{
    double period;
    Mutex mutex;
    unsigned rosSequenceCounter;

    Port rpcPort;
    BufferedPort<HumanState> humanStateDataPort;
    yarp::dev::PolyDriver robotDriver;
    yarp::dev::IEncoders *robotEncoders;

    Publisher<tf2_msgs_TFMessage> tfPublisher;


    bool updateHumanPose;

    iDynTree::KinDynComputations kinDynComputations;
    iDynTree::VectorDynSize robotConfiguration;

    iDynTree::FrameIndex robotBaseLinkIndex;
    iDynTree::FrameIndex robotKinematicFrameIndex;

    iDynTree::Transform humanFrame_H_robotKinematicFrame;
    iDynTree::Transform ground_H_humanFrame;

public:

    RobotBasePosePublisher()
    : period(0.01)
    , rosSequenceCounter(1)
    , updateHumanPose(false) {}

    double getPeriod()
    {
        // module periodicity (seconds).
        return period;
    }
    // Main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {
        TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());

        // Read robot
        robotEncoders->getEncoders(robotConfiguration.data());
        iDynTree::toEigen(robotConfiguration) = M_PI / 180.0 * iDynTree::toEigen(robotConfiguration);
        kinDynComputations.setJointPos(robotConfiguration);

        iDynTree::Transform robotKinematicFrame_H_robotBase;

        {
            LockGuard lock(mutex);
            if (updateHumanPose) {
                // This part is done only if the user issues a command
                updateHumanPose = false;
                // Read Human pose wrt ground
                // As this is done only when the user asks to update the pose
                // wait for the reading
                HumanState* humanStateData = humanStateDataPort.read();

                if (humanStateData) {
                    iDynTree::Position pelvisOrigin(humanStateData->baseOriginWRTGlobal.x,
                                                    humanStateData->baseOriginWRTGlobal.y,
                                                    humanStateData->baseOriginWRTGlobal.z);

                    iDynTree::Vector4 pelvisQuaternion;
                    pelvisQuaternion(0) = humanStateData->baseOrientationWRTGlobal.w;
                    pelvisQuaternion(1) = humanStateData->baseOrientationWRTGlobal.imaginary.x;
                    pelvisQuaternion(2) = humanStateData->baseOrientationWRTGlobal.imaginary.y;
                    pelvisQuaternion(3) = humanStateData->baseOrientationWRTGlobal.imaginary.z;
                    iDynTree::Rotation pelvisOrientation = iDynTree::Rotation::RotationFromQuaternion(pelvisQuaternion);
                    ground_H_humanFrame.setPosition(pelvisOrigin);
                    ground_H_humanFrame.setRotation(pelvisOrientation);
                }

            }
            // Update transforms
            robotKinematicFrame_H_robotBase = kinDynComputations.getRelativeTransform(robotKinematicFrameIndex, robotBaseLinkIndex);
        }
        // Final transform is
        iDynTree::Transform ground_H_robotBase = ground_H_humanFrame * humanFrame_H_robotKinematicFrame * robotKinematicFrame_H_robotBase;

        iDynTree::Position ground_P_robotBase = ground_H_robotBase.getPosition();
        iDynTree::Vector4 ground_R_robotBase = ground_H_robotBase.getRotation().asQuaternion();

        tf2_msgs_TFMessage &tf = tfPublisher.prepare();
        tf.transforms.resize(1);
        tf.transforms[0].header.seq   = rosSequenceCounter++;
        tf.transforms[0].header.stamp = currentTime;
        tf.transforms[0].transform.translation.x = ground_P_robotBase(0);
        tf.transforms[0].transform.translation.y = ground_P_robotBase(1);
        tf.transforms[0].transform.translation.z = ground_P_robotBase(2);
        tf.transforms[0].transform.rotation.x = ground_R_robotBase(1);
        tf.transforms[0].transform.rotation.y = ground_R_robotBase(2);
        tf.transforms[0].transform.rotation.z = ground_R_robotBase(3);
        tf.transforms[0].transform.rotation.w = ground_R_robotBase(0);
        
        tfPublisher.write();
        

        return true;
    }

    // Configure function. 
    bool configure(ResourceFinder &rf)
    {
        string moduleName = rf.check("name", Value("robot-basepose-publisher"), "Checking module name").asString();
        setName(moduleName.c_str());

        int periodInMilliseconds = rf.check("period", Value(10), "Checking module period").asInt();
        period = periodInMilliseconds / 1000.0;

        // Reading joint list
        if (!rf.check("joint_list")) {
            yError("Cannot find the \"joint_list\" parameter");
            close();
            return false;
        }

        std::vector<std::string> jointList;
        parseFrameListOption(rf.find("joint_list"), jointList);

        //Open robot interface
        yarp::os::Bottle controlBoardGroup = rf.findGroup("CONTROLBOARD_REMAPPER");
        if(controlBoardGroup.isNull())
        {
            yError("Cannot find the CONTROLBOARD_REMAPPER group");
            close();
            return false;
        }

        yarp::os::Property options_robot;
        options_robot.fromString(controlBoardGroup.toString());

        yarp::os::Bottle axesNames;
        yarp::os::Bottle &axesList = axesNames.addList();
        for (std::vector<std::string>::const_iterator it(jointList.begin());
             it != jointList.end(); ++it)
        {
            axesList.addString(*it);
        }
        options_robot.put("axesNames", axesNames.get(0));
        options_robot.put("localPortPrefix", "/" + getName() + "/robot");

        // Open the polydriver for the joints configuration
        if (!robotDriver.open(options_robot))
        {
            yError("Error in opening the device for the robot!");
            close();
            return false;
        }

        if (!robotDriver.view(robotEncoders) || !robotEncoders) {
            yError("Error in opening the device for the robot!");
            close();
            return false;
        }

        // Loading URDF model
        string urdfModelFile = rf.findFile("urdf_model");
        
        // Load model
        iDynTree::ModelLoader modelLoader;
        iDynTree::Model model;
        if (!modelLoader.loadReducedModelFromFile(urdfModelFile, jointList)
            || !modelLoader.isValid()) {
            yError() << "Failed to load URDF model from file " << urdfModelFile;
            close();
            return false;
        }
        model = modelLoader.model();
        robotConfiguration.resize(model.getNrOfDOFs());
        kinDynComputations.loadRobotModel(model);

        if (!tfPublisher.topic(rf.find("tfTopicName").asString())) {
            yError() << "Failed to create publisher to /tf";
            close();
            return false;
        }

        std::string robotChildFrame;
        Bottle& robotGroup = rf.findGroup("ROBOT_FRAME");
        if (robotGroup.isNull()) {
            yError("Failed to find ROBOT_FRAME group");
            close();
            return false;
        }

        robotChildFrame = robotGroup.find("childLinkRefFrameName").asString();

        iDynTree::Rotation rotation;
        iDynTree::Position position;
        if (!parseRotationMatrix(robotGroup.find("transformOrientation"), rotation)) {
            yError() << "Failed to parse robot \"transformationOrientation\"";
            close();
            return false;
        }
        if (!parsePositionVector(robotGroup.find("transformOrigin"), position)) {
            yError() << "Failed to parse robot \"transformOrigin\"";
            close();
            return false;
        }

        std::string robotKinematicSourceFrame = robotChildFrame;
        if (robotGroup.check("kinematicSourceFrame")) {
            robotKinematicSourceFrame = robotGroup.find("kinematicSourceFrame").asString();
        }
        yInfo("Robot kinematic frame set to %s", robotKinematicSourceFrame.c_str());

        robotBaseLinkIndex = model.getFrameIndex(robotChildFrame);
        robotKinematicFrameIndex = model.getFrameIndex(robotKinematicSourceFrame);

        if (robotBaseLinkIndex < 0 || robotKinematicFrameIndex < 0) {
            yError("Failed to find specified links in robot model");
            close();
            return false;
        }

        humanFrame_H_robotKinematicFrame.setPosition(position);
        humanFrame_H_robotKinematicFrame.setRotation(rotation);
        ground_H_humanFrame.Identity();
        
        yInfo("Constant Robot To human transform:");
        yInfo() << humanFrame_H_robotKinematicFrame.toString();

        Value defaultAutoconn; defaultAutoconn.fromString("false");
        bool autoconnect = rf.check("autoconnect", defaultAutoconn, "Checking autoconnection mode").asBool();

        string stateReaderPortName = "/" + getName() + "/state:i";
        humanStateDataPort.open(stateReaderPortName);

        if (autoconnect){
            string stateProviderServerName = rf.find("stateprovider_name").asString();
            if (!Network::connect(stateProviderServerName.c_str(), stateReaderPortName))
            {
                yError() << "Error! Could not connect to server " << stateProviderServerName;
                close();
                return false;
            }
        }

        std::string rpcPortName = "/" + getName() + "/rpc";
        if (!rpcPort.open(rpcPortName) || ! attach(rpcPort)) {
            yError("Failed to open RPC port %s", rpcPortName.c_str());
            close();
            return false;
        }

        rosSequenceCounter = 1;

        tf2_msgs_TFMessage &tf = tfPublisher.prepare();
        tf.transforms.resize(1);
        tf.transforms[0].header.frame_id = rf.find("worldRFName").asString();
        tf.transforms[0].child_frame_id = robotChildFrame;
        tfPublisher.write();
	
	updateHumanPose = false;


        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
        humanStateDataPort.close();
	
        tfPublisher.interrupt();
        tfPublisher.close();
	
        rpcPort.interrupt();
        rpcPort.close();
        return true;
    }

    bool setRobotPose(std::string newKinematicFrame = "") {
        LockGuard lock(mutex);

        if (!newKinematicFrame.empty()) {
            iDynTree::FrameIndex newFrameIndex = kinDynComputations.model().getFrameIndex(newKinematicFrame);
            if (newFrameIndex < 0) return false;
            robotKinematicFrameIndex = newFrameIndex;
        }
        updateHumanPose = true;
        return true;
    }

    virtual bool respond (const Bottle &command, Bottle &reply)
    {
        reply.clear();
        if (command.size() == 1 && command.get(0).isString()) {
            //setRobotPose
            std::string cmd = command.get(0).asString();
            bool ok = true;
            if (cmd == "help") {
                reply.addString("commands: setRobotPose\n");
                reply.addString("\t\tsetKinFrame [newFrame]\n");
                return true;
            } else if (cmd != "setRobotPose") {
                ok = false;
            }
            ok = ok && setRobotPose();
            if (ok) {
                reply.addString("Ok");
            } else {
                reply.addString("Fail");
            }
            return true;
        } else if (command.size() == 2
                   && command.get(0).isString()
                   && command.get(1).isString()) {
            std::string cmd = command.get(0).asString();
            bool ok = true;

            if (cmd != "setKinFrame") {
                ok = false;
                setRobotPose();

            }
            ok = ok && setRobotPose(command.get(1).asString());
            if (ok) {
                reply.addString("Ok");
            } else {
                reply.addString("Fail");
            }

            return true;
        }


        reply.addString("Fail");
        return true;
    }
};

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    Network yarp;
    if (!Network::checkNetwork(5.0))
    {
        yError() << " YARP server not available!";
        return EXIT_FAILURE;
    }
    
    RobotBasePosePublisher module;
    ResourceFinder rf;
    rf.setDefaultConfigFile("robot-basepose-publisher.ini");
    rf.setDefaultContext("human-dynamic-estimation");
    rf.configure(argc, argv);
    rf.setVerbose(true);
    Node node(rf.find("nodeName").asString());
    module.runModule(rf);                                   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    node.interrupt();
    return 0;
}
