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

#include <algorithm>
#include <cmath>
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


class xsensJointStatePublisherModule : public RFModule, public TypedReaderCallback<HumanState> 
{
    Port rpcPort;
    BufferedPort<HumanState> humanStateDataPort;

    Publisher<sensor_msgs_JointState> publisher;
    Publisher<tf2_msgs_TFMessage> publisher_tf;
    sensor_msgs_JointState joint_state;
    tf2_msgs_TFMessage tf;
    Mutex mutex;

    bool hasRobotFrame;
    iDynTree::Transform robotBaseLinkWRTGround;
    iDynTree::Transform robotBaseLinkWRTPelvis;
    iDynTree::Transform humanPelvisWRTGround;

public:
    double getPeriod()
    {
        // module periodicity (seconds).
        return 100;
    }
    // Main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {
        return true;
    }
    // Configure function. 
    bool configure(ResourceFinder &rf)
    {
        string moduleName = rf.check("name", Value("human-jointstate-bridge"), "Checking module name").asString();
        string urdfModelFile = rf.findFile("urdf_model");
        setName(moduleName.c_str());
        
        // Load model
        iDynTree::ModelLoader modelLoader;
        iDynTree::Model model;
        if (!modelLoader.loadModelFromFile(urdfModelFile)) {
            yError() << "Failed to load URDF model from file " << urdfModelFile;
            return false;
        }
        
        model = modelLoader.model();

        if (!publisher.topic(rf.find("jointStateTopicName").asString())) {
            yError() << "Failed to create publisher to /joint_states";
            close();
            return false;
        }
        
        if (!publisher_tf.topic(rf.find("tfTopicName").asString())) {
            yError() << "Failed to create publisher to /tf";
            close();
            return false;
        }

        hasRobotFrame = false;
        std::string robotChildFrame;
        Bottle& robotGroup = rf.findGroup("ROBOT_FRAME");
        if (!robotGroup.isNull()) {
            hasRobotFrame = true;
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

            robotBaseLinkWRTPelvis.setPosition(position);
            robotBaseLinkWRTPelvis.setRotation(rotation);
            humanPelvisWRTGround.Identity();
            setRobotPose();
        }

        tf.transforms.resize(1);

        if (hasRobotFrame) {
           tf.transforms.resize(2);
           tf.transforms[1].header.frame_id = rf.find("worldRFName").asString();
           tf.transforms[1].child_frame_id = robotChildFrame;
        }
        tf.transforms[0].header.frame_id = rf.find("worldRFName").asString();
        tf.transforms[0].child_frame_id = rf.find("tfPrefix").asString() + "/" + rf.find("human_childLinkRFName").asString();
       

        vector<string> joints;
        Value jointListValue = rf.find("jointList");
        if (jointListValue.isString()) {
            string configJointFile = rf.findFileByName(jointListValue.asString());
            Property config;
            if (config.fromConfigFile(configJointFile, true)) {
                if (!parseFrameListOption(config.find("jointList"), joints)) {
                    yError() << "Error while parsing joints list";
                    close();
                    return false;
                }
            } else {
                yError() << "Could not parse " << configJointFile;
                close();
                return false;
            }
        }
        else if (jointListValue.isList()) {
            if (!parseFrameListOption(jointListValue, joints)) {
                yError() << "Error while parsing joints list";
                close();
                return false;
            }
        } else {
            yError() << "\"jointList\" parameter malformed";
            close();
            return false;
        }

        Value defaultAutoconn; defaultAutoconn.fromString("false");
        bool autoconnect = rf.check("autoconnect", defaultAutoconn, "Checking autoconnection mode").asBool();

        yInfo() << "Joints from config file: " << joints.size() << joints;

        vector<string> URDFjoints;
        URDFjoints.reserve(model.getNrOfJoints());
        for (iDynTree::JointIndex jointIndex = 0; jointIndex < model.getNrOfJoints(); ++jointIndex) {
            string jointName = model.getJointName(jointIndex);
            if (!(find(joints.begin(), joints.end(), jointName) == joints.end())) {
            URDFjoints.push_back(jointName);
                if ((URDFjoints[jointIndex].compare(joints[jointIndex]))) {
                    yError() << "URDF joints is different from the order of the received joints";
                    close();
                    return false;
                }
            }
            else 
            {
                yError() << "URDF joints and received joints do not match";
                close();
                return false;   
            }
        }
        
        yInfo() << "Joints from URDF: " << URDFjoints.size() << URDFjoints;
        
        joint_state.name.resize(model.getNrOfJoints());
        
        for (size_t index = 0; index < URDFjoints.size(); ++index) {
            joint_state.name[index] = URDFjoints[index];
        }
        
        joint_state.position.resize(model.getNrOfJoints());
        joint_state.velocity.resize(model.getNrOfJoints());
        joint_state.effort.resize(model.getNrOfJoints());
        
        humanStateDataPort.useCallback(*this);
        string serverName = rf.check("stateprovider_name", Value("human-state-provider"), "Checking server name").asString();
        string stateProviderServerName = "/" + rf.find("stateprovider_name").asString() + "/state:o";
        string stateReaderPortName = "/" + getName() + "/state:i";
        humanStateDataPort.open(stateReaderPortName);

        if (autoconnect){
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

        
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
        humanStateDataPort.close();
        publisher.interrupt();
        publisher.close();
        publisher_tf.interrupt();
        publisher_tf.close();
        rpcPort.interrupt();
        rpcPort.close();
        return true;
    }
    
    virtual void onRead(HumanState& humanStateData) {
    
        TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());
        tf2_msgs_TFMessage &tfMsg = publisher_tf.prepare();
        sensor_msgs_JointState &jointStateMsg = publisher.prepare();
        joint_state.header.stamp = currentTime;
        
        for (size_t index = 0; index < joint_state.position.size(); ++index){
            joint_state.position[index] = humanStateData.positions[index];
        }
        jointStateMsg = joint_state;
        
        publisher.write();

        LockGuard lock(mutex);

        iDynTree::Position pelvisOrigin(humanStateData.baseOriginWRTGlobal.x,
                                        humanStateData.baseOriginWRTGlobal.y,
                                        humanStateData.baseOriginWRTGlobal.z);

        iDynTree::Vector4 pelvisQuaternion;
        pelvisQuaternion(0) = humanStateData.baseOrientationWRTGlobal.w;
        pelvisQuaternion(1) = humanStateData.baseOrientationWRTGlobal.imaginary.x;
        pelvisQuaternion(2) = humanStateData.baseOrientationWRTGlobal.imaginary.y;
        pelvisQuaternion(3) = humanStateData.baseOrientationWRTGlobal.imaginary.z;
        iDynTree::Rotation pelvisOrientation = iDynTree::Rotation::RotationFromQuaternion(pelvisQuaternion);
        humanPelvisWRTGround.setPosition(pelvisOrigin);
        humanPelvisWRTGround.setRotation(pelvisOrientation);


        tf.transforms[0].header.seq   = 1;
        tf.transforms[0].header.stamp = currentTime;
        tf.transforms[0].transform.translation.x = humanStateData.baseOriginWRTGlobal.x;
        tf.transforms[0].transform.translation.y = humanStateData.baseOriginWRTGlobal.y;
        tf.transforms[0].transform.translation.z = humanStateData.baseOriginWRTGlobal.z;
        tf.transforms[0].transform.rotation.x = humanStateData.baseOrientationWRTGlobal.imaginary.x;
        tf.transforms[0].transform.rotation.y = humanStateData.baseOrientationWRTGlobal.imaginary.y;
        tf.transforms[0].transform.rotation.z = humanStateData.baseOrientationWRTGlobal.imaginary.z;
        tf.transforms[0].transform.rotation.w = humanStateData.baseOrientationWRTGlobal.w;

        if (hasRobotFrame) {
            const iDynTree::Position& position = robotBaseLinkWRTGround.getPosition();
            const iDynTree::Vector4& orientation = robotBaseLinkWRTGround.getRotation().asQuaternion();
            tf.transforms[1].header.seq   = 1;
            tf.transforms[1].header.stamp = currentTime;
            tf.transforms[1].transform.translation.x = position(0);
            tf.transforms[1].transform.translation.y = position(1);
            tf.transforms[1].transform.translation.z = position(2);
            tf.transforms[1].transform.rotation.x = orientation(1);
            tf.transforms[1].transform.rotation.y = orientation(2);
            tf.transforms[1].transform.rotation.z = orientation(3);
            tf.transforms[1].transform.rotation.w = orientation(0);
        }

        tfMsg = tf;
        
        publisher_tf.write();
    }

    void setRobotPose() {
        LockGuard lock(mutex);
        robotBaseLinkWRTGround = robotBaseLinkWRTPelvis * humanPelvisWRTGround;

    }

    virtual bool respond (const Bottle &command, Bottle &reply)
    {
        reply.clear();
        if (command.size() == 1 && command.get(0).isString()) {
            //setRobotPose
            std::string cmd = command.get(0).asString();
            if (cmd == "help") {
                reply.addString("commands: setRobotPose");
            } else if (cmd == "setRobotPose") {
                //
                setRobotPose();
                reply.addString("ok");
            } else {
                reply.addString("Fail");
                return true;
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
    
    xsensJointStatePublisherModule module;
    ResourceFinder rf;
    rf.setDefaultConfigFile("human-jointstate-bridge.ini");
    rf.setDefaultContext("human-dynamic-estimation");
    rf.configure(argc, argv);
    rf.setVerbose(true);
    Node node(rf.find("nodeName").asString());
    module.runModule(rf);                                   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    node.interrupt();
    return 0;
}
