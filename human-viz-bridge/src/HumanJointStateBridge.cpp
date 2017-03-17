#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_TransformStamped.h"
#include "geometry_msgs_Vector3.h"
#include "msgs/String.h"
#include "sensor_msgs_JointState.h"
#include "std_msgs_Header.h"
#include "tf2_msgs_TFMessage.h"
#include "thrifts/HumanState.h"

#include <iDynTree/ModelIO/ModelLoader.h>
#include <TickTime.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>

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


class xsensJointStatePublisherModule : public RFModule, public TypedReaderCallback<HumanState> 
{
    Port client_port;
    BufferedPort<HumanState> humanStateDataPort;

    Publisher<sensor_msgs_JointState> publisher;
    Publisher<tf2_msgs_TFMessage> publisher_tf;
    
    //variables declaration
    sensor_msgs_JointState joint_state;
    tf2_msgs_TFMessage tf;

public:
    double getPeriod()
    {
        // module periodicity (seconds).
        return 0.1;
    }
    // Main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {
        std::cerr << "hi\n";
        HumanState* humanStateData = humanStateDataPort.read();
//         HumanState humanStateData;
//         humanStateData.positions.resize(5);
//         humanStateData.positions[0] = (1.0); 
//         humanStateData.positions[1] = (1.4); 
//         humanStateData.positions.push_back(4.5); 
//         humanStateData.positions.push_back(2.3); 
//         humanStateData.positions.push_back(5.6);
        std::cerr << "after\n";
        if (!humanStateData) return true;
        double we = humanStateData->positions[0];
        yInfo() << we;
        
//         tf.transforms.clear();
//         TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());
//         joint_state.header.stamp = currentTime;
//         joint_state.position[39] = var;
//         var += inc;
//         if (var<-0.3 || var>0.3) inc *= -1;
//         
//         ground_to_Pelvis.header.seq   = 1;
//         ground_to_Pelvis.header.stamp = currentTime;
//         ground_to_Pelvis.transform.translation.x = 0;
//         ground_to_Pelvis.transform.translation.y = 0;
//         ground_to_Pelvis.transform.translation.z = 0;
//         ground_to_Pelvis.transform.rotation.x = 0;
//         ground_to_Pelvis.transform.rotation.y = 0;
//         ground_to_Pelvis.transform.rotation.z = 0;
//         ground_to_Pelvis.transform.rotation.w = 1;
// 
//         tf.transforms.push_back(ground_to_Pelvis);
//         publisher_tf.write(tf);
//         publisher.write(joint_state);
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
            return false;
        }
        
        if (!publisher_tf.topic(rf.find("tfTopicName").asString())) {
            yError() << "Failed to create publisher to /tf";
            return false;
        }

        tf.transforms.resize(1);
        tf.transforms[0].header.frame_id = rf.find("worldRFName").asString();
        tf.transforms[0].child_frame_id = rf.find("tfPrefix").asString() + "/" + rf.find("childLinkRFName").asString();
        
        vector<string> joints;
        if (!parseFrameListOption(rf.find("jointList"), joints)) {
            yError() << "Error while parsing joints list";
            return false;
        }

        Value defaultAutoconn; defaultAutoconn.fromString("false");
        bool autoconnect = rf.check("autoconnect", defaultAutoconn, "Checking autoconnection mode").asBool();

        yInfo() << "Joints from config file: " << model.getNrOfJoints() << joints;

        vector<string> URDFjoints;
        URDFjoints.reserve(model.getNrOfJoints());
        for (iDynTree::JointIndex jointIndex = 0; jointIndex < model.getNrOfJoints(); ++jointIndex) {
            string jointName = model.getJointName(jointIndex);
            if (!(find(joints.begin(), joints.end(), jointName) == joints.end())) {
            URDFjoints.push_back(jointName);
                if ((URDFjoints[jointIndex].compare(joints[jointIndex]))) {
                    yError() << "URDF joints is different from the order of the received joints";
                    return false;
                }
            }
            else 
            {
                yError() << "URDF joints and received joints do not match";
                return false;   
            }
        }
        
        yInfo() << "Joints from URDf: " << URDFjoints[0].compare(joints[0]) << URDFjoints;
        
        joint_state.name.resize(model.getNrOfJoints());
        
        for (size_t index = 0; index < URDFjoints.size(); ++index) {
            joint_state.name[index] = URDFjoints[index];
        }
        
        joint_state.position.resize(model.getNrOfJoints());
        joint_state.velocity.resize(model.getNrOfJoints());
        joint_state.effort.resize(model.getNrOfJoints());
        
        humanStateDataPort.useCallback(*this);
        string serverName = rf.check("stateprovider_name", Value("human-state-provider"), "Checking server name").asString();
        string stateProviderServerName = "/" + rf.find("serverName").asString() + "/state:o";
        string stateReaderPortName = "/" + getName() + "/state:i";
        humanStateDataPort.open(stateReaderPortName);

        if (autoconnect){
            if (!Network::connect(stateProviderServerName.c_str(), stateReaderPortName))
            {
                yError() << "Error! Could not connect to server " << stateProviderServerName;
                return false;
            }
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
        return true;
    }
    
    virtual void onRead(HumanState& humanStateData) {
    
        TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());
        joint_state.header.stamp = currentTime;
        
        for (size_t index = 0; index < joint_state.position.size(); ++index){
            joint_state.position[index] = humanStateData.positions[index];
        }
        
        publisher.write(joint_state);

        tf.transforms[0].header.seq   = 1;
        tf.transforms[0].header.stamp = currentTime;
        tf.transforms[0].transform.translation.x = humanStateData.baseOriginWRTGlobal.x;
        tf.transforms[0].transform.translation.y = humanStateData.baseOriginWRTGlobal.y;
        tf.transforms[0].transform.translation.z = humanStateData.baseOriginWRTGlobal.z;
        tf.transforms[0].transform.rotation.x = humanStateData.baseOrientationWRTGlobal.imaginary.x;
        tf.transforms[0].transform.rotation.y = humanStateData.baseOrientationWRTGlobal.imaginary.y;
        tf.transforms[0].transform.rotation.z = humanStateData.baseOrientationWRTGlobal.imaginary.z;
        tf.transforms[0].transform.rotation.w = humanStateData.baseOrientationWRTGlobal.w;

        publisher_tf.write(tf);

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
    return 0;
}
