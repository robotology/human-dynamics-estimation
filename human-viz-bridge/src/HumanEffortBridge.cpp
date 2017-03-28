#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_TransformStamped.h"
#include "geometry_msgs_Vector3.h"
#include "msgs/String.h"
#include "sensor_msgs_JointState.h"
#include "sensor_msgs_Temperature.h"
#include "std_msgs_Header.h"
#include "tf2_msgs_TFMessage.h"
#include "thrifts/HumanState.h"
#include "thrifts/HumanDynamics.h"

#include <iDynTree/ModelIO/ModelLoader.h>
#include <TickTime.h>
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
#include <iostream>
#include <limits.h>
#include <string>
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
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

class xsensJointStatePublisherModule : public RFModule, public TypedReaderCallback<HumanDynamics> 
{
    Port client_port;
//     BufferedPort<HumanState> humanStateDataPort;
    BufferedPort<HumanDynamics> humanDynamicsDataPort;

//     Publisher<sensor_msgs_JointState> publisher;
//     Publisher<tf2_msgs_TFMessage> publisher_tf;
    vector< Publisher<sensor_msgs_Temperature> > publisher_vec;
    
    Publisher<sensor_msgs_Temperature> tempL1_pub; 
    Publisher<sensor_msgs_Temperature> tempL2_pub; 
    Publisher<sensor_msgs_Temperature> tempL3_pub; 
    Publisher<sensor_msgs_Temperature> tempL4_pub; 
    Publisher<sensor_msgs_Temperature> tempL5_pub; 
    Publisher<sensor_msgs_Temperature> tempL6_pub; 
    
    Publisher<sensor_msgs_Temperature> tempR1_pub; 
    Publisher<sensor_msgs_Temperature> tempR2_pub; 
    Publisher<sensor_msgs_Temperature> tempR3_pub; 
    Publisher<sensor_msgs_Temperature> tempR4_pub; 
    Publisher<sensor_msgs_Temperature> tempR5_pub; 
    Publisher<sensor_msgs_Temperature> tempR6_pub;      
    
//     sensor_msgs_JointState joint_state;
//     tf2_msgs_TFMessage tf;
    map<string,sensor_msgs_Temperature> sensorMsgsEffort_map;
    vector<string> jointsFrameEffort;
    vector<string> jointEffort;
    
    Value defaultReducedModel;
    
    int var, inc;
    
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
        string moduleName = rf.check("name", Value("human-effort-bridge"), "Checking module name").asString();
        string serverName = rf.check("serverName", Value("human-dynamics-estimator"), "Checking server name").asString();
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

//         if (!publisher.topic(rf.find("topicPrefix").asString() + rf.find("jointStateTopicName").asString())) {
//             yError() << "Failed to create publisher to /joint_states";
//             return false;
//         }
//         
//         if (!publisher_tf.topic(rf.find("tfTopicName").asString())) {
//             yError() << "Failed to create publisher to /tf";
//             return false;
//         }
        
        if (!parseFrameListOption(rf.find("frameEffortList"), jointsFrameEffort)) {
            yError() << "Error while parsing joints effort list";
            return false;
        }
        
        yInfo() << "Joints frame effort:" << jointsFrameEffort.size() << jointsFrameEffort;
        
        vector<string> URDFjoints;
        URDFjoints.reserve(model.getNrOfJoints());
        for (iDynTree::JointIndex jointIndex = 0; jointIndex < model.getNrOfJoints(); ++jointIndex) {
            string jointName = model.getJointName(jointIndex);
            URDFjoints.push_back(jointName);
        }
        
        defaultReducedModel.fromString("true");
        bool redModel = rf.check("reducedModel", defaultReducedModel, "Checking reduced model mode").asBool();
        if(!redModel) 
        {
            if (!parseFrameListOption(rf.find("jointEffortList"), jointEffort)) {
                yError() << "Error while parsing joints effort list";
                return false;
            }
        } else {
            if (!parseFrameListOption(rf.find("jointRedEffortList"), jointEffort)) {
                yError() << "Error while parsing joints effort list";
                return false;
            }
        }
        
        yInfo() << "Joints effort:" << jointEffort.size() << jointEffort;

//         publisher_vec.resize(1);

//         if(!publisher_vec[0].topic("/topic")) {
//             yError() << "Failed to create publisher to" << jointsEffort[0];
//             return false;
//         }
        
//         for (size_t index = 0; index < jointsEffort.size(); ++index) {
//             if (!publisher_vec[index].topic(rf.find("topicPrefix").asString() + "/" + jointsEffort[index])) {
//                 yError() << "Failed to create publisher to" << jointsEffort[index];
//                 return false;
//             }
//         }
        
        tempL1_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[0]);
        tempL2_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[1]);
        tempL3_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[2]);
        tempL4_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[3]);
        tempL5_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[4]);
        tempL6_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[5]);
                                                                
        tempR1_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[6]);
        tempR2_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[7]);
        tempR3_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[8]);
        tempR4_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[9]);
        tempR5_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[10]);
        tempR6_pub.topic(rf.find("topicPrefix").asString() + "/" + jointsFrameEffort[11]);
        
//         tf.transforms.resize(1);
//         tf.transforms[0].header.frame_id = rf.find("worldRFName").asString();
//         tf.transforms[0].child_frame_id = rf.find("tfPrefix").asString() + "/" + rf.find("childLinkRFName").asString();
//         
//         vector<string> joints;
//         if (!parseFrameListOption(rf.find("jointList"), joints)) {
//             yError() << "Error while parsing joints list";
//             return false;
//         }
//         
//         yInfo() << "Joints from config file: " << model.getNrOfJoints() << joints;
//        
//         vector<string> URDFjoints;
//         URDFjoints.reserve(model.getNrOfJoints());
//         for (iDynTree::JointIndex jointIndex = 0; jointIndex < model.getNrOfJoints(); ++jointIndex) {
//             string jointName = model.getJointName(jointIndex);
//             if (!(find(joints.begin(), joints.end(), jointName) == joints.end())) {
//             URDFjoints.push_back(jointName);
//                 if ((URDFjoints[jointIndex].compare(joints[jointIndex]))) {
//                     yError() << "URDF joints is different from the order of the received joints";
//                     return false;
//                 }
//             }
//             else 
//             {
//                 yError() << "URDF joints and received joints do not match";
//                 return false;   
//             }
//         }
//         
//         yInfo() << "Joints from URDF: " << URDFjoints;
//         
//         joint_state.name.resize(model.getNrOfJoints());
//         
//         for (size_t index = 0; index < URDFjoints.size(); ++index) {
//             joint_state.name[index] = URDFjoints[index];
//         }
//         
//         joint_state.position.resize(model.getNrOfJoints());
//         joint_state.velocity.resize(model.getNrOfJoints());
//         joint_state.effort.resize(model.getNrOfJoints());
//                 
//         humanStateDataPort.useCallback(*this);
//         string stateProviderServerName = "/" + rf.find("serverName").asString() + "/state:o";
//         string stateReaderPortName = "/" + getName() + "/state:i";
//         humanStateDataPort.open(stateReaderPortName);
//         Value defaultAutoconn; defaultAutoconn.fromString("true");
//         bool autoconn = rf.check("automaticConnection", defaultAutoconn, "Checking autoconnection mode").asBool();
//         if(autoconn){
//             if (!Network::connect(stateProviderServerName.c_str(),stateReaderPortName))
//             {
//                 yError() << "Error! Could not connect to server " << stateProviderServerName;
//                 return false;
//             }
//         }
        
        humanDynamicsDataPort.useCallback(*this);
        string dynamicsEstimatorServerName = "/" + rf.find("serverName").asString() + "/dynamicsEstimation:o";
        string dynamicsReaderPortName = "/" + getName() + "/state:i";
        humanDynamicsDataPort.open(dynamicsReaderPortName);
        Value defaultAutoconn; defaultAutoconn.fromString("true");
        bool autoconn = rf.check("automaticConnection", defaultAutoconn, "Checking autoconnection mode").asBool();
        if(autoconn){
            if (!Network::connect(dynamicsEstimatorServerName.c_str(),dynamicsReaderPortName))
            {
                yError() << "Error! Could not connect to server " << dynamicsEstimatorServerName;
                return false;
            }
        }
        
        
        for (size_t index = 0; index < jointsFrameEffort.size(); ++index) {
        sensorMsgsEffort_map[jointsFrameEffort[index]].header.frame_id = rf.find("tfPrefix").asString() + "/" + jointsFrameEffort[index];
        sensorMsgsEffort_map[jointsFrameEffort[index]].header.seq = 1;
        sensorMsgsEffort_map[jointsFrameEffort[index]].variance = 0;
        }
        
        var = 0;
        inc = 1;
        
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
//         humanStateDataPort.close();
//         publisher.interrupt();
//         publisher.close();
//         publisher_tf.interrupt();
//         publisher_tf.close();
        humanDynamicsDataPort.close();
        tempL1_pub.close(); tempL1_pub.interrupt();
        tempL2_pub.close(); tempL2_pub.interrupt();
        tempL3_pub.close(); tempL3_pub.interrupt();
        tempL4_pub.close(); tempL4_pub.interrupt();
        tempL5_pub.close(); tempL5_pub.interrupt();
        tempL6_pub.close(); tempL6_pub.interrupt();
        tempR1_pub.close(); tempR1_pub.interrupt();
        tempR2_pub.close(); tempR2_pub.interrupt();
        tempR3_pub.close(); tempR3_pub.interrupt();
        tempR4_pub.close(); tempR4_pub.interrupt();
        tempR5_pub.close(); tempR5_pub.interrupt();
        tempR6_pub.close(); tempR6_pub.interrupt();
        return true;
    }
    
    virtual void onRead(HumanDynamics& humanDynamicsData) {
    
        TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());
//         joint_state.header.stamp = currentTime;
//         
//         for (size_t index = 0; index < joint_state.position.size(); ++index){
//             joint_state.position[index] = humanStateData.positions[index];
//         }
//         
//         publisher.write(joint_state);
// 
//         tf.transforms[0].header.seq   = 1;
//         tf.transforms[0].header.stamp = currentTime;
//         tf.transforms[0].transform.translation.x = humanStateData.baseOriginWRTGlobal.x;
//         tf.transforms[0].transform.translation.y = humanStateData.baseOriginWRTGlobal.y;
//         tf.transforms[0].transform.translation.z = humanStateData.baseOriginWRTGlobal.z;
//         tf.transforms[0].transform.rotation.x = humanStateData.baseOrientationWRTGlobal.imaginary.x;
//         tf.transforms[0].transform.rotation.y = humanStateData.baseOrientationWRTGlobal.imaginary.y;
//         tf.transforms[0].transform.rotation.z = humanStateData.baseOrientationWRTGlobal.imaginary.z;
//         tf.transforms[0].transform.rotation.w = humanStateData.baseOrientationWRTGlobal.w;
// 
//         publisher_tf.write(tf);
        
        var += inc;
        if (var<0 || var>100) inc *= -1;
        
        
        
        for (size_t index = 0; index < jointsFrameEffort.size(); ++index) {
            sensorMsgsEffort_map[jointsFrameEffort[index]].header.stamp = currentTime;
            //sensorMsgsEffort_map[jointsFrameEffort[index]].temperature = var;          
        }
        
        tempL1_pub.write(sensorMsgsEffort_map[jointsFrameEffort[0]]);
        tempL2_pub.write(sensorMsgsEffort_map[jointsFrameEffort[1]]);
        tempL3_pub.write(sensorMsgsEffort_map[jointsFrameEffort[2]]);
        tempL4_pub.write(sensorMsgsEffort_map[jointsFrameEffort[3]]);
        tempL5_pub.write(sensorMsgsEffort_map[jointsFrameEffort[4]]);
        tempL6_pub.write(sensorMsgsEffort_map[jointsFrameEffort[5]]);
                                            
        tempR1_pub.write(sensorMsgsEffort_map[jointsFrameEffort[6]]);
        tempR2_pub.write(sensorMsgsEffort_map[jointsFrameEffort[7]]);
        tempR3_pub.write(sensorMsgsEffort_map[jointsFrameEffort[8]]);
        tempR4_pub.write(sensorMsgsEffort_map[jointsFrameEffort[9]]);
        tempR5_pub.write(sensorMsgsEffort_map[jointsFrameEffort[10]]);
        tempR6_pub.write(sensorMsgsEffort_map[jointsFrameEffort[11]]);

        
        //publisher_vec[0].write(temp);
        
//         for (size_t index = 0; index < jointsEffort.size(); ++index) {
//             publisher_vec[index].write(sensorMsgsEffort_map[jointsEffort[index]]);
//         }

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
    rf.setDefaultConfigFile("human-effort-bridge.ini");
    rf.setDefaultContext("human-dynamic-estimation");
    rf.configure(argc, argv);
    rf.setVerbose(true);
    Node node(rf.find("nodeName").asString());
    module.runModule(rf);                                   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    return 0;
}
