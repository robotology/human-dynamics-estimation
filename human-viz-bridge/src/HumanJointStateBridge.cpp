#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_TransformStamped.h"
#include "geometry_msgs_Vector3.h"
#include "msgs/String.h"
#include "sensor_msgs_JointState.h"
#include "std_msgs_Header.h"
#include "tf2_msgs_TFMessage.h"
#include "thrift/XsensDriverService.h"
#include "thrift/XsensSegmentsFrame.h"
#include "thrifts/HumanState.h"

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <TickTime.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>

#include <cmath>
#include <iostream>
#include <limits.h>
#include <string>
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace xsens;
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

class xsensJointStatePublisherModule : public RFModule//, public TypedReaderCallback<HumanState> 
{
    Port client_port;
    BufferedPort<HumanState> humanStateDataPort;

    Publisher<sensor_msgs_JointState> publisher;
    Publisher<tf2_msgs_TFMessage> publisher_tf;
    
    //variables declaration
    sensor_msgs_JointState joint_state;
    tf2_msgs_TFMessage tf;
    geometry_msgs_TransformStamped ground_to_Pelvis;
    double degree;
    double inc, var;
    
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
        string serverName = rf.check("serverName", Value("readxsens"), "Checking server name").asString();
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

        if (!publisher.topic("/human/joint_states")) {
            cerr<< "Failed to create publisher to /joint_states\n";
            return false;
        }
        
        if (!publisher_tf.topic("/tf")) {
        cerr<< "Failed to create publisher to /tf\n";
        return -1;
        }
        degree = 3.1416/360;
        inc = degree; 
        var = 0;
        
        ground_to_Pelvis.header.frame_id = "ground";
        ground_to_Pelvis.child_frame_id = "Pelvis";
        
        vector<string> joints;
        joints.reserve(model.getNrOfJoints());
        for (iDynTree::JointIndex jointIndex = 0; jointIndex < model.getNrOfJoints(); ++jointIndex) {
            string jointName = model.getJointName(jointIndex);
            joints.push_back(jointName);
        }
        
        joint_state.name.resize(model.getNrOfJoints());
        

//         for (size_t index = 0; index < joints.size(); ++index) {
//             joint_state.name[index] = joints[index];
//         }
        
        joint_state.name[0] ="jL5S1_rotx";
        joint_state.name[1] ="jL5S1_roty";
        joint_state.name[2] ="jL4L3_rotx";
        joint_state.name[3] ="jL4L3_roty";
        joint_state.name[4] ="jL1T12_rotx";
        joint_state.name[5] ="jL1T12_roty";
        joint_state.name[6] ="jT9T8_rotx";
        joint_state.name[7] ="jT9T8_roty";
        joint_state.name[8] ="jT9T8_rotz";
        joint_state.name[9] ="jT1C7_rotx";
        joint_state.name[10] ="jT1C7_roty";
        joint_state.name[11] ="jT1C7_rotz";
        joint_state.name[12] ="jC1Head_rotx";
        joint_state.name[13] ="jC1Head_roty";
        joint_state.name[14] ="jRightC7Shoulder_rotx";
        joint_state.name[15] ="jRightShoulder_rotx";
        joint_state.name[16] ="jRightShoulder_roty";
        joint_state.name[17] ="jRightShoulder_rotz";
        joint_state.name[18] ="jRightElbow_roty";
        joint_state.name[19] ="jRightElbow_rotz";
        joint_state.name[20] ="jRightWrist_rotx";
        joint_state.name[21] ="jRightWrist_rotz";
        joint_state.name[22] ="jLeftC7Shoulder_rotx";
        joint_state.name[23] ="jLeftShoulder_rotx";
        joint_state.name[24] ="jLeftShoulder_roty";
        joint_state.name[25] ="jLeftShoulder_rotz";
        joint_state.name[26] ="jLeftElbow_roty";
        joint_state.name[27] ="jLeftElbow_rotz";
        joint_state.name[28] ="jLeftWrist_rotx";
        joint_state.name[29] ="jLeftWrist_rotz";
        joint_state.name[30] ="jRightHip_rotx";
        joint_state.name[31] ="jRightHip_roty";
        joint_state.name[32] ="jRightHip_rotz";
        joint_state.name[33] ="jRightKnee_roty";
        joint_state.name[34] ="jRightKnee_rotz";
        joint_state.name[35] ="jRightAnkle_rotx";
        joint_state.name[36] ="jRightAnkle_roty";
        joint_state.name[37] ="jRightAnkle_rotz";
        joint_state.name[38] ="jRightBallFoot_roty";
        joint_state.name[39] ="jLeftHip_rotx";
        joint_state.name[40] ="jLeftHip_roty";
        joint_state.name[41] ="jLeftHip_rotz";
        joint_state.name[42] ="jLeftKnee_roty";
        joint_state.name[43] ="jLeftKnee_rotz";
        joint_state.name[44] ="jLeftAnkle_rotx";
        joint_state.name[45] ="jLeftAnkle_roty";
        joint_state.name[46] ="jLeftAnkle_rotz";
        joint_state.name[47] ="jLeftBallFoot_roty";
        
        yInfo() << "Joints: " << model.getNrOfJoints() << joints;
        
        joint_state.position.resize(model.getNrOfJoints());
        joint_state.velocity.resize(model.getNrOfJoints());
        joint_state.effort.resize(model.getNrOfJoints());
        
        //humanStateDataPort.useCallback(*this);
        string stateProviderServerName = "/" + rf.find("serverName").asString() + "/state:o";
        string stateReaderPortName = "/" + getName() + "/state:i";
        humanStateDataPort.open(stateReaderPortName);           
        if (!Network::connect(stateProviderServerName.c_str(),stateReaderPortName))
        {
            yError() << "Error! Could not connect to server " << stateProviderServerName;
            return false;
        }
        
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
        humanStateDataPort.close();
        return true;
    }
    
    virtual void onRead(HumanState& humanStateData) {
    
//         TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());
//         joint_state.header.stamp = currentTime;
//         
//         yInfo() << humanStateData.positions.toString();
//         return;
//         
// //         for (size_t index = 0; index < joint_state.position.size(); ++index){
// //             joint_state.position[index] = humanStateData.positions[index];
// //         }
//         
//         joint_state.position[39] = var;
//         var += inc;
//         if (var<-0.3 || var>0.3) inc *= -1;
//         
//         publisher.write(joint_state);
//         
//         tf.transforms.clear();
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
    
    Node node("/human/joint_state_publisher");

    xsensJointStatePublisherModule module;
    ResourceFinder rf;
    rf.setDefaultConfigFile("human-jointstate-bridge.ini");
    rf.setDefaultContext("human-dynamic-estimation");
    rf.configure(argc, argv);
    rf.setVerbose(true);
    module.runModule(rf);                                   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    return 0;
}
