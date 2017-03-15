#include <iostream>
#include <iomanip>
#include <limits.h>
#include <cmath>
#include <string>
#include <vector>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/all.h>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

using namespace std;
using namespace yarp::os;

#include "msgs/String.h"
#include "std_msgs_Header.h"
#include "TickTime.h"
#include "geometry_msgs_Vector3.h"
#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_TransformStamped.h"
#include "tf2_msgs_TFMessage.h"
#include <thrift/XsensSegmentsFrame.h>
#include "thrift/XsensDriverService.h"

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

class xsensTfPublisherModule : public RFModule, public yarp::os::TypedReaderCallback<xsens::XsensSegmentsFrame> 
{
    Port client_port;
    yarp::os::BufferedPort<xsens::XsensSegmentsFrame> xsensDataPort;
    xsens::XsensDriverService xsensDriver;
    
    Publisher<tf2_msgs_TFMessage> publisher_tf;
    
    //variables declaration
    int i, segmentsCount, trueSegmentsCount, offline;
    tf2_msgs_TFMessage tf;
    std::vector<std::string> segments;
    std::vector<xsens::FrameReferece> xsensSegments;
    
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
    // Message handler. 
    bool respond(const Bottle& command, Bottle& reply)
    {
        if (command.get(0).asString() == "quit")
        return false;
        else
        reply = command;
        return true;
    }
    // Configure function. 
    bool configure(yarp::os::ResourceFinder &rf)
    {
        std::string moduleName = rf.check("name", yarp::os::Value("xsensTFBridge"), "Checking module name").asString();
        std::string serverName = rf.check("serverName", yarp::os::Value("xsens"), "Checking module name").asString();
        setName(moduleName.c_str());
        
        if (!publisher_tf.topic("/tf")) {
            cerr<< "Failed to create publisher to /tf\n";
            return false;
        }
        
        i = 0;
//         segmentsCount = rf.find("segmentsNumber").asInt(); //49
//         trueSegmentsCount = rf.find("trueSegmentsNumber").asInt();
        segmentsCount = 49; //49
        trueSegmentsCount = 23;
        tf.transforms.resize(segmentsCount);                           //49
        offline = 0;

        // fake segments list received from the XsensDriverService
        segments.resize(trueSegmentsCount);
        segments[0] = "Pelvis";
        segments[1] = "L5";
        segments[2] = "L3";
        segments[3] = "T12";
        segments[4] = "T8";
        segments[5] = "Neck";
        segments[6] = "Head";
        segments[7] = "RightShoulder";
        segments[8] = "RightUpperArm";
        segments[9] = "RightForeArm";
        segments[10] = "RightHand";
        segments[11] = "LeftShoulder";
        segments[12] = "LeftUpperArm";
        segments[13] = "LeftForeArm";
        segments[14] = "LeftHand";
        segments[15] = "RightUpperLeg";
        segments[16] = "RightLowerLeg";
        segments[17] = "RightFoot";
        segments[18] = "RightToe";
        segments[19] = "LeftUpperLeg";
        segments[20] = "LeftLowerLeg";
        segments[21] = "LeftFoot";
        segments[22] = "LeftToe";
        
        if (offline != 0) {
            //std:: string driverServerName = "/" + rf.find("serverName").asString() + "/cmd:i"; 
            std::string driverServerName = "/" + serverName + "/frames:o";
            std::string segmentsListReaderPortName = "/" + getName() + "/segmentsList:i";
            client_port.open(segmentsListReaderPortName);
            if (!Network::connect(segmentsListReaderPortName,driverServerName.c_str()))
            {
               std::cout << "Error! Could not connect to server " << driverServerName << std::endl;
               return -1;
            }
            xsensDriver.yarp().attachAsClient(client_port);
            xsensSegments = xsensDriver.segments();
            for (i=0;i<trueSegmentsCount;i++){
               segments[i] = xsensSegments[i].frameName;
            }
        }

        for (i=0;i<segmentsCount;i++){
            tf.transforms[i].header.frame_id = "ground";
          }

        for (i=0;i<trueSegmentsCount;i++){
            tf.transforms[i].child_frame_id = segments[i];
          }

        tf.transforms[23].child_frame_id = "L5_f1";
        tf.transforms[24].child_frame_id = "L3_f1";
        tf.transforms[25].child_frame_id = "T12_f1";
        tf.transforms[26].child_frame_id = "T8_f1";
        tf.transforms[27].child_frame_id = "T8_f2";
        tf.transforms[28].child_frame_id = "Neck_f1";
        tf.transforms[29].child_frame_id = "Neck_f2";
        tf.transforms[30].child_frame_id = "Head_f1";
        tf.transforms[31].child_frame_id = "RightUpperArm_f1";
        tf.transforms[32].child_frame_id = "RightUpperArm_f2";
        tf.transforms[33].child_frame_id = "RightForeArm_f1";
        tf.transforms[34].child_frame_id = "RightHand_f1";
        tf.transforms[35].child_frame_id = "LeftUpperArm_f1";
        tf.transforms[36].child_frame_id = "LeftUpperArm_f2";
        tf.transforms[37].child_frame_id = "LeftForeArm_f1";
        tf.transforms[38].child_frame_id = "LeftHand_f1";
        tf.transforms[39].child_frame_id = "RightUpperLeg_f1";
        tf.transforms[40].child_frame_id = "RightUpperLeg_f2";
        tf.transforms[41].child_frame_id = "RightLowerLeg_f1";
        tf.transforms[42].child_frame_id = "RightFoot_f1";
        tf.transforms[43].child_frame_id = "RightFoot_f2";
        tf.transforms[44].child_frame_id = "LeftUpperLeg_f1";
        tf.transforms[45].child_frame_id = "LeftUpperLeg_f2";
        tf.transforms[46].child_frame_id = "LeftLowerLeg_f1";
        tf.transforms[47].child_frame_id = "LeftFoot_f1";
        tf.transforms[48].child_frame_id = "LeftFoot_f2";
       
        for (i=trueSegmentsCount;i<(segmentsCount);i++){
            tf.transforms[i].header.seq   = i;
            tf.transforms[i].transform.translation.x = 0;
            tf.transforms[i].transform.translation.y = 0;
            tf.transforms[i].transform.translation.z = 0;
            tf.transforms[i].transform.rotation.x = 0;
            tf.transforms[i].transform.rotation.y = 0;
            tf.transforms[i].transform.rotation.z = 0;
            tf.transforms[i].transform.rotation.w = 1;
          }
        
        xsensDataPort.useCallback(*this);
        //std::string xsensServerName = "/" + rf.find("serverName").asString() + "/frames:o";
        std::string xsensServerName = "/" + serverName + "/frames:o";
        std::string frameReaderPortName = "/" + getName() + "/frames:i";
        xsensDataPort.open(frameReaderPortName);           
        if (!Network::connect(xsensServerName.c_str(),frameReaderPortName))
        {
            std::cout << "Error! Could not connect to server " << xsensServerName << std::endl;
            return false;
        }

        return true;
    }
    // Interrupt function.
    bool interruptModule()
    {
        cout << "Interrupting your module, for port cleanup" << endl;
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
        // optional, close port explicitly
        cout << "Calling close function\n";
        return true;
    }
    
    virtual void onRead(xsens::XsensSegmentsFrame& xsensData) {
    
        // Send the transforms
        for (i=0;i<trueSegmentsCount;i++){
            tf.transforms[i].header.seq   = i;
            tf.transforms[i].header.stamp = normalizeSecNSec(yarp::os::Time::now());
            tf.transforms[i].transform.translation.x = xsensData.segmentsData[i].position.x;
            tf.transforms[i].transform.translation.y = xsensData.segmentsData[i].position.y;
            tf.transforms[i].transform.translation.z = xsensData.segmentsData[i].position.z;
            tf.transforms[i].transform.rotation.x = xsensData.segmentsData[i].orientation.imaginary.x;
            tf.transforms[i].transform.rotation.y = xsensData.segmentsData[i].orientation.imaginary.y;
            tf.transforms[i].transform.rotation.z = xsensData.segmentsData[i].orientation.imaginary.z;
            tf.transforms[i].transform.rotation.w = xsensData.segmentsData[i].orientation.w;
        }
          
        for (i=trueSegmentsCount;i<(segmentsCount);i++){
            tf.transforms[i].header.stamp = normalizeSecNSec(yarp::os::Time::now());
        }
        publisher_tf.write(tf);
        
    }
};

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    Network yarp;
    if (!yarp::os::Network::checkNetwork(5.0))
    {
        yError() << " YARP server not available!";
        return EXIT_FAILURE;
    }
    
    Node node("/human/yarp_human_state_publisher");

    xsensTfPublisherModule module;
    ResourceFinder rf;
//     rf.setDefaultConfigFile("xsensTFBridge.ini");
//     rf.setDefaultContext("human-dynamic-estimation");
    rf.configure(argc, argv);
    rf.setVerbose(true);
    module.runModule(rf);                                   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    return 0;
}