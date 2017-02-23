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

#include "String.h"
#include "std_msgs_Header.h"
#include "TickTime.h"
#include "geometry_msgs_Vector3.h"
#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_TransformStamped.h"
#include "tf2_msgs_TFMessage.h"
#include <thrift/XsensFrame.h>
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

class MyModule : public RFModule, public yarp::os::TypedReaderCallback<xsens::XsensFrame>
{
    Port handlerPort; // a port to handle messages
    Port client_port;
    
    Publisher<tf2_msgs_TFMessage> publisher_tf;
    
    yarp::os::BufferedPort<xsens::XsensFrame> xsensDataPort;
    xsens::XsensDriverService xsensDriver;
    
    //variables declaration
    int i, segmentsCount, trueSegmentCount;
    
    //tf_tfMessage tfMessage;
    tf2_msgs_TFMessage tf;

    std::string xsensServerName;
    std::string driverServerName;
    std::vector<std::string> segments;
    
public:
    double getPeriod()
    {
        // module periodicity (seconds), called implicitly by the module.
        return 0.1;
    }
    // This is our main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {
        cout << "waiting for input" << endl;
        xsens::XsensFrame *xsensData = xsensDataPort.read();
        if (xsensData!=NULL) {
            cout << "got xsens data " << endl;
            }

        // Send the transforms
        for (i=0;i<trueSegmentCount;i++){
        tf.transforms[i].header.seq   = i;
        tf.transforms[i].header.stamp = normalizeSecNSec(xsensData->time);
        tf.transforms[i].transform.translation.x = xsensData->segmentsData[i].position.c1;
        tf.transforms[i].transform.translation.y = xsensData->segmentsData[i].position.c2;
        tf.transforms[i].transform.translation.z = xsensData->segmentsData[i].position.c3;
        tf.transforms[i].transform.rotation.x = xsensData->segmentsData[i].orientation.c2;
        tf.transforms[i].transform.rotation.y = xsensData->segmentsData[i].orientation.c3;
        tf.transforms[i].transform.rotation.z = xsensData->segmentsData[i].orientation.c4;
        tf.transforms[i].transform.rotation.w = xsensData->segmentsData[i].orientation.c1;
        }
        
        for (i=(trueSegmentCount-1);i<segmentsCount;i++){
        tf.transforms[i].header.seq   = i;
        tf.transforms[i].header.stamp = normalizeSecNSec(xsensData->time);
        tf.transforms[i].transform.translation.x = 0;
        tf.transforms[i].transform.translation.y = 0;
        tf.transforms[i].transform.translation.z = 0;
        tf.transforms[i].transform.rotation.x = 0;
        tf.transforms[i].transform.rotation.y = 0;
        tf.transforms[i].transform.rotation.z = 0;
        tf.transforms[i].transform.rotation.w = 1;
        }

        publisher_tf.write(tf);

        i++;
        for (i=0;i<segmentsCount;i++){
        cout << tf.transforms[i].child_frame_id << endl;
        cout << "position [" << tf.transforms[i].transform.translation.x << " " << tf.transforms[i].transform.translation.x << " " << tf.transforms[i].transform.translation.x << "]" << endl;
        cout << "orientation [" << tf.transforms[i].transform.rotation.w << " " << tf.transforms[i].transform.rotation.x << " " << tf.transforms[i].transform.rotation.y << " " << tf.transforms[i].transform.rotation.z << "]" << endl;
        }
        xsensDataPort.write();
        return true;
    }
    // Message handler. Just echo all received messages.
    bool respond(const Bottle& command, Bottle& reply)
    {
        cout << "Got something, echo is on" << endl;
        if (command.get(0).asString() == "quit")
            return false;
        else
            reply = command;
        return true;
    }
    // Configure function. Receive a previously initialized
    // resource finder object. Use it to configure your module.
    // If you are migrating from the old module, this is the function
    // equivalent to the "open" method.
    bool configure(yarp::os::ResourceFinder &rf)
    {     
        if (!publisher_tf.topic("/tf")) {
        cerr<< "Failed to create publisher to /tf\n";
        return -1;
        }
        
        xsensDataPort.open("/xsens_data_reader");
        xsensServerName = "/xsens/frames:o"; // >>yarp conf 10.255.36.7 10000 >> yarp rpc /xsens/cmd:i >>yarp read ... /xsens/frames:o
        if (!Network::connect("/xsens_data_writer","/xsens_data_reader"))
        {
           std::cout << "Error! Could not connect to server " << xsensServerName << std::endl;
           return -1;
        }
        //xsensDataPort.useCallback(*this);
        
        // fake segments list received from the XsensDriverService
        segments.resize(23);
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
        
        i = 0;
        segmentsCount = 49; //49
        trueSegmentCount = 23;
        tf.transforms.resize(49); //49
        
        for (i=0;i<segmentsCount;i++){
        tf.transforms[i].header.frame_id = "ground";
        }
        
        for (i=0;i<trueSegmentCount;i++){
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

        handlerPort.open("/myModule");
        attach(handlerPort);

        driverServerName = "/xsens/cmd:i"; // >>yarp conf 10.255.36.7 10000 >> yarp rpc /xsens/cmd:i >>yarp read ... /xsens/frames:o
        client_port.open("/XsensDriverService/client");
//         if (!Network::connect("/XsensDriverService/client",driverServerName.c_str()))
//         {
//            std::cout << "Error! Could not connect to server " << driverServerName << std::endl;
//            return -1;
//         }
        xsensDriver.yarp().attachAsClient(client_port);
        cout << xsensDriver.segments() << endl;

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
        handlerPort.close();
        return true;
    }
    
     virtual void onRead(xsens::XsensFrame& b) {
        // process data in b
    }
};

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    Network yarp;
    Node node("/human/yarp_human_state_publisher");
    
    /* create your module */
    MyModule module;
    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setVerbose(true);
    cout << "Configuring and starting module. \n";
    module.runModule(rf);   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    cout<<"Main returning..."<<endl;
    return 0;
}