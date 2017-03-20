#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_TransformStamped.h"
#include "geometry_msgs_Vector3.h"
#include "msgs/String.h"
#include "std_msgs_Header.h"
#include "tf2_msgs_TFMessage.h"
#include "thrift/XsensDriverService.h"
#include "thrift/XsensSegmentsFrame.h"

#include <iDynTree/Model/Indeces.h>
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
using namespace xsens;

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

class HumanTFBridge : public RFModule, public TypedReaderCallback<XsensSegmentsFrame> 
{
    Port client_port;
    BufferedPort<XsensSegmentsFrame> xsensDataPort;
    XsensDriverService xsensDriver;
    
    Publisher<tf2_msgs_TFMessage> publisher_tf;
    
    //variables declaration
    tf2_msgs_TFMessage tf;
    vector<string> segments;
    vector<string> fakeSegments;
    
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
        string moduleName = rf.check("name", Value("human-tf-bridge"), "Checking module name").asString();
        string serverName = rf.check("serverName", Value("xsens"), "Checking server name").asString();
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

        if (!publisher_tf.topic(rf.find("tfTopicName").asString())) {
            yError() << "Failed to create publisher to /tf";
            return false;
        }
        
        tf.transforms.resize(model.getNrOfLinks());
        
        Value defaultOffline; defaultOffline.fromString("true");
        bool offline = rf.check("offline", defaultOffline, "Checking offline mode").asBool();
        
        if (!offline) {
            
            string driverServerName = "/" + rf.find("serverName").asString() + "/cmd:i"; 
            string segmentsListReaderPortName = "/" + getName() + "/segmentsList:o";
            client_port.open(segmentsListReaderPortName);
            if (!Network::connect(segmentsListReaderPortName,driverServerName.c_str()))
            {
               yError() << "Error! Could not connect to server " << driverServerName;
               return false;
            }
            xsensDriver.yarp().attachAsClient(client_port);
            vector<FrameReferece> xsensSegments = xsensDriver.segments();
            
            segments.reserve(xsensSegments.size());
            
            for (vector<FrameReferece>::const_iterator element(xsensSegments.begin());
                 element != xsensSegments.end(); ++element) {
                segments.push_back(element->frameName);
            }
        } 
        else 
        {
            if (!parseFrameListOption(rf.find("segmentList"), segments)) {
                yError() << "Error while parsing segments list";
                return false;
            }
        }   
        
        vector<string> fakeSegments;
        fakeSegments.reserve(model.getNrOfLinks() - segments.size());
        for (iDynTree::LinkIndex linkIndex = 0; linkIndex < model.getNrOfLinks(); ++linkIndex) {
            string linkName = model.getLinkName(linkIndex);
            if (find(segments.begin(), segments.end(), linkName) == segments.end()) {
                fakeSegments.push_back(linkName);
            }
        }
        
        yInfo() << "Segments: " << segments;
        yInfo() << "Fake Segments: " << fakeSegments;
        
        string worldRFName = rf.find("worldRFName").asString();
        for (size_t index = 0; index < segments.size(); ++index) {
            tf.transforms[index].child_frame_id = rf.find("tfPrefix").asString() + "/" + segments[index];
            tf.transforms[index].header.frame_id = worldRFName;
        }
        
        for (size_t index = 0; index < fakeSegments.size(); ++index) {
            tf.transforms[segments.size() + index].child_frame_id = rf.find("tfPrefix").asString() + "/" + fakeSegments[index];
            tf.transforms[segments.size() + index].header.frame_id = worldRFName;
            tf.transforms[segments.size() + index].header.seq   = index;
            tf.transforms[segments.size() + index].transform.translation.x = 0;
            tf.transforms[segments.size() + index].transform.translation.y = 0;
            tf.transforms[segments.size() + index].transform.translation.z = 0;
            tf.transforms[segments.size() + index].transform.rotation.x = 0;
            tf.transforms[segments.size() + index].transform.rotation.y = 0;
            tf.transforms[segments.size() + index].transform.rotation.z = 0;
            tf.transforms[segments.size() + index].transform.rotation.w = 1;
        }
        
        xsensDataPort.useCallback(*this);
        string xsensServerName = "/" + rf.find("serverName").asString() + "/frames:o";
        string frameReaderPortName = "/" + getName() + "/frames:i";
        xsensDataPort.open(frameReaderPortName);       
        Value defaultAutoconn; defaultAutoconn.fromString("true");
        bool autoconn = rf.check("automaticConnection", defaultAutoconn, "Checking autoconnection mode").asBool();
        if(autoconn){
            if (!Network::connect(xsensServerName.c_str(),frameReaderPortName))
            {
                yError() << "Error! Could not connect to server " << xsensServerName;
                return false;
            }
        }
        
        
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
        client_port.close();
        xsensDataPort.close();
        publisher_tf.interrupt();
        publisher_tf.close();
        return true;
    }
    
    virtual void onRead(XsensSegmentsFrame& xsensData) {
    
        TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());
        // Send the transforms
        for (size_t index = 0; index < segments.size(); ++index){
            tf.transforms[index].header.seq   = index;
            tf.transforms[index].header.stamp = currentTime;
            tf.transforms[index].transform.translation.x = xsensData.segmentsData[index].position.x;
            tf.transforms[index].transform.translation.y = xsensData.segmentsData[index].position.y;
            tf.transforms[index].transform.translation.z = xsensData.segmentsData[index].position.z;
            tf.transforms[index].transform.rotation.x = xsensData.segmentsData[index].orientation.imaginary.x;
            tf.transforms[index].transform.rotation.y = xsensData.segmentsData[index].orientation.imaginary.y;
            tf.transforms[index].transform.rotation.z = xsensData.segmentsData[index].orientation.imaginary.z;
            tf.transforms[index].transform.rotation.w = xsensData.segmentsData[index].orientation.w;
        }
          
         for (size_t index = 0; index < fakeSegments.size(); ++index) {
            tf.transforms[segments.size() + index].header.stamp = currentTime;
        }
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
    
    HumanTFBridge module;
    ResourceFinder rf;
    rf.setDefaultConfigFile("human-tf-bridge.ini");
    rf.setDefaultContext("human-dynamic-estimation");
    rf.configure(argc, argv);
    rf.setVerbose(true);
    Node node(rf.find("nodeName").asString());
    module.runModule(rf);                                   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    node.interrupt();
    return 0;
}
