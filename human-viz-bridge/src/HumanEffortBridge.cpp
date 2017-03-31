#include "sensor_msgs_Temperature.h"
#include "std_msgs_Header.h"
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
#include <iostream>
#include <limits.h>
#include <string>
#include <vector>
#include <cmath>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace human;

struct EffortPublisher {
    string linkEffort;
    string jointEffort;
    vector<int> indices;
    vector<string> realJointsNames;
    Publisher<sensor_msgs_Temperature> *publisher;
    sensor_msgs_Temperature effortMsg;
};

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
    BufferedPort<HumanDynamics> humanDynamicsDataPort;
    
    vector<EffortPublisher> effortsList;

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
        
        vector<string> linksEffortList;
        if (!parseFrameListOption(rf.find("linkEffortList"), linksEffortList)) {
            yError() << "Error while parsing links effort list";
            return false;
        }
        
        yInfo() << "efforts links list:" << linksEffortList.size() << linksEffortList;
        
        vector<string> URDFjoints;
        URDFjoints.reserve(model.getNrOfJoints());
        for (iDynTree::JointIndex jointIndex = 0; jointIndex < model.getNrOfJoints(); ++jointIndex) {
            string jointName = model.getJointName(jointIndex);
            URDFjoints.push_back(jointName);
        }
        sort(URDFjoints.begin(), URDFjoints.end());
        
        vector<string> jointsEffortList;
        if (!parseFrameListOption(rf.find("jointEffortList"), jointsEffortList)) {
            yError() << "Error while parsing joints effort list";
            return false;
        }

        yInfo() << "efforts joints list:" << jointsEffortList.size() << jointsEffortList;
        
        if (linksEffortList.size() != jointsEffortList.size()) {
            yError() << "links list and joints list have different lengths";
            return false;
        }
        
        effortsList.resize(linksEffortList.size());
        for (size_t index = 0; index < linksEffortList.size(); ++index) {
            effortsList[index].linkEffort = linksEffortList[index];
            effortsList[index].jointEffort = jointsEffortList[index];
            for (size_t indexURDF = 0; indexURDF < URDFjoints.size(); ++indexURDF) {
                if(URDFjoints[indexURDF].find(jointsEffortList[index])!=string::npos) {
                    effortsList[index].realJointsNames.push_back(URDFjoints[indexURDF]);
                    effortsList[index].indices.push_back(indexURDF);    
                }
            }
        }
        

        for (size_t index = 0; index < effortsList.size(); ++index) {
            std::string topicName = rf.find("topicPrefix").asString() + "/" + effortsList[index].jointEffort;
            effortsList[index].publisher = new Publisher<sensor_msgs_Temperature>();
            if (!effortsList[index].publisher) {
                yError() << "Failed to create publisher to" << effortsList[index].linkEffort;
                return false;
            }
            if (!effortsList[index].publisher->topic(topicName)) {
                yError() << "Failed to create publisher to" << effortsList[index].linkEffort;
                return false;
            }
        }

        humanDynamicsDataPort.useCallback(*this);
        string dynamicsEstimatorServerName = rf.find("serverName").asString();
        string dynamicsReaderPortName = "/" + getName() + "/dynamicsEstimation:i";
        humanDynamicsDataPort.open(dynamicsReaderPortName);
        Value defaultAutoconn; defaultAutoconn.fromString("false");
        bool autoconn = rf.check("autoconnect", defaultAutoconn, "Checking autoconnection mode").asBool();
        if(autoconn){
            if (!Network::connect(dynamicsEstimatorServerName.c_str(),dynamicsReaderPortName))
            {
                yError() << "Error! Could not connect to server " << dynamicsEstimatorServerName;
                return false;
            }
        }
        
        for (size_t index = 0; index < effortsList.size(); ++index) {
            EffortPublisher &effort = effortsList[index];
            effort.effortMsg.header.frame_id = rf.find("tfPrefix").asString() + "/" + effortsList[index].linkEffort;
            effort.effortMsg.header.seq = 1;
            effort.effortMsg.variance = 0;
        }
        
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
        humanDynamicsDataPort.close();

        for (size_t index = 0; index < effortsList.size(); ++index) {
            EffortPublisher &effort = effortsList[index];
            if (effort.publisher) {        
                effort.publisher->interrupt();
                effort.publisher->close();
                delete effort.publisher;
                effort.publisher = 0;
            }
        }
        effortsList.clear();
        return true;
    }
    
    virtual void onRead(HumanDynamics& humanDynamicsData) {
        
        TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());
        
        for (size_t index = 0; index < effortsList.size(); ++index) {
            EffortPublisher &effort = effortsList[index];
            if (!effort.publisher) return;
            sensor_msgs_Temperature &effortMsg = effort.publisher->prepare();
            effort.effortMsg.header.stamp = currentTime;
            
            double effort_temp = 0;
            for (size_t indexJoint = 0; indexJoint < effortsList[index].indices.size(); ++indexJoint) {
                effort_temp += std::abs(humanDynamicsData.jointVariables[effortsList[index].indices[indexJoint]].torque[0]);
            }
            effort.effortMsg.temperature = effort_temp;
            effortMsg = effort.effortMsg; 
            effort.publisher->write();
        }

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
    module.runModule(rf);
    node.interrupt();
    // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    return 0;
}
