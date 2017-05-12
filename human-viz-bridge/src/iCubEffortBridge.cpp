#include "sensor_msgs_Temperature.h"
#include "std_msgs_Header.h"

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <TickTime.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Property.h>
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
using namespace yarp::dev;

//---------------------------------------------------------------------------
//Utility functions for parsing INI file
static bool parseStringListOption(const Value &option, vector<string> &parsedList);
//---------------------------------------------------------------------------

struct EffortPublisher {
    string linkEffort;
    vector<size_t> jointDoFs;
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

class iCubEffortPublisherModule : public RFModule 
{
    iDynTree::VectorDynSize robotTorques;
    PolyDriver m_robot;
    ITorqueControl *robotITorqueControl;
    map<std::string, EffortPublisher> effortMap;
    int period;

public:
    double getPeriod()
    {
        // module periodicity (seconds).
        return period;
    }
    // Main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {
        TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());
        
        robotITorqueControl->getTorques(robotTorques.data());
        
        for (auto& effortList : effortMap) {
            EffortPublisher &effort = effortList.second;
            if (!effort.publisher) return false;
            effort.publisher->prepare();
            double effort_temp;
            for (size_t& effortIdx : effort.jointDoFs) {
                effort_temp += std::abs(robotTorques.getVal(effortIdx));
            }
            effort.effortMsg.temperature = effort_temp;
            effort.publisher->write();  
        }
        
        return true;
    }
    // Configure function. 
    bool configure(ResourceFinder &rf)
    {
        
        period = rf.check("period", Value(0.1), "Checking period value").asInt();
        string moduleName = rf.check("name", Value("icub-effort-bridge"), "Checking module name").asString();
        string urdfModelFile = rf.findFile("robotModelFilename");
        setName(moduleName.c_str());
        
        //Robot model
        const string robotModelFilename = rf.findFile("robotModelFilename");

        vector<string> robot_jointList;
        if (!parseStringListOption(rf.find("robot_JointList"), robot_jointList))
        {
            yError("Error while parsing the robot joint list");
            return false;
        }

        iDynTree::ModelLoader modelLoader;
              
        if(!modelLoader.loadReducedModelFromFile(robotModelFilename, robot_jointList))
        {
            yError("Something wrong with the robot model loading!");
            return false;
        }
    
        iDynTree::Model robotModel = modelLoader.model(); 
        iDynTree::Traversal traversal;
        robotModel.computeFullTreeTraversal(traversal, robotModel.getDefaultBaseLink());
        robotTorques.resize(robot_jointList.size());
        robotTorques.zero();
        
        vector<string> jointSuffixes = {"_pitch", "_roll", "_yaw"};
        for (size_t jointIndex = 0; jointIndex < robot_jointList.size(); jointIndex++) {
            
            string robotJoint = robot_jointList[jointIndex];
            
            size_t suffixStartingIndex = std::string::npos;
            for (const std::string& suffix : jointSuffixes) {
                suffixStartingIndex = robotJoint.find(suffix);
                if (suffixStartingIndex != std::string::npos) {
                    break;
                }
            }
            
            //two possible cases:
            // - suffixStartingIndex != std::string::npos
            // - suffixStartingIndex == std::string::npos
            std::string mapKey = robotJoint;
            if (suffixStartingIndex != std::string::npos) {
                mapKey = mapKey.erase(suffixStartingIndex);
            }
            
            map<std::string, EffortPublisher>::iterator effortPublisherFound = effortMap.find(mapKey);
            if (effortPublisherFound == effortMap.end()) {
                // create a new EffortPub
                EffortPublisher newEffortPublisher;
                // populate it with the correct fields
                newEffortPublisher.linkEffort = robotModel.getLinkName(traversal.getChildLinkIndexFromJointIndex(robotModel, robotModel.getJointIndex(robotJoint)));
                
                // move here the new Publisher....
                // insert it into the map,
                std::pair<map<std::string, EffortPublisher>::iterator, bool> inserted = effortMap.insert(map<std::string, EffortPublisher>::value_type(mapKey, newEffortPublisher));
                
                //get back the iterator to this element
                effortPublisherFound = inserted.first;
                
            }
            
            // do the "common" stuff
            effortPublisherFound->second.jointDoFs.push_back(jointIndex);
            cout << effortMap.find(mapKey)->first << " in the reference frame: " << effortMap.find(mapKey)->second.linkEffort << endl;
             
        }
        
        for (auto& effortList : effortMap) {
            std::string topicName = rf.find("topicPrefix").asString() + "/" + effortList.first;
            effortList.second.publisher = new Publisher<sensor_msgs_Temperature>();
            if (!effortList.second.publisher) {
                yError() << "Failed to create publisher to" << effortList.second.linkEffort;
                return false;
            }
            if (!effortList.second.publisher->topic(topicName)) {
                yError() << "Failed to create publisher to" << effortList.second.linkEffort;
                return false;
            }
            
        }
        
        for (auto& effortList : effortMap) {
            cout << effortMap.find(effortList.first)->first << " in the reference frame: " << effortMap.find(effortList.first)->second.linkEffort << endl;
            EffortPublisher &effort = effortList.second;
            effort.effortMsg.header.frame_id = effortList.second.linkEffort;
            effort.effortMsg.header.seq = 1;
            effort.effortMsg.variance = 0;
        }
            
        robotITorqueControl = 0;
        if (!getRobotTorquesInterface(rf, robot_jointList, robotITorqueControl) || !robotITorqueControl)
        {
            yError("Failed to open robot ITorqueControl interface");
            return false;
        }
        
        
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
        for (auto& effortList : effortMap) {
            EffortPublisher &effort = effortList.second;
            if (effort.publisher) {        
                effort.publisher->interrupt();
                effort.publisher->close();
                delete effort.publisher;
                effort.publisher = 0;
            }
        }
        effortMap.clear();
        
        m_robot.close();
        return true;
    }
    
    bool getRobotTorquesInterface(const yarp::os::Searchable& config,
                                  const std::vector<std::string>& robot_jointList,
                                  yarp::dev::ITorqueControl *& torques)
    {
        /*
         * ------Configure the autoconnect option
         */
        yarp::os::Value falseValue;
        falseValue.fromString("false");
        bool autoconnect = config.check("autoconnect",
                                    falseValue,
                                    "Checking the autoconnect option").asBool();
        
        /*
         * ------Robot configuration for the RemoteControlBoardRemapper
         */
        yarp::os::Bottle controlBoardGroup = config.findGroup("CONTROLBOARD_REMAPPER");
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
        for (std::vector<std::string>::const_iterator it = robot_jointList.begin(); it != robot_jointList.end(); ++it)
        {
            axesList.addString(*it);
        }
        options_robot.put("axesNames", axesNames.get(0));
        options_robot.put("localPortPrefix", "/" + getName() + "/robot");
        
        //TODO: find a better way to express VOCAB
        //1987212385 is the int cast for VOCAB REVOLUTE JOINT TYPE
        
        // Open the polydriver for the joints configuration
        if (!m_robot.open(options_robot))
        {
            yError("Error in opening the device for the robot!");
            close();
            return false;
        }
        
        
        if (!m_robot.view(torques) || !torques)
        {
            yError("ControlBoard does not support ITorqueControl interface");
            close();
            return false;
        }
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
    
    iCubEffortPublisherModule module;
    ResourceFinder rf;
    rf.setDefaultConfigFile("icub-effort-bridge.ini");
    rf.setDefaultContext("human-dynamic-estimation");
    rf.configure(argc, argv);
    rf.setVerbose(true);
    Node node(rf.find("nodeName").asString());
    module.runModule(rf);
    node.interrupt();
    // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    return 0;
}

static bool parseStringListOption(const yarp::os::Value &option, std::vector<std::string> &parsedList)
{
    if (option.isNull() || !option.isList() || !option.asList()) return false;
    yarp::os::Bottle *frames = option.asList();
    parsedList.reserve(static_cast<size_t>(frames->size()));
    
    for (int i = 0; i < frames->size(); ++i)
    {
        if (frames->get(i).isString())
        {
            parsedList.push_back(frames->get(i).asString());
        }
    }
    return true;
}
