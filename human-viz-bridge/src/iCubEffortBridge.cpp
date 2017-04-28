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

struct Joint {
    string jointName;
    int nrDoFs;
    vector<string> jointDoFs;
};

struct EffortPublisher {
    string linkEffort;
    Joint jointEffort;
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
    vector<EffortPublisher> effortsList;
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
        
        for (size_t index = 0; index < effortsList.size(); ++index) {
            EffortPublisher &effort = effortsList[index];
            if (!effort.publisher) return false;
            //sensor_msgs_Temperature &effortMsg = effort.publisher->prepare();
            effort.publisher->prepare();
            effort.effortMsg.header.stamp = currentTime;
            
            double effort_temp = 0;
            for (size_t indexJoint = 0; indexJoint < effortsList[index].jointEffort.nrDoFs; ++indexJoint) {
                effort_temp += std::abs(robotTorques.getVal(index+indexJoint));
            }
            effort.effortMsg.temperature = effort_temp;
            //effortMsg = effort.effortMsg; 
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

        size_t effortIndex = 0;
        size_t DoFiterator = 1;
        string robotJoint = "*";
        vector<string> jointDegrees = {"_pitch", "_roll", "_yaw"};
        for (size_t jointIndex = 0; jointIndex < robot_jointList.size(); jointIndex++) {
            
            effortsList.resize(effortIndex+1);
            effortsList[effortIndex].jointEffort.nrDoFs = 0;
            DoFiterator = 0;
            
            if (robot_jointList[jointIndex].find(robotJoint)==string::npos) {
               if(robot_jointList[jointIndex].find(jointDegrees[0])!=string::npos || robot_jointList[jointIndex].find(jointDegrees[1])!=string::npos || robot_jointList[jointIndex].find(jointDegrees[2])!=string::npos ) 
               {
                   robotJoint = robot_jointList[jointIndex];
                   size_t found = robotJoint.find("_");
                   size_t erasePos = robotJoint.find("_",(found+1));
                   cout << robotJoint << endl;
                   if (erasePos < 100) {
                       effortsList[effortIndex].jointEffort.jointName = robotJoint.erase(erasePos);
                   } else {
                       effortsList[effortIndex].jointEffort.jointName = robotJoint.erase(found);
                   }
                   effortsList[effortIndex].linkEffort = robotModel.getLinkName(traversal.getChildLinkIndexFromJointIndex(robotModel, robotModel.getJointIndex(robot_jointList[jointIndex])));
                   bool sameJoint = robot_jointList[(jointIndex+(DoFiterator))].find(effortsList[effortIndex].jointEffort.jointName)!=string::npos; 
                   while(sameJoint){
                       effortsList[effortIndex].jointEffort.jointDoFs.resize(DoFiterator+1);
                       effortsList[effortIndex].jointEffort.jointDoFs[(DoFiterator)] = robot_jointList[(jointIndex+(DoFiterator))];
                       effortsList[effortIndex].jointEffort.nrDoFs++;
                       DoFiterator++;
                       if((jointIndex+(DoFiterator))>=robot_jointList.size()) {
                           sameJoint = 0;
                       } else {
                       sameJoint = robot_jointList[(jointIndex+(DoFiterator))].find(effortsList[effortIndex].jointEffort.jointName)!=string::npos;
                       }
                   }
                   ++effortIndex;
               }
               else 
               {
                   robotJoint = robot_jointList[jointIndex];
                   cout << robotJoint << endl;
                   effortsList[effortIndex].jointEffort.jointName = robot_jointList[jointIndex];
                   effortsList[effortIndex].linkEffort = robotModel.getLinkName(traversal.getChildLinkIndexFromJointIndex(robotModel, robotModel.getJointIndex(robot_jointList[jointIndex])));
                   effortsList[effortIndex].jointEffort.nrDoFs++;
                   effortsList[effortIndex].jointEffort.jointDoFs.resize(effortsList[effortIndex].jointEffort.nrDoFs);
                   effortsList[effortIndex].jointEffort.jointDoFs[0] = robot_jointList[jointIndex];
                   ++effortIndex;
               }
            }
        }

        for (size_t index = 0; index < effortsList.size(); ++index) {
            std::string topicName = rf.find("topicPrefix").asString() + "/" + effortsList[index].jointEffort.jointName;
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
        
        for (size_t index = 0; index < effortsList.size(); ++index) {
            EffortPublisher &effort = effortsList[index];
//             effort.effortMsg.header.frame_id = rf.find("tfPrefix").asString() + "/" + effortsList[index].linkEffort;
            effort.effortMsg.header.frame_id = effortsList[index].linkEffort;
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
