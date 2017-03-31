/*!
 * @file HumanForcesProvider.cpp
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#include "HumanForcesProvider.h"
#include "ForceReader.h"
#include "AbstractForceReader.h"
#include "FTForceReader.h"
#include "PortForceReader.h"
#include "FrameTransformer.h"
#include "GenericFrameTransformer.h"
#include "RobotFrameTransformer.h"

#include <yarp/os/LogStream.h> 
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Property.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Time.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <TickTime.h>
#include <iterator>


//---------------------------------------------------------------------------
//Utility functions for parsing INI file
static bool parseRotationMatrix(const yarp::os::Value&, iDynTree::Rotation&);
static bool parsePositionVector(const yarp::os::Value&, iDynTree::Position&);
static bool parseStringListOption(const yarp::os::Value &option, std::vector<std::string> &parsedList);
//---------------------------------------------------------------------------

static inline TickTime normalizeSecNSec(double yarpTimeStamp)
{
    uint64_t time = static_cast<uint64_t>(yarpTimeStamp * 1000000000UL);
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


yarp::os::BufferedPort<human::HumanState>* HumanForcesProvider::getHumanStatePort(const yarp::os::Searchable& config)
{
    if (!m_humanConfigured) {

        yarp::os::Value falseValue;
        falseValue.fromString("false");
        bool autoconnect = config.check("autoconnect",
                                        falseValue,
                                        "Checking the autoconnect option").asBool();

        const std::string humanStateRemoteName = config.find("humanRemoteName").asString();

        if (!m_humanJointConfigurationPort.open("/" + getName() + "/humanState:i"))
        {
            yError("Unable to open the input port [%s]", ("/" + getName() + "/humanState:i").c_str());
            close();
            return 0;
        }
        if (autoconnect)
        {
            if (!yarp::os::Network::connect(humanStateRemoteName, m_humanJointConfigurationPort.getName()))  //to connect remote and local port
            {
                yError("Unable to connect port [%s] to [%s]", humanStateRemoteName.c_str(), m_humanJointConfigurationPort.getName().c_str());
                close();
                return 0;
            }
        }
        m_humanConfigured = true;
    }
    return &m_humanJointConfigurationPort;

}

bool HumanForcesProvider::getRobotEncodersInterface(const yarp::os::Searchable& config,
                                                    const std::vector<std::string>& robot_jointList,
                                                    yarp::dev::IEncoders *& encoders)
{
    if (!m_robotConfigured) {
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

        m_robotConfigured = true;
    }

    if (!m_robot.view(encoders) || !encoders)
    {
        yError("ControlBoard does not support IEncoders interface");
        close();
        return false;
    }
    return true;
}



HumanForcesProvider::HumanForcesProvider()
: m_period(0.1)
, m_humanConfigured(false)
, m_robotConfigured(false)
, m_rosForcesScale(1) {}

//---------------------------------------------------------------------
HumanForcesProvider::~HumanForcesProvider() {}

//---------------------------------------------------------------------
double HumanForcesProvider::getPeriod()
{
    return m_period;
}

//---------------------------------------------------------------------
bool HumanForcesProvider::configure(yarp::os::ResourceFinder &rf)
{
    m_humanConfigured = false;
    m_robotConfigured = false;

    /*
     * ------Configure module name
     */
    const std::string moduleName = rf.check("name",
                                            yarp::os::Value("human-forces-provider"),
                                            "Checking module name").asString();
    setName(moduleName.c_str());
    
    /*
     * ------Configure module periodicity
     */
    int periodInMs = rf.check("period",
                              yarp::os::Value(10),
                              "Checking period in [ms]").asInt();
    m_period = periodInMs / 1000.0;
    
    /*
     * ------Models loading
     */
    
    // Human model
    const std::string humanModelFilename = rf.findFile("humanModelFilename");
    
    std::vector<std::string> human_jointList;
    yarp::os::Value jointListValue = rf.find("human_JointList");
    if (jointListValue.isString()) {
        std::string configJointFile = rf.findFileByName(jointListValue.asString());
        yarp::os::Property config;
        if (config.fromConfigFile(configJointFile, true)) {
            if (!parseStringListOption(config.find("jointList"), human_jointList)) {
                yError() << "Error while parsing joints list";
                return false;
            }
        } else {
            yError() << "Could not parse " << configJointFile;
            return false;
        }
    }
    else if (jointListValue.isList()) {
        if (!parseStringListOption(jointListValue, human_jointList)) {
            yError() << "Error while parsing joints list";
            return false;
        }
    } else {
        yError() << "\"jointList\" parameter malformed";
        return false;
    }
    
    yInfo() << "Joints from config file: " << human_jointList.size() << human_jointList;
        
    iDynTree::ModelLoader modelLoader;
    
    if(!modelLoader.loadReducedModelFromFile(humanModelFilename, human_jointList))
    {
        yError("Something wrong with the human model loading!");
        return false;
    }
    
    iDynTree::Model humanModel = modelLoader.model();

    //Robot model
    const std::string robotModelFilename = rf.findFile("robotModelFilename");
    std::vector<std::string> robot_jointList;
    
    if (!parseStringListOption(rf.find("robot_JointList"), robot_jointList))
    {
        yError("Error while parsing the robot joint list");
        return false;
    }
    
    if(!modelLoader.loadReducedModelFromFile(robotModelFilename, robot_jointList))
    {
        yError("Something wrong with the robot model loading!");
        return false;
    }
    iDynTree::Model robotModel = modelLoader.model();

    yarp::os::Value falseValue;
    falseValue.fromString("false");
    bool autoconnect = rf.check("autoconnect",
                                falseValue,
                                "Checking the autoconnect option").asBool();

    if (rf.check("rosTopic", falseValue, "Checking support for ROS topics").asBool())
    {
        m_rosNode = new yarp::os::Node("/" + getName());
        if (!m_rosNode)
        {
            yError("Could not create ROS node %s", ("/" + getName()).c_str());
            close();
            return false;
        }

        m_tfPrefix = rf.check("rosTFPrefix", yarp::os::Value(""), "Checking TF prefix").asString();
	
	m_rosForcesScale = rf.check("rosForceScale", yarp::os::Value(1), "Checking output scale").asDouble();
    }

    /*
     * ------Reading the specified sources (in file .ini)
     */
    std::vector<std::string> sources;
    if (!parseStringListOption(rf.find("sources"), sources))
    {
        yWarning("No source specified.  This module will do nothing.");
    }

    if (m_rosNode)
    {
        m_topics.resize(sources.size());
        m_rosSequence = 1;
    }

    for(std::vector<std::string>::const_iterator it = sources.begin(); it != sources.end(); ++it)
    {
        yarp::os::Bottle sourceGroup = rf.findGroup("FORCE_SOURCE_" + *it);
        if (sourceGroup.isNull())
        {
            yWarning("Could not find group %s",("FORCE_SOURCE_" + *it).c_str());
            continue;
        }
        std::string type = sourceGroup.find("type").asString();
        if (type.empty())
        {
            yWarning("Could not find \"type\" parameter for group %s",("FORCE_SOURCE_" + *it).c_str());
            continue;
        }
        std::string remote = sourceGroup.find("remote").asString();
        if (remote.empty())
        {
            yWarning("Could not find \"remote\" parameter for group %s",("FORCE_SOURCE_" + *it).c_str());
            continue;
        }
        std::string appliedLink = sourceGroup.find("appliedLink").asString();
        if (appliedLink.empty())
        {
            yWarning("Could not find \"appliedLink\" parameter for group %s",("FORCE_SOURCE_" + *it).c_str());
            continue;
        }
        std::string inputFrame = sourceGroup.find("inputFrame").asString();
        if (inputFrame.empty())
        {
            yWarning("Could not find \"inputFrame\" parameter for group %s",("FORCE_SOURCE_" + *it).c_str());
            continue;
        }
        
        human::AbstractForceReader *forceReader = 0;
        
        if (type == "analogsensor")
        {
            yarp::os::Property options;
            options.put("device", "analogsensorclient");
            options.put("local", "/" + getName() + "/source_" + *it );  //local port name
            options.put("remote", remote);                              //device port name where we connect to
            
            yarp::dev::PolyDriver *driver = new yarp::dev::PolyDriver();
            if (!driver)
            {
                yError("Error in creating analogserver device [%s]", remote.c_str());
                close();
                return false;
            }
            m_drivers.push_back(driver);
            
            if (!driver->open(options))
            {
                yError("Error in opening analogserver device [%s]", remote.c_str());
                close();
                return false;
            }
            
            yarp::dev::IAnalogSensor *analogSensor = 0;
            if (!driver->view(analogSensor) || !analogSensor)
            {
                yError("Device [%s] does not support IAnalogSensor interface", remote.c_str());
                close();
                return false;
            }
            
            forceReader = new human::FTForceReader(appliedLink,
                                                   inputFrame,
                                                   *analogSensor);
        }
        else if (type == "port")
        {
            yarp::os::BufferedPort<yarp::sig::Vector> *port = new yarp::os::BufferedPort<yarp::sig::Vector>;
            if(!port)
            {
                yError("Unable to create input port [%s]", ("/" + getName() + "/source_" + *it + "/force:i").c_str());
                close();
                return false;
            }
            m_ports.push_back(port);
            
            if (!port->open("/" + getName() + "/source_" + *it + "/force:i"))
            {
                yError("Unable to open input port [%s]", ("/" + getName() + "/source_" + *it + "/force:i").c_str());
                close();
                return false;
            }
            if (autoconnect)
            {
                if (!yarp::os::Network::connect(remote, port->getName()))  //to connect remote and local port
                {
                    yError("Unable to connect port [%s] to [%s]", remote.c_str(), port->getName().c_str());
                    close();
                    return false;
                }
            }
            
            forceReader = new human::PortForceReader(appliedLink,
                                                     inputFrame,
                                                     *port);

        }

        if (!forceReader)
        {
            yError("Cannot create source of type [%s]", type.c_str());
            close();
            return false;
        }
        m_readers.push_back(forceReader);

        if (m_rosNode)
        {
            //create also topic
            std::vector<std::string>::difference_type index = std::distance<std::vector<std::string>::const_iterator>(sources.begin(), it);
            if (index >= 0) {
                m_topics[static_cast<size_t>(index)] = new yarp::os::Publisher<geometry_msgs::WrenchStamped>();
                if (m_topics[static_cast<size_t>(index)]) {
                    std::string ROSname = getName();
                    while(ROSname.find("-")!=std::string::npos) {
                        ROSname.replace(ROSname.find("-"),1,"_");
                    }
                    m_topics[static_cast<size_t>(index)]->topic("/" + ROSname + "/" + appliedLink);
                }
            }

        }
        
        yarp::os::Bottle transformationGroup = rf.findGroup("TRANSFORMATION_" + *it);
        if (transformationGroup.isNull())
        {
            yWarning("Could not find group %s",("TRANSFORMATION_" + *it).c_str());
            continue;
        }
        std::string transformType = transformationGroup.find("type").asString();
        if (transformType.empty())
        {
            yWarning("Could not find \"type\" parameter for group %s",("TRANSFORMATION_" + *it).c_str());
            continue;
        }
        std::string outputFrame = transformationGroup.find("outputFrame").asString();
        if (outputFrame.empty())
        {
            yWarning("Could not find \"outputFrame\" parameter for group %s",("TRANSFORMATION_" + *it).c_str());
            continue;
        }
        iDynTree::Rotation rotationMatrix;
        if (!parseRotationMatrix(transformationGroup.find("rotationMatrix"), rotationMatrix))
        {
            yError("Error parsing the rotation matrix in group %s", ("TRANSFORMATION_" + *it).c_str());
            continue;
        }
        iDynTree::Position originPosition;
        if (!parsePositionVector(transformationGroup.find("originPosition"), originPosition))
        {
            yError("Error parsing the origin position in group %s", ("TRANSFORMATION_" + *it).c_str());
            continue;
        }
        iDynTree::Position humanFootPosition;
        humanFootPosition.zero();
        if (transformationGroup.check("humanFootPosition", "Checking optional parameter humanFootPosition")) {
            if (!parsePositionVector(transformationGroup.find("humanFootPosition"), humanFootPosition))
            {
                yError("Error parsing the  human foot position in group %s", ("TRANSFORMATION_" + *it).c_str());
                continue;
            }
        }
        
        human::FrameTransformer *frameTransform = 0;
        
        if (transformType == "constant")
        {
            human::GenericFrameTransformer *genericFrameTransform = new human::GenericFrameTransformer(inputFrame,
                                                                                                       outputFrame);
            if (!genericFrameTransform)
            {
                close();
                return false;
            }
            genericFrameTransform->setTransform(iDynTree::Transform(rotationMatrix, originPosition));
            frameTransform = genericFrameTransform;
        }
        else if (transformType == "robot")
        {
            std::string humanLinkingFrame = transformationGroup.find("humanLinkingFrame").asString();
            if (humanLinkingFrame.empty())
            {
                yWarning("Could not find \"humanLinkingFrame\" parameter for group %s",("TRANSFORMATION_" + *it).c_str());
                continue;
            }
            std::string robotLinkingFrame = transformationGroup.find("robotLinkingFrame").asString();
            if (robotLinkingFrame.empty())
            {
                yWarning("Could not find \"robotLinkingFrame\" parameter for group %s",("TRANSFORMATION_" + *it).c_str());
                continue;
            }
            
            iDynTree::Transform l_sole_H_LeftSole = iDynTree::Transform(rotationMatrix, originPosition);
            iDynTree::Transform LeftSole_H_LeftFoot = iDynTree::Transform(iDynTree::Rotation::Identity(), humanFootPosition);
            iDynTree::Transform fixtureTransform = l_sole_H_LeftSole * LeftSole_H_LeftFoot;

            yarp::os::BufferedPort<human::HumanState>* humanPort = getHumanStatePort(rf);
            if (!humanPort)
            {
                yError("Failed to obtain connection to human state port");
                return false;
            }

            yarp::dev::IEncoders *robotEncoders = 0;

            if (!getRobotEncodersInterface(rf, robot_jointList, robotEncoders) || !robotEncoders)
            {
                yError("Failed to open robot IEncoders interface");
                return false;
            }

            human::RobotFrameTransformer *robotFrameTransform = new human::RobotFrameTransformer(fixtureTransform,
                                                                                                 inputFrame,
                                                                                                 robotLinkingFrame,
                                                                                                 humanLinkingFrame,
                                                                                                 outputFrame,
                                                                                                 *robotEncoders,
                                                                                                 *humanPort);
            if (!robotFrameTransform)
            {
                close();
                return false;
            }
            
            robotFrameTransform->init(humanModel, robotModel);
            frameTransform = robotFrameTransform;
        }
    
        forceReader->setTransformer(frameTransform);
    }
    
    /*
     * ------Open port:o for the module output (to the next module human-dynamics-estimation)
     */
    if (!m_outputPort.open("/" + getName() + "/forces:o"))
    {
        yError("Unable to open port %s", ("/" + getName() + "/forces:o").c_str());
        return false;
    }
    
    human::HumanForces &allForces = m_outputPort.prepare();
    allForces.forces.reserve(m_readers.size());
    m_outputPort.unprepare();
    yInfo("%s properly configured", getName().c_str());
    return true;
}

//---------------------------------------------------------------------
bool HumanForcesProvider::updateModule()
{
    human::HumanForces &allForces = m_outputPort.prepare();
    std::vector<human::Force6D> &forcesVector = allForces.forces;
    
    TickTime rosTime = normalizeSecNSec(yarp::os::Time::now());
    
    forcesVector.resize(m_readers.size());
    for (size_t index = 0; index < m_readers.size(); ++index) {
        human::Force6D &currentForce = forcesVector[index];
        m_readers[index]->readForce(currentForce);

        if (m_rosNode && m_topics[index])
        {
            geometry_msgs::WrenchStamped &wrench = m_topics[index]->prepare();

            wrench.header.seq = m_rosSequence;
            wrench.header.stamp = rosTime;
            wrench.header.frame_id = m_tfPrefix + currentForce.appliedLink;

            wrench.wrench.force.x = m_rosForcesScale * currentForce.fx;
            wrench.wrench.force.y = m_rosForcesScale * currentForce.fy;
            wrench.wrench.force.z = m_rosForcesScale * currentForce.fz;
            wrench.wrench.torque.x = m_rosForcesScale * currentForce.ux;
            wrench.wrench.torque.y = m_rosForcesScale * currentForce.uy;
            wrench.wrench.torque.z = m_rosForcesScale * currentForce.uz;

            m_topics[index]->write();
        }
    }

    m_rosSequence++;
    /*
     * ------Write data on port:o
     */
    m_outputPort.write();
    return true;
}

//---------------------------------------------------------------------
bool HumanForcesProvider::close()
{
    /*
     * ------Releasing allocated memory for m_readers
     */
    for (std::vector<human::ForceReader*>::iterator it = m_readers.begin();
    it != m_readers.end(); ++it)
    {
        human::AbstractForceReader *AbstractForceReader = dynamic_cast<human::AbstractForceReader*>(*it);
        if (AbstractForceReader)
        {
            delete AbstractForceReader->getTransformer();
        }
        delete *it;
    }
    m_readers.clear();
    
    /*
     * ------Close the m_outputPort
     */
    m_outputPort.close();
    
    /*
     * ------Releasing allocated memory for m_ports
     */
    for(std::vector<yarp::os::BufferedPort<yarp::sig::Vector>*>::const_iterator it = m_ports.begin();
        it != m_ports.end(); ++it)
    {
        if (!(*it)) {
            continue;
        }
        (*it)->close();
        delete *it;
    }
    m_ports.clear();

    /*
     * ------Releasing allocated memory for m_topics
     */
    for (std::vector<yarp::os::Publisher<geometry_msgs::WrenchStamped>*>::iterator it(m_topics.begin());
         it != m_topics.end(); ++it)
    {
        if (!(*it)) {
            continue;
        }
        (*it)->close();
        delete *it;
    }
    m_topics.clear();
    
    /*
     * ------Close the m_humanJointConfigurationPort
     */
    m_humanJointConfigurationPort.close();
    
    /*
     * ------Close the m_robot port
     */
    m_robot.close();
    
    /*
     * ------Releasing allocated memory for m_drivers
     */
    for(std::vector<yarp::dev::PolyDriver*>::const_iterator it = m_drivers.begin();
        it != m_drivers.end(); ++it)
    {
        (*it)->close();
        delete *it;
    }
    m_drivers.clear();

    delete m_rosNode;
    m_rosNode = 0;

    return true;
}

//---------------------------------------------------------------------------
/*
 * Implementation of the utility functions.
 */
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
