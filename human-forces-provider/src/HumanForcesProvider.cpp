/*!
 * @file ForceReader.h
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
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/api.h> //for IEncoders binaries
#include <yarp/dev/IEncoders.h>

#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <iostream>
#include <cassert>


//---------------------------------------------------------------------------
//Utility functions for parsing INI file
static bool parseRotationMatrix(const yarp::os::Value&, iDynTree::Rotation&);
static bool parsePositionVector(const yarp::os::Value&, iDynTree::Position&);
//---------------------------------------------------------------------------


HumanForcesProvider::HumanForcesProvider()
: m_period(0.1){}


HumanForcesProvider::~HumanForcesProvider() {}


double HumanForcesProvider::getPeriod()
{
    return m_period;
}


bool HumanForcesProvider::configure(yarp::os::ResourceFinder &rf)
{
    /*
     * ------Configure module name
     */
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("humanForceProvider"),
                                      "Checking module name").asString();
    setName(moduleName.c_str());
    
    /*
     * ------Configure module periodicity
     */
    int periodInMs = rf.check("period",
                              yarp::os::Value(100),
                              "Checking period in [ms]").asInt();
    m_period = periodInMs / 1000.0;
    
    /*
     * ------Open a port for the human joint configuration
     */
    if (!m_humanJointConfiguration_port.open("/humanJointConfiguration:i"))
    {
        yError() << "Unable to open port /humanJointConfiguration:i";
        return false;
    }
    
    
   /* **************** NOTE ON THE FORCE PLATES ********************************
    * The configuration of the force plates consists into 2 parts:
    * 1 -> INI configuration for the polydriver (since they are handled with a 
    *      polydriver, documented in AnalogSensorClient.cpp of github/YARP) ;
    * 2 -> INI configuration with the utility functions for the frame 
    *      transformation.
    * **************************************************************************/
    
    /*
     * ------Configure force plate 1 (FP1)
     */
    std::string appliedLink_fp1  = rf.find("appliedLink_fp1").asString();
    std::string inputFrame_fp1   = rf.find("inputFrame_fp1").asString();
    std::string outputFrame_fp1  = rf.find("outputFrame_fp1").asString();

    std::string ft1LocalName  = rf.check("ft1_localName",
                                        yarp::os::Value("/" + getName() + "/FTSensor1:i"),
                                        "Checking FP1 port local name").asString();
    std::string ft1RemoteName = rf.check("ft1_remoteName",
                                         yarp::os::Value("/amti/first/analog:o"),
                                         "Checking FP1 port remote name").asString();
    
    iDynTree::Rotation rotationMatrix_fp1;
    iDynTree::Position footPosition_fp1;
    iDynTree::Position solePosition_fp1;
    iDynTree::Position calibPosition_fp1;
    iDynTree::Position position_fp1;
    
    if (!parseRotationMatrix(rf.find("rotationMatrix_fp1"), rotationMatrix_fp1))
    {
        yError("Somenthing wrong in parsing the rotation matrix!");
        return false;
    }
    
    if (!parsePositionVector(rf.find("footPosition_fp1"), footPosition_fp1))
    {
        yError("Somenthing wrong in parsing the foot position vector!");
        return false;
    }
    
    if (!parsePositionVector(rf.find("solePosition_fp1"), solePosition_fp1))
    {
        yError("Somenthing wrong in parsing the sole position vector!");
        return false;
    }
    
    if (!parsePositionVector(rf.find("calibPosition_fp1"), calibPosition_fp1))
    {
        yError("Somenthing wrong in parsing the calibration position vector!");
        return false;
    }
    
    position_fp1 = footPosition_fp1 + solePosition_fp1 + calibPosition_fp1;
    iDynTree::Transform transform_fp1(rotationMatrix_fp1, position_fp1);
    
    /*
     * ------Configure force plate 2 (FP2)
     */
    std::string appliedLink_fp2  = rf.find("appliedLink_fp2").asString();
    std::string inputFrame_fp2   = rf.find("inputFrame_fp2").asString();
    std::string outputFrame_fp2  = rf.find("outputFrame_fp2").asString();
   
    std::string ft2LocalName  = rf.check("ft2_localName",
                                        yarp::os::Value("/" + getName() + "/FTSensor2:i"),
                                        "Checking FP2 port local name").asString();
    std::string ft2RemoteName = rf.check("ft2_remoteName",
                                         yarp::os::Value("/amti/second/analog:o"),
                                         "Checking FP2 port remote name").asString();
    
    iDynTree::Rotation rotationMatrix_fp2;
    iDynTree::Position footPosition_fp2;
    iDynTree::Position solePosition_fp2;
    iDynTree::Position calibPosition_fp2;
    iDynTree::Position position_fp2;
    
    if (!parseRotationMatrix(rf.find("rotationMatrix_fp2"), rotationMatrix_fp2))
    {
        yError("Somenthing wrong in parsing the rotation matrix!");
        return false;
    }
    
    if (!parsePositionVector(rf.find("footPosition_fp2"), footPosition_fp2))
    {
        yError("Somenthing wrong in parsing the foot position vector!");
        return false;
    }
    
    if (!parsePositionVector(rf.find("solePosition_fp2"), solePosition_fp2))
    {
        yError("Somenthing wrong in parsing the sole position vector!");
        return false;
    }
    
    if (!parsePositionVector(rf.find("calibPosition_fp2"), calibPosition_fp2))
    {
        yError("Somenthing wrong in parsing the calibration position vector!");
        return false;
    }
    
    position_fp2 = footPosition_fp2 + solePosition_fp2 + calibPosition_fp2;
    iDynTree::Transform transform_fp2(rotationMatrix_fp2, position_fp2 );

    
    /*
     * ------Configure and open the polidriver
     */
    yarp::os::Property options_FP;
    options_FP.put("device", "analogsensorclient");
    
    //-----------------------SENSOR 1 : FP1 -------------------------------------//
    
    options_FP.put("local", ft1LocalName);          //local port name
    options_FP.put("remote", ft1RemoteName);        //device port name where we connect to
    
    bool ok = m_forcePoly1.open(options_FP);
    if (!ok)
    {
        yError("Error in opening FT1 Sensor device!");
        close();
        return false;
    }

    yarp::dev::IAnalogSensor *firstSensor = 0;
    if (!m_forcePoly1.view(firstSensor) || !firstSensor)
    {
        yError("Error in viewing IAnalogSensor1!");
        close();
        return false;
    }
    
    human::GenericFrameTransformer *frameTransform_fp1 = new human::GenericFrameTransformer(inputFrame_fp1,
                                                                                            outputFrame_fp1);
    frameTransform_fp1->setTransform(transform_fp1);
    
    human::FTForceReader *forceReader1 = new human::FTForceReader(appliedLink_fp1,
                                                                  inputFrame_fp1,
                                                                  *firstSensor);
    forceReader1->setTransformer(frameTransform_fp1);
    m_readers.push_back(forceReader1);
    
    
    //-----------------------SENSOR 2 : FP2 -------------------------------------//
    
    options_FP.put("local" , ft2LocalName);          //local port name
    options_FP.put("remote", ft2RemoteName);         //device port name where we connect to
    
    ok = m_forcePoly2.open(options_FP);
    if (!ok)
    {
        yError("Error in opening FT2 Sensor device!");
        close();
        return false;
    }
    
    yarp::dev::IAnalogSensor *secondSensor = 0;
    if (!m_forcePoly2.view(secondSensor) || !secondSensor)
    {
        yError("Error in viewing IAnalogSensor2!");
        close();
        return false;
    }
    
    human::GenericFrameTransformer *frameTransform_fp2 = new human::GenericFrameTransformer(inputFrame_fp2,
                                                                                            outputFrame_fp2);
    frameTransform_fp2->setTransform(transform_fp2);
    
    human::FTForceReader *forceReader2 = new human::FTForceReader(appliedLink_fp2,
                                                                  inputFrame_fp2,
                                                                  *secondSensor);
    forceReader2->setTransformer(frameTransform_fp2);
    m_readers.push_back(forceReader2);
    
    
    /* ******************* NOTE ON THE ROBOT ***********************************
     * From the robot we need:
     * 1 -> the configuration of the robot joints of interest handled by a
     *      RemoteControlBoardRemapper device (documented in 
     *      YARP/RemoteControlBoardRemapper);
     * 2 -> the configuration of the human
     * *************************************************************************/
    
    /*
     * ------Models loading
     */
    std::string humanModelFilename = rf.find("humanModelFilename").asString();
    std::string robotModelFilename = rf.find("robotModelFilename").asString();
    
    iDynTree::ModelLoader modelLoader;
    if(!modelLoader.loadModelFromFile(humanModelFilename))
    {
        yError("Something wrong with the human model loading!");
        return false;
    }
    iDynTree::Model humanModel = modelLoader.model();
    
    if(!modelLoader.loadModelFromFile(robotModelFilename))
    {
        yError("Something wrong with the robot model loading!");
        return false;
    }
    iDynTree::Model robotModel = modelLoader.model();
    
    /*
     *  ------Settings for the transforms
     */
    iDynTree::Rotation robotLeftSole_R_humanLeftSole;
    iDynTree::Position robotLeftSole_pos_humanLeftSole;
    
    if (!parseRotationMatrix(rf.find("robotLeftSole_R_humanLeftSole"), robotLeftSole_R_humanLeftSole))
    {
        yError("Somenthing wrong in parsing the rotation matrix between robotLeftSole and humanLeftSole!");
        return false;
    }
    if (!parsePositionVector(rf.find("robotLeftSole_pos_humanLeftSole"), robotLeftSole_pos_humanLeftSole))
    {
        yError("Somenthing wrong in parsing the position between robotLeftSole and humanLeftSole!");
        return false;
    }
    iDynTree::Transform robotLeftSole_T_humanLeftSole(robotLeftSole_R_humanLeftSole,
                                                      robotLeftSole_pos_humanLeftSole);
    
    //TODO: check on strings
    std::string frameRobotSole   = rf.find("frameRobotSole").asString();
    std::string frameHumanFoot   = rf.find("frameHumanFoot").asString();
    std::string frameRobotArm_left   = rf.find("frameRobotArm_left").asString();
    std::string frameRobotArm_right  = rf.find("frameRobotArm_right").asString();
    std::string frameHumanHand_left  = rf.find("frameHumanHand_left").asString();
    std::string frameHumanHand_right = rf.find("frameHumanHand_right").asString();
    
    /*
     * ------Robot configuration for the RemoteControlBoardRemapper
     */
    yarp::os::Property options_robot;
    options_robot.put("device", "remotecontrolboardremapper");
    yarp::os::Bottle axesNames;
    yarp::os::Bottle &axesList = axesNames.addList();
    yarp::os::Bottle *axesNameList = rf.find("axesNames").asList();
    
    if (!axesNameList)
    {
        //TODO: check on the order of joints == same as the one in the iCub urdf!!
    }
    
    axesList = *axesNameList;
    axesList.addString("torso_pitch");
    axesList.addString("torso_roll");
    axesList.addString("torso_yaw");
    axesList.addString("l_shoulder_pitch");
    axesList.addString("l_shoulder_roll");
    axesList.addString("l_shoulder_yaw");
    axesList.addString("l_elbow");
    axesList.addString("r_shoulder_pitch");
    axesList.addString("r_shoulder_roll");
    axesList.addString("r_shoulder_yaw");
    axesList.addString("r_elbow");
    axesList.addString("l_hip_picth");
    axesList.addString("l_hip_roll");
    axesList.addString("l_hip_yaw");
    axesList.addString("l_knee");
    axesList.addString("l_ankle_pitch");
    axesList.addString("l_ankle_roll");
    axesList.addString("r_hip_picth");
    axesList.addString("r_hip_roll");
    axesList.addString("r_hip_yaw");
    axesList.addString("r_knee");
    axesList.addString("r_ankle_pitch");
    axesList.addString("r_ankle_roll");
    options_robot.put("axesNames", axesNames.get(0));
    
    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
    remoteControlBoardsList.addString("/icub/torso");
    remoteControlBoardsList.addString("/icub/left_leg");
    remoteControlBoardsList.addString("/icub/right_leg");
    remoteControlBoardsList.addString("/icub/left_arm");
    remoteControlBoardsList.addString("/icub/right_arm");
    options_robot.put("remoteControlBoards", remoteControlBoards.get(0));
    options_robot.put("localPortPrefix", "/" + getName() + "/robot");
    
    
    // Open the polydriver for the joints configuration
    ok = m_PolyRobot.open(options_robot);
    if (!ok)
    {
        yError("Error in opening the device for the robot!");
        close();
        return false;
    }
    
    yarp::dev::IEncoders *robotEncoder = 0;
    if (!m_PolyRobot.view(robotEncoder) || !robotEncoder)
    {
        yError("Error in viewing robot sensors!");
        close();
        return false;
    }
    
    /*
     * ------Open ports for robot forces
     */
    if (!m_robotLeftArmForce_port.open("/robotLeftForces:i"))
    {
        yError() << "Unable to open port /robotLeftForces:i";
        return false;
    }
    
    if (!m_robotRightArmForce_port.open("/robotRightForces:i"))
    {
        yError() << "Unable to open port /robotRightForces:i";
        return false;
    }
    
    //-----------------------FTS SENSOR ON ROBOT RIGHT ARM -----------------------//

    std::string humanContactLinkWithRobotRightArm = rf.find("humanContactLinkWithRobotRightArm").asString();

    human::RobotFrameTransformer *frameTransform_robotRightArm = new human::RobotFrameTransformer(robotLeftSole_T_humanLeftSole,
                                                                                                 frameRobotArm_right,
                                                                                                 frameRobotSole,
                                                                                                 frameHumanFoot,
                                                                                                 frameHumanHand_left,
                                                                                                 *robotEncoder,
                                                                                                 m_humanJointConfiguration_port);
    frameTransform_robotRightArm->init(humanModel, robotModel);
    
    human::PortForceReader *forceReader3 = new human::PortForceReader(humanContactLinkWithRobotRightArm,
                                                                      frameRobotArm_right,
                                                                      m_robotRightArmForce_port);
    forceReader3->setTransformer(frameTransform_robotRightArm);
    m_readers.push_back(forceReader3);
    
    //-----------------------FTS SENSOR ON ROBOT LEFT ARM -----------------------//
    
    std::string humanContactLinkWithRobotLeftArm = rf.find("humanContactLinkWithRobotLeftArm").asString();
    
    human::RobotFrameTransformer *frameTransform_robotLeftArm = new human::RobotFrameTransformer(robotLeftSole_T_humanLeftSole,
                                                                                                 frameRobotArm_left,
                                                                                                 frameRobotSole,
                                                                                                 frameHumanFoot,
                                                                                                 frameHumanHand_right,
                                                                                                 *robotEncoder,
                                                                                                 m_humanJointConfiguration_port);
    frameTransform_robotLeftArm->init(humanModel, robotModel);
    
    human::PortForceReader *forceReader4 = new human::PortForceReader(humanContactLinkWithRobotLeftArm,
                                                                      frameRobotArm_left,
                                                                      m_robotRightArmForce_port);
    forceReader4->setTransformer(frameTransform_robotLeftArm);
    m_readers.push_back(forceReader4);


    /* *****************************************************************************/
    
    /*
     * ------Open port:o for the module output (to the next module human-dynamics-estimation)
     */
    if (!m_output_port.open("/"+ getName() + "/forces:o"))
    {
        yError() << "Unable to open port " << (getName() + "/forces:o");
        return false;
    }
    
    human::HumanForces &allForces = m_output_port.prepare();
    allForces.forces.reserve(2);
    m_output_port.unprepare();
    return true;
}

//---------------------------------------------------------------------
bool HumanForcesProvider::updateModule()
{
    human::HumanForces &allForces = m_output_port.prepare();
    std::vector<human::Force6D> &forcesVector = allForces.forces;
    
    forcesVector.resize(m_readers.size());
    for (std::vector<human::ForceReader*>::iterator it(m_readers.begin());
         it != m_readers.end(); ++it) {
        std::vector<human::ForceReader*>::difference_type index = std::distance(m_readers.begin(), it);
        assert(index >= 0);
        (*it)->readForce(forcesVector[static_cast<size_t>(index)]);
    }
    

    /*
     * ------Write data on port:o
     */
    m_output_port.write();
    return true;
}

//---------------------------------------------------------------------
bool HumanForcesProvider::close()
{
    //releasing allocated memory
    for (std::vector<human::ForceReader*>::iterator it(m_readers.begin());
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
    
    m_output_port.close();
    m_robotRightArmForce_port.close();
    m_robotLeftArmForce_port.close();
    m_humanJointConfiguration_port.close();
    
    
    m_PolyRobot.close();
    m_forcePoly2.close();
    m_forcePoly1.close();
    
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
