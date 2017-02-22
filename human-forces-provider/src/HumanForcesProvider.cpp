
//
//  HumanForcesProvider.cpp
//  HumanForcesProvider
//
//  Created by Claudia Latella on 14/02/17.
//  Copyright © 2017 Claudia Latella. All rights reserved.

#include "HumanForcesProvider.h"
#include "ForceReader.h"
#include "FTForceReader.h"
#include "PortForceReader.h"

#include <yarp/os/LogStream.h>  //for using yError()
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IEncoders.h>
#include <iostream>


HumanForcesProvider::HumanForcesProvider()
: m_period(0.1) {}

HumanForcesProvider::~HumanForcesProvider() {}

//---------------------------------------------------------------------
double HumanForcesProvider::getPeriod()
{
    return m_period;
}

//---------------------------------------------------------------------
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
     * ------Configure module periodicty
     */
    int periodInMs = rf.check("period",
                              yarp::os::Value(100),
                              "Checking period in [ms]").asInt();
    m_period = periodInMs / 1000.0;
    
    
    /* HANDLING OF THE FORCEPLATES*********************************************************/
    // The two forceplates are handled with a polydriver that is configured as follows:
     
    /*
     * ------Configure force plates devices
     */
    std::string appliedLink_fp1 = rf.find("appliedLink_fp1").asString();
    std::string expressedFrame_fp1 = rf.find("expressedFrame_fp1").asString();
    
    std::string ft1LocalName = rf.check("ft1_localName",
                                        yarp::os::Value("/" + getName() + "/FTSensor1:i"),
                                        "Checking FP1 port local name").asString();
    std::string ft1RemoteName = rf.check("ft1_remoteName",
                                         yarp::os::Value("/amti/first/analog:o"),
                                         "Checking FP1 port remote name").asString();
    
    
    std::string appliedLink_fp2 = rf.find("appliedLink_fp2").asString();
    std::string expressedFrame_fp2 = rf.find("expressedFrame_fp2").asString();
    
    std::string ft2LocalName = rf.check("ft2_localName",
                                        yarp::os::Value("/" + getName() + "/FTSensor2:i"),
                                        "Checking FP2 port local name").asString();
    std::string ft2RemoteName = rf.check("ft2_remoteName",
                                         yarp::os::Value("/amti/second/analog:o"),
                                         "Checking FP2 port remote name").asString();
    
    
    /*
     * ------Configure and open the polydriver [in AnalogSensorClient.cpp of github/YARP]
     */
    yarp::os::Property options_FP;
    options_FP.put("device", "analogsensorclient");
    
    // -----------------------SENSOR 1 : FP1-----------------------//
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
    
    // TODO: Tranforming forceplate1 forces in human forces --> by using FrameTransformer interface
    human::FTForceReader *forceReader1 = new human::FTForceReader(appliedLink_fp1,
                                                                  expressedFrame_fp1,
                                                                  *firstSensor);
    m_readers.push_back(forceReader1);
    
    // -----------------------SENSOR 2 : FP2-----------------------//
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
    
    // TODO: Tranforming forceplate2 forces in human forces --> by using FrameTransformer interface
    human::FTForceReader *forceReader2 = new human::FTForceReader(appliedLink_fp2,
                                                                  expressedFrame_fp2,
                                                                  *secondSensor);
    m_readers.push_back(forceReader2);
    
    
    /* HANDLING OF THE ROBOT*********************************************************/
    /* From the robot we need:
     *  1. the configuration of the joints of interest --> handled by a
     *     RemoteControlBoardRemapper device
     *  2. the estimated forces acquired by the two FTS in the robot arms
     */
    
    /*
     * 1. Robot configuration
     */
    yarp::os::Property options_robot;
    options_robot.put("device", "remotecontrolboardremapper");
    yarp::os::Bottle axesNames;
    yarp::os::Bottle & axesList = axesNames.addList();
    yarp::os::Bottle * axesNameList = rf.find("axesNames").asList();
    if (!axesNameList) {
        //TODO: check on the order of joints == same as the one in the iCub urdf!!
    }
    axesList = *axesNameList;
//    axesList.addString("torso_pitch");
//    axesList.addString("torso_roll");
//    axesList.addString("torso_yaw");
//    axesList.addString("leftLeg_pitch");
//    axesList.addString("leftLeg_roll");
//    axesList.addString("leftLeg_yaw");
//    axesList.addString("rightLeg_pitch");
//    axesList.addString("rightLeg_roll");
//    axesList.addString("rightLeg_yaw");
//    axesList.addString("leftArm_pitch");
//    axesList.addString("leftArm_roll");
//    axesList.addString("leftArm_yaw");
//    axesList.addString("rightArm_pitch");
//    axesList.addString("rightArm_roll");
//    axesList.addString("rightArm_yaw");
    options_robot.put("axesNames", axesNames.get(0));
    
    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
    remoteControlBoardsList.addString("/icub/torso");
    remoteControlBoardsList.addString("/icub/leftLeg");
    remoteControlBoardsList.addString("/icub/rightLeg");
    remoteControlBoardsList.addString("/icub/leftArm");
    remoteControlBoardsList.addString("/icub/rightArm");
    options_robot.put("remoteControlBoards", remoteControlBoards.get(0));
    options_robot.put("localPortPrefix", "/" + getName() + "/robot");
    
    // Open the polydriver
    ok = m_PolyRobot.open(options_robot);
    if (!ok)
    {
        yError("Error in opening remoteControlBoards device for the robot!");
        close();
        return false;
    }
    
    yarp::dev::IEncoders *robotJointConfig = 0;
    if (!m_PolyRobot.view(robotJointConfig) || !robotJointConfig)
    {
        yError("Error in viewing IEncoders!");
        close();
        return false;
    }
    
    //TODO: use robotJointConfig data for transforming robot forces in human forces
    //       --> by using FrameTransformer interface
    
    /*
     * 2. robot arms forces estimation
     */
    
    if (!m_robotLeftArmForceEstimation.open("/"+ getName() + "/robotLeftForces:i"))
    {
        yError() << "Unable to open port " << (getName() + "/robotLeftForces:i");
        return false;
    }
    
    
    
    if (!m_robotRightArmForceEstimation.open("/"+ getName() + "/robotRightForces:i"))
    {
        yError() << "Unable to open port " << (getName() + "/robotRightForces:i");
        return false;
    }
    
    // // -----------------------SENSOR ON ROBOT RIGHT ARM -----------------------//
    // std::string humanContactLink_rightArmRobot = rf.find("humanContactLink_rightArmRobot").asString();
    // std::string humanContactFrame_rightArmRobot = rf.find("humanContactFrame_rightArmRobot").asString();
    // yarp::dev::IAnalogSensor *rightArmRobotSensor = 0;
    //
    // human::PortForceReader *forceReader3 = new human::PortForceReader("humanContactLink_rightArmRobot",
    //                                                                     "humanContactFrame_rightArmRobot",
    //                                                                     *rightArmRobotSensor);
    // m_readers.push_back(forceReader3);
    //
    // // -----------------------SENSOR ON ROBOT LEFT ARM -----------------------//
    // std::string humanContactLink_leftArmRobot = rf.find("humanContactLink_leftArmRobot").asString();
    // std::string humanContactFrame_leftArmRobot = rf.find("humanContactFrame_leftArmRobot").asString();
    // yarp::dev::IAnalogSensor *leftArmRobotSensor = 0;
    //
    // human::PortForceReader *forceReader4 = new human::PortForceReader("humanContactLink_leftArmRobot",
    //                                                                     "humanContactFrame_leftArmRobot",
    //                                                                     *leftArmRobotSensor);
    // m_readers.push_back(forceReader4);

    /* *****************************************************************************/
    
    /*
     * ------Open port:o for the module output (to the next module human-dynamics-estimation)
     */
    if (!m_outputPort.open("/"+ getName() + "/forces:o"))
    {
        yError() << "Unable to open port " << (getName() + "/forces:o");
        return false;
    }
    
    human::HumanForces &allForces = m_outputPort.prepare();
    allForces.forces.reserve(2);
    m_outputPort.unprepare();
    return true;
}

//---------------------------------------------------------------------
bool HumanForcesProvider::updateModule()
{
    human::HumanForces &allForces = m_outputPort.prepare();
    std::vector<human::Force6D> &forcesVector = allForces.forces;
    
    forcesVector.resize(m_readers.size());
    for (std::vector<human::ForceReader*>::iterator it(m_readers.begin());
         it != m_readers.end(); ++it) {
        unsigned index = std::distance(m_readers.begin(), it);
        (*it)->readForce(forcesVector[index]);
    }
    
    
    /*
     * ------Write data on port:o
     */
    m_outputPort.write();
    return true;
}

//---------------------------------------------------------------------
/*
 * Close function, to perform cleanup.
 */
bool HumanForcesProvider::close()
{
    m_outputPort.close();
    m_robotRightArmForceEstimation.close();
    m_robotLeftArmForceEstimation.close();
    
    
    for (std::vector<human::ForceReader*>::iterator it(m_readers.begin());
         it != m_readers.end(); ++it) {
        delete *it;
    }
    m_readers.clear();
    m_PolyRobot.close();
    m_forcePoly2.close();
    m_forcePoly1.close();
    
    return true;
}
