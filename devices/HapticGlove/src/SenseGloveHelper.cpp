/**
 * @file SenseGloveHelper.cpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <limits>
#include <cmath>

#include <SenseGloveHelper.hpp>

using namespace  senseGlove;


SenseGloveHelper::SenseGloveHelper(){
    yInfo()<<LogPrefix<<"SenseGloveHelper()";

    m_isReady = false;
    m_forceFbDof = 5;
    m_buzzDof = 5;
    m_handNoLinks = 20;
    m_gloveNoLinks = 30;
    m_NoSensors = 20;

    m_desiredBuzzValues.resize(m_buzzDof, 0);
    m_desiredForceValues.resize(m_forceFbDof, 0);
    m_glovePose = Eigen::MatrixXd::Zero(m_gloveNoLinks, 7);
    m_handPose = Eigen::MatrixXd::Zero(m_handNoLinks, 7);
    m_handJointsAngles = Eigen::MatrixXd::Zero(m_handNoLinks, 3);
}

bool SenseGloveHelper::configure(const yarp::os::Searchable& config)
{
    yInfo()<<LogPrefix<<"configure:: ";

    if (!(config.check("rightHand") && config.find("rightHand").isBool())) {
        yInfo() << LogPrefix << "Using default hand Sense Glove: Right hand";
        m_isRightHand = true;
    }
    else {
        m_isRightHand  = config.find("rightHand").asBool();
        yInfo() << LogPrefix << "Using the right hand: " << m_isRightHand <<"(if false, using left hand)";
    }

    // get human hand link name
    if (!(config.check("hand_link") && config.find("hand_link").isString()))
    {
        yError() << LogPrefix<<"Unable to find hand_link in the config file.";
        return false;
    }
    m_humanHandLinkName = config.find("hand_link").asString();
    yInfo() << LogPrefix << "human hand link name: " << m_humanHandLinkName;

    // get human hand joint names
    yarp::os::Bottle* jointListYarp;
    if (!(config.check("human_joint_list") && config.find("human_joint_list").isList()))
    {
        yError() << LogPrefix<<"Unable to find human_joint_list in the config file.";
        return false;
    }
    jointListYarp = config.find("human_joint_list").asList();

    for(size_t i=0; i< jointListYarp->size(); i++)
    {
        m_humanJointNameList.push_back(jointListYarp->get(i).asString());
    }
    yInfo()<<LogPrefix<<"human joint names: "<<m_humanJointNameList;

    // get human hand finger names
    yarp::os::Bottle* fingerListYarp;
    if (!(config.check("human_finger_list") && config.find("human_finger_list").isList()))
    {
        yError() << LogPrefix<<"Unable to find human_finger_list in the config file.";
        return false;
    }
    fingerListYarp = config.find("human_finger_list").asList();

    for(size_t i=0; i< fingerListYarp->size(); i++)
    {
        m_humanFingerNameList.push_back(fingerListYarp->get(i).asString());
    }
    yInfo()<<LogPrefix<<"human finger names: "<<m_humanFingerNameList;

    if(!setupGlove())
    {
        yError() << LogPrefix << "Unable to set up the sense glove.";
        return false;
    }

    return true;
}

bool SenseGloveHelper::setupGlove()
{
    yInfo() << LogPrefix << "setupGlove()";

    if (!SGCore::DeviceList::SenseCommRunning()) // Returns true if SenseComm is running.
    {
        yError() <<  LogPrefix << "SenseComm is not running. Please run SenseComm, then try again.";
        return false;
    }

    if (!SGCore::SG::SenseGlove::GetSenseGlove(m_isRightHand, m_glove))
    {
        yError() << LogPrefix << "No sense gloves connected to the system. Ensure the USB connection is "
                    "secure, then try again.";
        return false;
    }

    SGCore::SG::SG_GloveInfo gloveModel = m_glove.GetGloveModel();
    yInfo() << LogPrefix << "glove model:" << gloveModel.ToString(true);

    return true;
}

bool SenseGloveHelper::setFingersForceReference(const std::vector<double> & desiredValue)
{
    if (desiredValue.size() != m_forceFbDof)
    {
        yError() << LogPrefix <<"Size of the input desired vecotr and the number of haptic force feedbacks are not equal.";
        return false;
    }

    for (size_t i = 0; i < m_forceFbDof; i++)
    {
            m_desiredForceValues[i] = (int)std::round(std::max(0.0,std::min(desiredValue[i], 40.0))*100/40);
    }

    m_glove.SendHaptics(SGCore::Haptics::SG_FFBCmd(m_desiredForceValues));

    return true;
}


bool SenseGloveHelper::setBuzzMotorsReference(const std::vector<double> & desiredValue)
{
    if (desiredValue.size() != m_buzzDof)
    {
        yError() << LogPrefix << "Size of the input desired vector and the number of buzz motors are not equal.";
        return false;
    }
    for (size_t i = 0; i < m_buzzDof; i++)
    {
            m_desiredBuzzValues[i] = (int)std::round(std::max(0.0,std::min(desiredValue[i], 100.0)));
    }
    m_glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(m_desiredBuzzValues));

    return true;
}

bool SenseGloveHelper::setPalmFeedbackThumper(const int desiredValue)
{
    // to check: better develop and invetigate different options
    if (desiredValue == 0)
        return m_glove.SendHaptics(SGCore::Haptics::Impact_Thump_100);
    else if (desiredValue == 1)
        return m_glove.SendHaptics(SGCore::Haptics::Object_Grasp_100);
    else
        return m_glove.SendHaptics(SGCore::Haptics::Button_Double_100);
}

bool SenseGloveHelper::getHandPose(Eigen::MatrixXd& measuredValue)
{
    // to check [?]
    SGCore::HandProfile profile = SGCore::HandProfile::Default(m_glove.IsRight());
    SGCore::HandPose handPose;
    if (!m_glove.GetHandPose(profile, handPose))
    {
        yWarning() << LogPrefix << "m_glove.GetHandPose method of the glove returns error.";
        measuredValue = m_handPose;
        return true; // to avoid stopping the device
    }

    int count = 0;
    for (int i = 0; i < handPose.jointPositions.size(); i++)  // size is 5 (5 Fingers)
    {
        for (int j = 0; j < handPose.jointPositions[i].size(); j++)  // size is 3 (3 joints each)
        {
            m_handPose(count, 0) = handPose.jointPositions[i][j].x;
            m_handPose(count, 1) = handPose.jointPositions[i][j].y;
            m_handPose(count, 2) = handPose.jointPositions[i][j].z;

            m_handPose(count, 3) = handPose.jointRotations[i][j].x; // wrt to the origin frame
            m_handPose(count, 4) = handPose.jointRotations[i][j].y;
            m_handPose(count, 5) = handPose.jointRotations[i][j].z;
            m_handPose(count, 6) = handPose.jointRotations[i][j].w;
            count++;
        }
    }
    measuredValue = m_handPose;

    return true;
}

bool SenseGloveHelper::getHandJointsAngles()
{
    // to check [?]
    SGCore::HandProfile profile = SGCore::HandProfile::Default(m_glove.IsRight());
    SGCore::HandPose handPose;

    if (!m_glove.GetHandPose(profile, handPose))
    {
        yWarning() << LogPrefix << "m_glove.GetHandPose method of the glove returns error.";
        return true;
    }

    int count = 0;
    for (int i = 0; i < handPose.handAngles.size(); i++)  // size is 5 (5 Fingers)
    {
        for (int j = 0; j < handPose.handAngles[i].size(); j++)  // size is 3 (3 joints each)
        {
            // Euler representations of all possible hand angles
            m_handJointsAngles(count, 0) = handPose.handAngles[i][j].x;
            m_handJointsAngles(count, 1) = handPose.handAngles[i][j].y;
            m_handJointsAngles(count, 2) = handPose.handAngles[i][j].z;
            count++;
        }
    }
    return true;
}

bool SenseGloveHelper::getHandJointsAngles(std::vector<double> & jointAngleList)
{
    getHandJointsAngles( );
    if(jointAngleList.size()!=m_humanJointNameList.size())
    {
        jointAngleList.resize(m_humanJointNameList.size(),0.0); // 16
    }

    // thumb
    jointAngleList[0]=m_handJointsAngles(0, 2);
    jointAngleList[1]=m_handJointsAngles(0, 1);
    jointAngleList[2]=m_handJointsAngles(1, 1);
    jointAngleList[3]=m_handJointsAngles(2, 1);

    // m_handJointsAngles(3, :)--> all of them are zero
    // index (3:5)
    jointAngleList[4]=m_handJointsAngles(4, 1);
    jointAngleList[5]=m_handJointsAngles(5, 1);
    jointAngleList[6]=m_handJointsAngles(6, 1);

    // m_handJointsAngles(7, :)--> all of them are zero
    // middle (6:8)
    jointAngleList[7]=m_handJointsAngles(8, 1);
    jointAngleList[8]=m_handJointsAngles(9, 1);
    jointAngleList[9]=m_handJointsAngles(10, 1);

    // m_handJointsAngles(11, :)--> all of them are zero
    // ring (9:11)
    jointAngleList[10]=m_handJointsAngles(12, 1);
    jointAngleList[11]=m_handJointsAngles(13, 1);
    jointAngleList[12]=m_handJointsAngles(14, 1);

    // m_handJointsAngles(15, :)--> all of them are zero
    // pinkie (12:14)
    jointAngleList[13]=m_handJointsAngles(16, 1);
    jointAngleList[14]=m_handJointsAngles(17, 1);
    jointAngleList[15]=m_handJointsAngles(18, 1);

    return true;
}

bool SenseGloveHelper::getHandJointsAngles(Eigen::MatrixXd measuredValue )
{
    measuredValue = m_handJointsAngles;
    return true;
}

bool SenseGloveHelper::getGlovePose(Eigen::MatrixXd& measuredValue)
{
    SGCore::SG::SG_GlovePose glovePose;
    if (!m_glove.GetGlovePose(glovePose))
    {
        yWarning() << LogPrefix << "m_glove.GetGlovePose return error.";
        measuredValue = m_glovePose;
        return true;
    }

    int count = 0;
    for (int i = 0; i < glovePose.jointPositions.size(); i++) // glove no of fingers
    {
        for (int j = 0; j < glovePose.jointPositions[i].size(); j++) // glove's finger no of links
        {
            m_glovePose(count, 0) = glovePose.jointPositions[i][j].x;
            m_glovePose(count, 1) = glovePose.jointPositions[i][j].y;
            m_glovePose(count, 2) = glovePose.jointPositions[i][j].z;

            m_glovePose(count, 3) = glovePose.jointRotations[i][j].x; // wrt to the origin frame
            m_glovePose(count, 4) = glovePose.jointRotations[i][j].y;
            m_glovePose(count, 5) = glovePose.jointRotations[i][j].z;
            m_glovePose(count, 6) = glovePose.jointRotations[i][j].w;
            count++;
        }
    }
    measuredValue = m_glovePose;
    return true;
}

bool SenseGloveHelper::getGloveSensorData(std::vector<float>& measuredValues)
{
    SGCore::SG::SG_SensorData sensorData;
    if (!m_glove.GetSensorData(sensorData))
    {
        yWarning() << LogPrefix << "m_glove.GetSensorData return error.";
        measuredValues = m_sensorData;
        return true;
    }
    m_sensorData = sensorData.GetAngleSequence();
    measuredValues = m_sensorData;
    return true;
}

bool SenseGloveHelper::getGloveIMUData(std::vector<double>& gloveImuData)
{
     SGCore::Kinematics::Quat imu;

     if(gloveImuData.size()!=4)
     {
         gloveImuData.resize(4, 0.0);
     }

    if(!m_glove.GetIMURotation(imu))
    {
        yWarning()<< LogPrefix << "Cannot get glove IMU value";
        return true; // to avoid crashing
    }

    gloveImuData[0]= imu.w;
    gloveImuData[1]= imu.x;
    gloveImuData[2]= imu.y;
    gloveImuData[3]= imu.z;

    double norm= 0.0;
    for (size_t i=0; i<gloveImuData.size(); i++)
    {
        norm+= gloveImuData[i]*gloveImuData[i];
    }
    norm = std::sqrt(norm);

    for (size_t i=0; i<gloveImuData.size(); i++)
    {
        gloveImuData[i]= gloveImuData[i]/norm;
    }

    return true;
}

bool SenseGloveHelper::isGloveConnected()
{
    return m_glove.IsConnected();
}

bool SenseGloveHelper::turnOffBuzzMotors()
{
    m_glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd::off);
    return true;
}

bool SenseGloveHelper::turnOffForceFeedback()
{
    m_glove.SendHaptics(SGCore::Haptics::SG_FFBCmd::off);
    return true;
}

int SenseGloveHelper::getNoOfBuzzMotors() const
{
    return m_buzzDof;
}

int SenseGloveHelper::getNoOfForceFeedback() const
{
    return m_forceFbDof;
}


int SenseGloveHelper::getNoGloveLinks() const
{
    return m_gloveNoLinks;
}

int SenseGloveHelper::getNoHandLinks() const
{
    return m_handNoLinks;
}

int SenseGloveHelper::getNoSensors() const
{
    return m_NoSensors;
}

bool SenseGloveHelper::getHumanJointNameList( std::vector<std::string>& jointList)const {
    if (m_humanJointNameList.size()==0)
    {
        yError()<< LogPrefix<< "m_humanJointNameList vector size is zero.";
        return false;
    }

    jointList.resize(m_humanJointNameList.size());

    for(size_t i=0; i<m_humanJointNameList.size(); i++)
        jointList[i] = m_humanJointNameList[i];

    return true;
}

bool SenseGloveHelper::getHumanHandLinkName(std::string& handLinkName) const
{
    handLinkName=m_humanHandLinkName;
    return true;
}


bool SenseGloveHelper::getHumanFingerNameList(std::vector<std::string>& fingerList) const{

    if (m_humanFingerNameList.size()==0)
    {
        yError()<< LogPrefix<< "m_humanFingerNameList vector size is zero.";
        return false;
    }

    fingerList.resize(m_humanFingerNameList.size());

    for(size_t i=0; i<m_humanFingerNameList.size(); i++)
        fingerList[i] = m_humanFingerNameList[i];

    return true;
}


SenseGloveHelper::~SenseGloveHelper(){}

bool SenseGloveHelper::isRightHand() const
{
    return m_isRightHand;
}

bool SenseGloveHelper::close()
{
    turnOffBuzzMotors();
    turnOffForceFeedback();
    return true;
}
