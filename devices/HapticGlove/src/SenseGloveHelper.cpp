// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <limits>

#include <SenseGloveHelper.hpp>

using namespace senseGlove;

SenseGloveHelper::SenseGloveHelper()
{
    yInfo() << LogPrefix << "SenseGloveHelper()";

    m_isReady = false;
    m_forceFbDof = 5;
    m_buzzDof = 5;
    m_gloveNoLinks = 30;
    m_handNoLinks = 20;
    m_handNoLinksForEulerAngles = 15;
    m_NoJointSensors = 20;

    m_desiredBuzzValues.resize(m_buzzDof, 0);
    m_desiredForceValues.resize(m_forceFbDof, 0);
    m_glovePose = Eigen::MatrixXd::Zero(m_gloveNoLinks, 7);
    m_handPose = Eigen::MatrixXd::Zero(m_handNoLinks, 7);
    m_handOrientationEulerAngles = Eigen::MatrixXd::Zero(m_handNoLinksForEulerAngles, 3);
}

bool SenseGloveHelper::configure(const yarp::os::Searchable& config)
{
    yInfo() << LogPrefix << "configure:: ";

    if (!(config.check("rightHand") && config.find("rightHand").isBool())) {
        yInfo() << LogPrefix << "Using default hand Sense Glove: Right hand";
        m_isRightHand = true;
    }
    else {
        m_isRightHand = config.find("rightHand").asBool();
        yInfo() << LogPrefix << "Using the right hand: " << m_isRightHand
                << "(if false, using left hand)";
    }

    // get human hand link name
    if (!(config.check("hand_link") && config.find("hand_link").isString())) {
        yError() << LogPrefix << "Unable to find hand_link in the config file.";
        return false;
    }
    m_humanHandLinkName = config.find("hand_link").asString();
    yInfo() << LogPrefix << "human hand link name: " << m_humanHandLinkName;

    // get human hand joint names
    yarp::os::Bottle* jointListYarp;
    if (!(config.check("human_joint_list") && config.find("human_joint_list").isList())) {
        yError() << LogPrefix << "Unable to find human_joint_list in the config file.";
        return false;
    }
    jointListYarp = config.find("human_joint_list").asList();

    for (size_t i = 0; i < jointListYarp->size(); i++) {
        m_humanJointNameList.push_back(jointListYarp->get(i).asString());
    }
    yInfo() << LogPrefix << "human joint names: " << m_humanJointNameList;

    // get human hand finger names
    yarp::os::Bottle* fingerListYarp;
    if (!(config.check("human_finger_list") && config.find("human_finger_list").isList())) {
        yError() << LogPrefix << "Unable to find human_finger_list in the config file.";
        return false;
    }
    fingerListYarp = config.find("human_finger_list").asList();

    for (size_t i = 0; i < fingerListYarp->size(); i++) {
        m_humanFingerNameList.push_back(fingerListYarp->get(i).asString());
    }
    yInfo() << LogPrefix << "human finger names: " << m_humanFingerNameList;

    if (!setupGlove()) {
        yError() << LogPrefix << "Unable to set up the sense glove.";
        return false;
    }

    return true;
}

bool SenseGloveHelper::setupGlove()
{
    yInfo() << LogPrefix << "setupGlove()";

    if (!SGCore::DeviceList::SenseCommRunning()) // Returns true if SenseComm is
                                                 // running.
    {
        yError() << LogPrefix << "SenseComm is not running. Please run SenseComm, then try again.";
        return false;
    }

    if (!SGCore::SG::SenseGlove::GetSenseGlove(m_isRightHand, m_glove)) {
        yError() << LogPrefix
                 << "No sense gloves connected to the system. Ensure the USB "
                    "connection is "
                    "secure, then try again.";
        return false;
    }

    SGCore::SG::SG_GloveInfo gloveModel = m_glove.GetGloveModel();
    yInfo() << LogPrefix << "glove model:" << gloveModel.ToString(true);

    return true;
}

bool SenseGloveHelper::setFingersForceReference(const std::vector<double>& desiredValue)
{
    if (desiredValue.size() != m_forceFbDof) {
        yError() << LogPrefix
                 << "Size of the input desired vecotr and the number of haptic "
                    "force feedbacks are not equal.";
        return false;
    }

    for (size_t i = 0; i < m_forceFbDof; i++) {
        m_desiredForceValues[i] =
            (int) std::round(std::max(0.0, std::min(desiredValue[i], 40.0)) * 100 / 40);
    }

    m_glove.SendHaptics(SGCore::Haptics::SG_FFBCmd(m_desiredForceValues));

    return true;
}

bool SenseGloveHelper::setBuzzMotorsReference(const std::vector<double>& desiredValue)
{
    if (desiredValue.size() != m_buzzDof) {
        yError() << LogPrefix
                 << "Size of the input desired vector and the number of buzz "
                    "motors are not equal.";
        return false;
    }
    for (size_t i = 0; i < m_buzzDof; i++) {
        m_desiredBuzzValues[i] = (int) std::round(std::max(0.0, std::min(desiredValue[i], 100.0)));
    }
    m_glove.SendHaptics(SGCore::Haptics::SG_BuzzCmd(m_desiredBuzzValues));

    return true;
}

bool SenseGloveHelper::setPalmFeedbackThumper(ThumperCmd desiredValue)
{

    bool result;
    switch (desiredValue) {
        case ThumperCmd::None:
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::None);
            break;

        case ThumperCmd::TurnOff:
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::TurnOff);
            break;

        case ThumperCmd::Cue_Game_Over:
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Cue_Game_Over);
            break;

        case ThumperCmd::Button_Double_100:
            //    result =
            //        m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Button_Double_100);
            yWarning() << LogPrefix
                       << "The glove may stop working due to overloal, so "
                          "Button_Double_60 is passed instead of Button_Double_100";
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Button_Double_60);
            break;

        case ThumperCmd::Button_Double_60:
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Button_Double_60);
            break;

        case ThumperCmd::Impact_Thump_100:
            //    result =
            //        m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Impact_Thump_100);
            yWarning() << LogPrefix
                       << "The glove may stop working due to overloal, so "
                          "Impact_Thump_30 is passed instead of Impact_Thump_100";
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Impact_Thump_30);
            break;

        case ThumperCmd::Impact_Thump_30:
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Impact_Thump_30);
            break;

        case ThumperCmd::Impact_Thump_10:
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Impact_Thump_10);
            break;

        case ThumperCmd::Object_Grasp_100:
            //    result =
            //        m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Object_Grasp_100);
            yWarning() << LogPrefix
                       << "The glove may stop working due to overloal, so "
                          "Object_Grasp_60 is passed instead of Object_Grasp_100";
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Object_Grasp_60);
            break;

        case ThumperCmd::Object_Grasp_60:
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Object_Grasp_60);
            break;

        case ThumperCmd::Object_Grasp_30:
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::Object_Grasp_30);
            break;

        default:
            result = m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::TurnOff);
            break;
    }

    return result;
}

bool SenseGloveHelper::getHandLinksPose(Eigen::MatrixXd& measuredValue)
{

    SGCore::HandProfile profile = SGCore::HandProfile::Default(m_glove.IsRight());
    SGCore::HandPose handPose;
    if (!m_glove.GetHandPose(profile, handPose)) {
        yWarning() << LogPrefix << "m_glove.GetHandPose method of the glove returns error.";
        measuredValue = m_handPose;
        return true; // to avoid stopping the device
    }

    int count = 0;
    // size is 5 (5 Fingers)
    for (int i = 0; i < handPose.jointPositions.size(); i++) {
        // size is 4 (4 links each finger)
        for (int j = 0; j < handPose.jointPositions[i].size(); j++) {
            m_handPose(count, 0) = handPose.jointPositions[i][j].x;
            m_handPose(count, 1) = handPose.jointPositions[i][j].y;
            m_handPose(count, 2) = handPose.jointPositions[i][j].z;

            // wrt to the origin frame
            m_handPose(count, 3) = handPose.jointRotations[i][j].w;
            m_handPose(count, 4) = handPose.jointRotations[i][j].x;
            m_handPose(count, 5) = handPose.jointRotations[i][j].y;
            m_handPose(count, 6) = handPose.jointRotations[i][j].z;
            count++;
        }
    }
    measuredValue = m_handPose;

    return true;
}

bool SenseGloveHelper::getHandJointsAngles()
{

    SGCore::HandProfile profile = SGCore::HandProfile::Default(m_glove.IsRight());
    SGCore::HandPose handPose;

    if (!m_glove.GetHandPose(profile, handPose)) {
        yWarning() << LogPrefix << "m_glove.GetHandPose method of the glove returns error.";
        return true;
    }

    int count = 0;
    // size is 5 (5 Fingers)
    for (int i = 0; i < handPose.handAngles.size(); i++) {
        // size is 3
        for (int j = 0; j < handPose.handAngles[i].size(); j++) {
            // Euler representations of all possible hand angles
            m_handOrientationEulerAngles(count, 0) = handPose.handAngles[i][j].x;
            m_handOrientationEulerAngles(count, 1) = handPose.handAngles[i][j].y;
            m_handOrientationEulerAngles(count, 2) = handPose.handAngles[i][j].z;
            count++;
        }
    }
    return true;
}

bool SenseGloveHelper::getHandJointsAngles(std::vector<double>& jointAngleList)
{
    getHandJointsAngles();

    //  if (jointAngleList.size() != m_humanJointNameList.size()) {
    //  yInfo() << "m_humanJointNameList.size(): " << m_humanJointNameList.size();
    jointAngleList.resize(m_humanJointNameList.size(), 0.0); // 20
                                                             //  }
    //  std::cout << "m_handOrientationEulerAngles\n"
    //            << m_handOrientationEulerAngles << std::endl;

    // thumb (0:3)
    jointAngleList[0] = m_handOrientationEulerAngles(0, 2);
    jointAngleList[1] = m_handOrientationEulerAngles(0, 1);
    jointAngleList[2] = m_handOrientationEulerAngles(1, 1);
    jointAngleList[3] = m_handOrientationEulerAngles(2, 1);

    // index (4:7)
    jointAngleList[4] = m_handOrientationEulerAngles(3, 2);
    jointAngleList[5] = m_handOrientationEulerAngles(3, 1);
    jointAngleList[6] = m_handOrientationEulerAngles(4, 1);
    jointAngleList[7] = m_handOrientationEulerAngles(5, 1);

    // middle (8:11)
    jointAngleList[8] = m_handOrientationEulerAngles(6, 2);
    jointAngleList[9] = m_handOrientationEulerAngles(6, 1);
    jointAngleList[10] = m_handOrientationEulerAngles(7, 1);
    jointAngleList[11] = m_handOrientationEulerAngles(8, 1);

    // ring (12:15)
    jointAngleList[12] = m_handOrientationEulerAngles(9, 2);
    jointAngleList[13] = m_handOrientationEulerAngles(9, 1);
    jointAngleList[14] = m_handOrientationEulerAngles(10, 1);
    jointAngleList[15] = m_handOrientationEulerAngles(11, 1);

    // pinkie (16:19)
    jointAngleList[16] = m_handOrientationEulerAngles(12, 2);
    jointAngleList[17] = m_handOrientationEulerAngles(12, 1);
    jointAngleList[18] = m_handOrientationEulerAngles(13, 1);
    jointAngleList[19] = m_handOrientationEulerAngles(14, 1);

    return true;
}

bool SenseGloveHelper::getHandJointsAngles(Eigen::MatrixXd measuredValue)
{
    measuredValue = m_handOrientationEulerAngles;
    return true;
}

bool SenseGloveHelper::getGloveLinksPose(Eigen::MatrixXd& measuredValue)
{
    SGCore::SG::SG_GlovePose glovePose;
    if (!m_glove.GetGlovePose(glovePose)) {
        yWarning() << LogPrefix << "m_glove.GetGlovePose return error.";
        measuredValue = m_glovePose;
        return true;
    }

    int count = 0;
    // glove no of fingers :5
    for (int i = 0; i < glovePose.jointPositions.size(); i++) {
        // glove's finger no of links : 6
        for (int j = 0; j < glovePose.jointPositions[i].size(); j++) {
            m_glovePose(count, 0) = glovePose.jointPositions[i][j].x / 1000.0; // mm to meter
            m_glovePose(count, 1) = glovePose.jointPositions[i][j].y / 1000.0; // mm to meter
            m_glovePose(count, 2) = glovePose.jointPositions[i][j].z / 1000.0; // mm to meter

            // wrt to the origin frame
            m_glovePose(count, 3) = glovePose.jointRotations[i][j].w;
            m_glovePose(count, 4) = glovePose.jointRotations[i][j].x;
            m_glovePose(count, 5) = glovePose.jointRotations[i][j].y;
            m_glovePose(count, 6) = glovePose.jointRotations[i][j].z;
            count++;
        }
    }
    measuredValue = m_glovePose;
    return true;
}

bool SenseGloveHelper::getGloveFingertipLinksPose(std::vector<std::vector<double>>& fingertipPoses)
{

    if (fingertipPoses.size() != m_humanFingerNameList.size()) {
        fingertipPoses.resize(m_humanFingerNameList.size(), std::vector<double>(PoseSize));
    }
    // avoid iterating on all the elements
    if (fingertipPoses[0].size() != PoseSize) {

        fingertipPoses.resize(m_humanFingerNameList.size(), std::vector<double>(PoseSize));
    }

    Eigen::MatrixXd glovePoses;
    getGloveLinksPose(glovePoses); // 30X7
    if (glovePoses.rows() != m_gloveNoLinks || glovePoses.cols() != PoseSize) {
        yWarning() << LogPrefix << "glovePoses size is not correct:: rows:" << glovePoses.rows()
                   << " , cols:" << glovePoses.cols();
        return true; // to avoid stoping
    }

    for (size_t i = 0; i < m_humanFingerNameList.size(); i++) {
        for (size_t j = 0; j < PoseSize; j++)
            fingertipPoses[i][j] = glovePoses(i * 6 + 5,
                                              j); // the last value of each finger for the glove
                                                  // data is associated with the fingertip
    }

    return true;
}

bool SenseGloveHelper::getGloveSensorData(std::vector<float>& measuredValues)
{
    SGCore::SG::SG_SensorData sensorData;
    if (!m_glove.GetSensorData(sensorData)) {
        yWarning() << LogPrefix << "m_glove.GetSensorData return error.";
        measuredValues = m_sensorData;
        return true;
    }
    m_sensorData = sensorData.GetAngleSequence();
    measuredValues = m_sensorData;
    return true;
}

bool SenseGloveHelper::getPalmLinkPose(std::vector<double>& palmLinkPose)
{
    SGCore::Kinematics::Quat imu;

    if (palmLinkPose.size() != 7) {
        palmLinkPose.resize(7, 0.0);
    }

    if (!m_glove.GetIMURotation(imu)) {
        yWarning() << LogPrefix << "Cannot get glove IMU value";
        return true; // to avoid crashing
    }
    // position
    palmLinkPose[0] = 0.0;
    palmLinkPose[1] = 0.0;
    palmLinkPose[2] = 0.0;
    // orientation: IMU
    palmLinkPose[3] = imu.w;
    palmLinkPose[4] = imu.x;
    palmLinkPose[5] = imu.y;
    palmLinkPose[6] = imu.z;

    double norm = 0.0;
    for (size_t i = 3; i < palmLinkPose.size(); i++) {
        norm += palmLinkPose[i] * palmLinkPose[i];
    }
    norm = std::sqrt(norm);

    for (size_t i = 3; i < palmLinkPose.size(); i++) {
        palmLinkPose[i] = palmLinkPose[i] / norm;
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

bool SenseGloveHelper::turnOffPalmFeedbackThumper()
{
    m_glove.SendHaptics(SGCore::Haptics::SG_ThumperCmd::TurnOff);
    return true;
}

int SenseGloveHelper::getNumOfBuzzMotors() const
{
    return m_buzzDof;
}

int SenseGloveHelper::getNumOfForceFeedback() const
{
    return m_forceFbDof;
}

int SenseGloveHelper::getNumGloveLinks() const
{
    return m_gloveNoLinks;
}

int SenseGloveHelper::getNumHandLinks() const
{
    return m_handNoLinks;
}

int SenseGloveHelper::getNumSensors() const
{
    return m_NoJointSensors;
}

bool SenseGloveHelper::getHumanJointNameList(std::vector<std::string>& jointList) const
{
    if (m_humanJointNameList.size() == 0) {
        yError() << LogPrefix << "m_humanJointNameList vector size is zero.";
        return false;
    }

    jointList.resize(m_humanJointNameList.size());

    for (size_t i = 0; i < m_humanJointNameList.size(); i++)
        jointList[i] = m_humanJointNameList[i];

    return true;
}

bool SenseGloveHelper::getHumanHandLinkName(std::string& handLinkName) const
{
    handLinkName = m_humanHandLinkName;
    return true;
}

bool SenseGloveHelper::getHumanFingerNameList(std::vector<std::string>& fingerList) const
{

    if (m_humanFingerNameList.size() == 0) {
        yError() << LogPrefix << "m_humanFingerNameList vector size is zero.";
        return false;
    }

    fingerList.resize(m_humanFingerNameList.size());

    for (size_t i = 0; i < m_humanFingerNameList.size(); i++)
        fingerList[i] = m_humanFingerNameList[i];

    return true;
}

SenseGloveHelper::~SenseGloveHelper() {}

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
