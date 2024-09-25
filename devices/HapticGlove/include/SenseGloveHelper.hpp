// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#ifndef SENSE_GLOVE_HELPER_HPP
#define SENSE_GLOVE_HELPER_HPP

#include <Eigen/Dense>

// std
#include <array>
#include <iostream>
#include <memory>
#include <stdint.h>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

// Sense Glove
#include <DeviceList.h>
#include <SenseGlove.h>

/**
 * GloveControlHelper is an helper class for controlling the glove.
 */
namespace senseGlove {
    class SenseGloveHelper;
    enum class ThumperCmd : unsigned int;
    const std::string LogPrefix = "senseGlove::SenseGloveHelper::";
    const size_t PoseSize = 7; // [ position <x, y, z>, quaternion <w, x, y, z> ]
} // namespace senseGlove

enum class senseGlove::ThumperCmd : unsigned int
{
    None = 126,

    /// <summary> Turn off the thumper effects. </summary>
    TurnOff = 124,

    /// <summary> A 5-second long, constant vibration. </summary>
    Cue_Game_Over = 118,

    /// <summary> A double-click at 100% intensity. </summary>
    Button_Double_100 = 10,
    /// <summary> A double click at 60% intensity. </summary>
    Button_Double_60 = 11,

    /// <summary> Simulates an impact of the hand at 100% intensity. </summary>
    Impact_Thump_100 = 1,
    /// <summary> Simulates an impact of the hand at 30% intensity. </summary>
    Impact_Thump_30 = 3,
    /// <summary> Simulates an sharp impact of the hand at 40% intensity.
    /// </summary>
    Impact_Thump_10 = 6,

    /// <summary> A light vibration to cue the user that an object it picked up.
    /// 100% intensity. </summary>
    Object_Grasp_100 = 7,
    /// <summary> A light vibration to cue the user that an object it picked up.
    /// 60% intensity. </summary>
    Object_Grasp_60 = 8,
    /// <summary> A light vibration to cue the user that an object it picked up.
    /// 30% intensity. </summary>
    Object_Grasp_30 = 9
};

class senseGlove::SenseGloveHelper
{

    /// Number of the actuated motors Dofs to produce force feedback to the human
    int m_forceFbDof;

    /// Number of the actuated vibro-tactile Dofs to produce vibro tactile
    /// feedback to the human
    int m_buzzDof;

    /// Number of the links of the human hand model
    int m_handNoLinks;

    /// Number of the links of the human hand model used for retrieving the euler
    /// angles
    int m_handNoLinksForEulerAngles;

    /// Number of the links of the glove
    int m_gloveNoLinks;

    /// Number of the sensors of the glove
    int m_NoJointSensors;

    /// true if the glove is ready to use, i.e., communication working
    bool m_isReady;

    /// true if the glove is the right hand
    bool m_isRightHand;

    /// Desired force feedback [N], resistence force between 0-40 N transformed to
    /// percentage 0-100
    std::vector<int> m_desiredForceValues;

    /// Desired vibro-tactile feedbacks, percentage 0-100
    std::vector<int> m_desiredBuzzValues;

    /// sensory data of the glove in radians
    std::vector<float> m_sensorData;

    /// sensory data of the glove poses
    Eigen::MatrixXd m_glovePose;

    /// sensory data of the hand link poses;  from thumb to pinky, proximal to
    /// distal, pos [x y z] Quat [x y z w]
    Eigen::MatrixXd m_handPose;

    /// sensory data of the human hand joints angles; From thumb to pinky,
    /// proximal to distal [rad] [Pronation/Supination (x), Flexion/Extension (y),
    /// Abduction/Adduction (z)]
    Eigen::MatrixXd m_handOrientationEulerAngles;

    /// vector of the human joint names
    std::vector<std::string> m_humanJointNameList;

    /// vector of the human finger names
    std::vector<std::string> m_humanFingerNameList;

    /// the name of the human hand link name
    std::string m_humanHandLinkName;

    /// the object to interface with the sense glove sdk
    SGCore::SG::SenseGlove m_glove;

    /**
     * Setup the communication with the glove
     * @return true / false in case of success / failure
     */
    bool setupGlove();

    /**
     * Get the human hand joint angles from the sense glove data structure
     * @return true / false in case of success / failure
     */
    bool getHandJointsAngles();

public:
    /**
      Constructor
    **/
    SenseGloveHelper();

    /**
      Destructor
    **/
    ~SenseGloveHelper();

    /**
     * Configure the class
     * @param config configuration options
     * @param rightHand if true, the right hand glove will be configured,
     * otherwise left.
     * @return true / false in case of success / failure
     */
    bool configure(const yarp::os::Searchable& config);

    /**
     * Set the desired Force Feedback for all the fingers
     * @param desiredValue desired force feedback of all the fingers
     * @return true / false in case of success / failure
     */
    bool setFingersForceReference(const std::vector<double>& desiredValue);

    /**
     * Set the desired vibro-tactile feedback for all the fingers
     * @param desiredValue desired vibro-tactile values
     * @return true / false in case of success / failure
     */
    bool setBuzzMotorsReference(const std::vector<double>& desiredValue);

    /**
     * Set the desired vibro-tactile feedback for the palm
     * @param desiredValue desired vibro-tactile value of the palm
     * @return true / false in case of success / failure
     */
    bool setPalmFeedbackThumper(ThumperCmd desiredValue);

    /**
     * Get the measured hand link poses values
     * @param measuredValue measured joint values
     * @return true / false in case of success / failure
     */
    bool getHandLinksPose(Eigen::MatrixXd& measuredValue);

    /**
     * Get the human hand joint angles
     * @param jointAngleList std vector of doubles of human hand joint angles
     * @return true / false in case of success / failure
     */
    bool getHandJointsAngles(std::vector<double>& jointAngleList);

    /**
     * Get the human hand joint angles
     * @param measuredValue Eigen matrix of human hand joint angles
     * @return true / false in case of success / failure
     */
    bool getHandJointsAngles(Eigen::MatrixXd measuredValue);

    /**
     * Get the glove link poses
     * @param measuredValue Eigen matrix of glove poses
     * @return true / false in case of success / failure
     */
    bool getGloveLinksPose(Eigen::MatrixXd& measuredValue);

    /**
     * Get the fingertip poses based on glove sensory data
     * @param fingertipPoses Eigen matrix of glove poses [(pos: x, y, z) ,
     * (rotation: w, x, y, z )]
     * @return true / false in case of success / failure
     */
    bool getGloveFingertipLinksPose(std::vector<std::vector<double>>& fingertipPoses);

    /**
     * Get the glove sensory data
     * @param measuredValue vector of glove sensory data
     * @return true / false in case of success / failure
     */
    bool getGloveSensorData(std::vector<float>& measuredValues);

    /**
     * Get the glove IMU data
     * @param palmLinkPose human palm pose based on glove IMU data with the order
     * pos(x y z), quat(w x y z)
     * @return true / false in case of success / failure
     */
    bool getPalmLinkPose(std::vector<double>& palmLinkPose);

    /**
     * Trun off the vibro-tactile feedback
     * @return true / false in case of success / failure
     */
    bool turnOffBuzzMotors();

    /**
     * Trun off the force feedback
     * @return true / false in case of success / failure
     */
    bool turnOffForceFeedback();

    /**
     * Trun off the palm thumper feedback
     * @return true / false in case of success / failure
     */
    bool turnOffPalmFeedbackThumper();

    /**
     * get the number of buzz motors/vibro-tactile feedback motors
     * @return number of buzz motors
     */
    int getNumOfBuzzMotors() const;

    /**
     * get the number of force feedback motors
     * @return number of force feedback motors
     */
    int getNumOfForceFeedback() const;

    /**
     * Get the number of hand links
     * @return the number of hand links
     */
    int getNumHandLinks() const;

    /**
     * Get the number of glove links
     * @return the number of glove links
     */
    int getNumGloveLinks() const;

    /**
     * Get the number of glove sensors
     * @return the number of glove sensors
     */
    int getNumSensors() const;

    /**
     * Check if the glove is connected
     * @return true / false in case of connected / disconnected
     */
    bool isGloveConnected();

    /**
     * Get the human joint list
     * @param jointList the human joint list
     * @return true / false in case of success / failure
     */
    bool getHumanJointNameList(std::vector<std::string>& jointList) const;

    /**
     * Get the human hand link name
     * @param handLinkName the human hand link name
     * @return true / false in case of success / failure
     */
    bool getHumanHandLinkName(std::string& handLinkName) const;

    /**
     * Get the human finger list
     * @param fingerList the human joint list
     * @return true / false in case of success / failure
     */
    bool getHumanFingerNameList(std::vector<std::string>& fingerList) const;

    /**
     * Get the left/right hand
     * @return true / false in case of right / left hand
     */
    bool isRightHand() const;

    /**
     * close the device
     * @return true / false in case of connected / disconnected
     */
    bool close();
};

#endif
