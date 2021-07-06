/**
 * @file SenseGloveHelper.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef SENSE_GLOVE_HELPER_HPP
#define SENSE_GLOVE_HELPER_HPP

#include <Eigen/Dense>

// std
#include <array>
#include <iostream>
#include <memory>

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
const std::string LogPrefix = "senseGlove::SenseGloveHelper::";
const size_t PoseSize = 7;
} // namespace senseGlove

class senseGlove::SenseGloveHelper {

  int m_forceFbDof; /**< Number of the actuated motors Dofs to produce force
                   feedback to the human*/

  int m_buzzDof; /**< Number of the actuated vibro-tactile Dofs to produce vibro
                    tactile feedback to the human*/

  int m_handNoLinks; /**< Number of the links of the human hand model*/

  int m_handNoLinksForEulerAngles; /**< Number of the links of the human hand
                                      model used for retrieving the euler
                                      angles*/

  int m_gloveNoLinks; /**< Number of the links of the glove*/

  int m_NoJointSensors; /**< Number of the sensors of the glove */

  bool m_isReady; /**< true if the glove is ready to use, i.e., communication
                     working*/

  bool m_isRightHand; /**< true if the glove is the right hand*/

  //
  std::vector<int>
      m_desiredForceValues; /**< Desired force feedback [N], resistence force
                               between 0-40 N transformed to percentage 0-100 */

  std::vector<int> m_desiredBuzzValues; /**< Desired vibro-tactile feedbacks,
                                           percentage 0-100*/

  std::vector<float> m_sensorData; /**< sensory data of the glove in degree [?
                                      or radians] // to check */

  Eigen::MatrixXd
      m_glovePose; /**< sensory data of the glove poses // to check */

  Eigen::MatrixXd
      m_handPose; /**< sensory data of the hand link poses;  from thumb to
                     pinky, proximal to distal, pos [x y z] Quat [x y z w]*/

  Eigen::MatrixXd
      m_handOrientationEulerAngles; /**< sensory data of the human hand joints
                             angles; From thumb to pinky, proximal to distal
                             [rad] [Pronation/Supination (x), Flexion/Extension
                             (y), Abduction/Adduction (z)]*/

  std::vector<std::string>
      m_humanJointNameList; /**< vector of the human joint names */

  std::vector<std::string>
      m_humanFingerNameList; /**< vector of the human finger names */

  std::string m_humanHandLinkName; /**< the name of the human hand link name*/

  SGCore::SG::SenseGlove
      m_glove; /**< the object to interface with the sense glove sdk */

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
   * @param config confifuration options
   * @param rightHand if true, the right hand glove will be configured,
   * otherwise left.
   * @return true / false in case of success / failure
   */
  bool configure(const yarp::os::Searchable &config);

  /**
   * Set the desired Force Feedback for all the fingers
   * @param desiredValue desired force feedback of all the fingers
   * @return true / false in case of success / failure
   */
  bool setFingersForceReference(const std::vector<double> &desiredValue);

  /**
   * Set the desired vibro-tactile feedback for all the fingers
   * @param desiredValue desired vibro-tactile values
   * @return true / false in case of success / failure
   */
  bool setBuzzMotorsReference(const std::vector<double> &desiredValue);

  /**
   * Set the desired vibro-tactile feedback for the palm
   * @param desiredValue desired vibro-tactile value of the palm
   * @return true / false in case of success / failure
   */
  bool setPalmFeedbackThumper(const int desiredValue);

  /**
   * Get the measured hand link poses values
   * @param measuredValue measured joint values
   * @return true / false in case of success / failure
   */
  bool getHandLinksPose(Eigen::MatrixXd &measuredValue);

  /**
   * Get the human hand joint angles
   * @param jointAngleList std vector of doubles of human hand joint angles
   * @return true / false in case of success / failure
   */
  bool getHandJointsAngles(std::vector<double> &jointAngleList);

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
  bool getGloveLinksPose(Eigen::MatrixXd &measuredValue);

  /**
   * Get the fingertip poses based on glove sensory data
   * @param fingertipPoses Eigen matrix of glove poses [(pos: x, y, z) ,
   * (rotation: w, x, y, z )]
   * @return true / false in case of success / failure
   */
  bool
  getGloveFingertipLinksPose(std::vector<std::vector<double>> &fingertipPoses);

  /**
   * Get the glove sensory data
   * @param measuredValue vector of glove sensory data
   * @return true / false in case of success / failure
   */
  bool getGloveSensorData(std::vector<float> &measuredValues);

  /**
   * Get the glove IMU data
   * @param palmLinkPose human palm pose based on glove IMU data with the order
   * pos(x y z), quat(w x y z)
   * @return true / false in case of success / failure
   */
  bool getPalmLinkPose(std::vector<double> &palmLinkPose);

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
   * get the number of buzz motors/vibro-tactile feedback motors
   * @return number of buzz motors
   */
  int getNoOfBuzzMotors() const;

  /**
   * get the number of force feedback motors
   * @return number of force feedback motors
   */
  int getNoOfForceFeedback() const;

  /**
   * Get the number of hand links
   * @return the number of hand links
   */
  int getNoHandLinks() const;

  /**
   * Get the number of glove links
   * @return the number of glove links
   */
  int getNoGloveLinks() const;

  /**
   * Get the number of glove sensors
   * @return the number of glove sensors
   */
  int getNoSensors() const;

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
  bool getHumanJointNameList(std::vector<std::string> &jointList) const;

  /**
   * Get the human hand link name
   * @param handLinkName the human hand link name
   * @return true / false in case of success / failure
   */
  bool getHumanHandLinkName(std::string &handLinkName) const;

  /**
   * Get the human finger list
   * @param fingerList the human joint list
   * @return true / false in case of success / failure
   */
  bool getHumanFingerNameList(std::vector<std::string> &fingerList) const;

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
