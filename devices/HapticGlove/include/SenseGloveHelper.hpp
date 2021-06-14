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
#include <memory>
#include <iostream>

// YARP

#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>


// Sense Glove
#include <DeviceList.h>
#include <SenseGlove.h>

/**
 * GloveControlHelper is an helper class for controlling the glove.
 */
namespace senseGlove
{
    class SenseGloveHelper;
    bool yarpListToStringVector(yarp::os::Value*& input, std::vector<std::string>& output);

}

class senseGlove::SenseGloveHelper
{

    int m_forceFbDof; /**< Number of the actuated motors Dofs to produce force feedback to the
                         human*/
    int m_buzzDof; /**< Number of the actuated vibro tactile Dofs to produce vibro tactile
                         feedback to the human*/
    int m_handNoLinks; /**< Number of the links of the hand model*/

    int m_gloveNoLinks; /**< Number of the links of the hand model*/

    int m_NoSensors;  /**< Number of the sensors of the glove */

    bool m_isReady; /**< true if the glove is ready to use, communication working*/

    bool m_isRightHand; /**< true if the glove is the right hand*/


    std::vector<int> m_desiredForceValues; /**< Desired joint value [deg or deg/s]. */
    std::vector<int> m_desiredBuzzValues; /**< Joint position [deg]. */
    std::vector<float> m_sensorData; /**< sensory data of the glove in degree */
    Eigen::MatrixXd m_glovePose; /**< sensory data of the glove poses*/
    Eigen::MatrixXd m_handPose; /**< sensory data of the hand link poses;  From thumb to pinky, proximal to distal;
                                pos [x y z] Quat [x y z w]*/

    Eigen::MatrixXd m_handJointsAngles; /**< sensory data of the hand joints angles;  From thumb to pinky,
                                proximal to distal [rad] [Pronation/Supination (x), Flexion/Extension (y), Abduction/Adduction (z)]*/

    yarp::sig::Vector m_jointsFeedbackInRadians; /**< Joint position [rad]. */

    std::vector<std::string> m_humanJointNameList;

    SGCore::SG::SenseGlove m_glove;

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
     * @param rightHand if true, the right hand glove will be configured, otherwise left.
     * @return true / false in case of success / failure
     */
    bool configure(const yarp::os::Searchable& config, const bool& rightHand);

    /**
     * Setup the communication with the glove
     * @return true / false in case of success / failure
     */
    bool setupGlove();

    /**
     * Set the desired Force Feedback for all the fingers
     * @param desiredValue desired force feedback of all the fingers
     * @return true / false in case of success / failure
     */
    bool setFingersForceReference(const yarp::sig::Vector& desiredValue);

    /**
     * Set the desired vibro-tactile feedback for all the fingers
     * @param desiredValue desired vibro-tactile values
     * @return true / false in case of success / failure
     */
    bool setBuzzMotorsReference(const yarp::sig::Vector& desiredValue);

    /**
     * Set the desired vibro-tactile feedback for the palm
     * @param desiredValue desired vibro-tactile value of the palm
     * @return true / false in case of success / failure
     */
    bool setPalmFeedbackThumper(const int desiredValue);

    /**
     * Get the measured joint values
     * @param measuredValue measured joint values
     * @return true / false in case of success / failure
     */
    bool getHandPose(Eigen::MatrixXd& measuredValue);


    bool getHandJointsAngles();


    bool getHandJointsAngles(std::vector<double> & jointAngleList) ;


    bool getHandJointsAngles(Eigen::MatrixXd measuredValue);


    bool getGlovePose(Eigen::MatrixXd& measuredValue);


    bool getGloveSensorData(std::vector<float>& measuredValues);
    
    /**
     * Set the number of vibro-tactile reference
     * @param desiredValue desired vibro-tactile values
     * @return true / false in case of success / failure
     */
    bool turnOffBuzzMotors();

    bool turnOffForceFeedback();

    const int getNoOfBuzzMotors() const;

    int getNoOfForceFeedback();

    /**
     * Check if the glove is connected
     * @return true / false in case of connected / disconnected
     */
    bool isGloveConnected();

    /**
     * Get the number of hand links
     * @return the number of hand links
     */
    int getNoHandLinks();

    /**
     * Get the number of glove links
     * @return the number of glove links
     */
    int getNoGloveLinks();

    /**
     * Get the number of glove sensors
     * @return the number of glove sensors
     */
    int getNoSensors();

    /**
     * Get the human joint list
     * @param jointList the human joint list
     */
    void getHumanJointNameList(std::vector<std::string>& jointList)const ;

    /**
     * Get the glove IMU data
     * @param gloveImuData glove IMU data with the order x y z w
     * @return true / false in case of success / failure
     */
    bool getGloveIMUData(std::vector<double>& gloveImuData);

};

/**
 * Utility function: transform yarp list to a std string vector
 * @param input yarp list
 * @param output generated std string vector
 * @return true / false in case of success / failure
 */
bool senseGlove::yarpListToStringVector(yarp::os::Value*& input, std::vector<std::string>& output)
{
    // clear the std::vector
    output.clear();

    // check if the yarp value is a list
    if (!input->isList())
    {
        yError() << "[yarpListToStringVector] The input is not a list.";
        return false;
    }

    yarp::os::Bottle* bottle = input->asList();
    for (int i = 0; i < bottle->size(); i++)
    {
        // check if the elements of the bottle are strings
        if (!bottle->get(i).isString())
        {
            yError() << "[yarpListToStringVector] There is a field that is not a string.";
            return false;
        }
        output.push_back(bottle->get(i).asString());
    }
    return true;
}


#endif
