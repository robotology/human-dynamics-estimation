/*!
 * @file RobotFrameTransformer.cpp
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */

#include <cmath>

#include "RobotFrameTransformer.h"

#include <yarp/os/Log.h>
#include <yarp/dev/IEncoders.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/EigenHelpers.h>


namespace human
{
    RobotFrameTransformer::RobotFrameTransformer(const iDynTree::Transform &constTransformFromFixture,
                                                 const std::string &inputFrame,
                                                 const std::string &robotLinkingFrame,
                                                 const std::string &humanLinkingFrame,
                                                 const std::string &outputFrame,
                                                 yarp::dev::IEncoders &robotJointConfiguration_encoder,
                                                 yarp::os::BufferedPort<HumanState> &humanJointConfigurationPort)
    : GenericFrameTransformer(inputFrame,outputFrame)
    , m_constTransformFromFixture(constTransformFromFixture)
    , m_robotLinkingFrame(robotLinkingFrame)
    , m_humanLinkingFrame(humanLinkingFrame)
    , m_originFrameIndex(iDynTree::FRAME_INVALID_INDEX)
    , m_robotLinkingFrameIndex(iDynTree::FRAME_INVALID_INDEX)
    , m_humanLinkingFrameIndex(iDynTree::FRAME_INVALID_INDEX)
    , m_transformedFrameIndex(iDynTree::FRAME_INVALID_INDEX)
    , m_robotJointConfiguration_encoder(robotJointConfiguration_encoder)
    , m_humanJointConfigurationPort(humanJointConfigurationPort){}
    
    
    bool RobotFrameTransformer::init(const iDynTree::Model &humanModel,
                                     const iDynTree::Model &robotModel)
    {
        if (!m_humanComputations.loadRobotModel(humanModel))
        {
            yError("Error in reading the human model!");
            return false;
        }
        
        m_humanConfiguration.resize(m_humanComputations.getNrOfDegreesOfFreedom());
        //FIXME: zero configuration is a valid configuration, but it could lead to errors
        m_humanConfiguration.zero();
        m_humanVelocity.resize(m_humanComputations.getNrOfDegreesOfFreedom());
        m_humanVelocity.zero();
        
        if (!m_robotComputations.loadRobotModel(robotModel))
        {
            yError("Error in reading the robot model!");
            return false;
        }
        
        m_robotConfiguration.resize(m_robotComputations.getNrOfDegreesOfFreedom());
        m_robotConfiguration.zero();
        m_robotVelocity.resize(m_robotComputations.getNrOfDegreesOfFreedom());
        m_robotVelocity.zero();
        
        // transforming frames in indeces
        m_originFrameIndex = m_robotComputations.getFrameIndex(getOriginFrame());
        if (m_originFrameIndex < 0)
        {
            yError("Error in indexing the the robot origin frame");
            return false;
        }
        
        m_robotLinkingFrameIndex = m_robotComputations.getFrameIndex(m_robotLinkingFrame);
        if (m_robotLinkingFrameIndex < 0)
        {
            yError("Error in indexing the robotLikingFrame");
            return false;
        }
        
        m_humanLinkingFrameIndex = m_humanComputations.getFrameIndex(m_humanLinkingFrame);
        if (m_humanLinkingFrameIndex < 0)
        {
            yError("Error in indexing the humanLikingFrame");
            return false;
        }
        
        m_transformedFrameIndex = m_humanComputations.getFrameIndex(getTransformedFrame());
        if (m_transformedFrameIndex < 0)
        {
            yError("Error in indexing the human transformed frame");
            return false;
        }
        
        return true;
    }
    
    
    bool RobotFrameTransformer::transformForceFrame(const yarp::sig::Vector &inputForce,
                                                    yarp::sig::Vector &transformedForce,
                                                    std::string &transformedExpressedFrame)
    {
        // UPDATE MATRIX
        
        iDynTree::Vector3 worldGravity;
        worldGravity.zero(); worldGravity(2) = -9.81;

        // -----read human configuration
        human::HumanState *tmp = m_humanJointConfigurationPort.read(false);
        
        // transform yarp Vector in iDynTree vector
        if (tmp && !iDynTree::toiDynTree(tmp->positions, m_humanConfiguration))
        {
            yError("Somenthing wrong in the conversion from a YARP to iDynTree vector!");
            return false;
        }
        
        // -----set human configuration
        m_humanComputations.setRobotState(m_humanConfiguration,
                                          m_humanVelocity,
                                          worldGravity);
        
        // -----read robot configuration
        m_robotJointConfiguration_encoder.getEncoders(m_robotConfiguration.data());
        
        // since raw robot data are in deg --> transformation in rad
        iDynTree::toEigen(m_robotConfiguration) *= M_PI / 180.0;
        
        // -----set robot configuration
        m_robotComputations.setRobotState(m_robotConfiguration,
                                          m_robotVelocity,
                                          worldGravity);
        
        // Important note:
        // In the following  lines we want to retrieve the following non const transformation:
        // outputFrame_T_inputframe = inv( inputFrame_T_robotLinkingFrame * robotLinkingFrame_T_humanLinkingFrame * robotLinkingFrame_T_outputFrame)
        
        // non const transform
        iDynTree::Transform robotArm_T_leftSole = m_robotComputations.getRelativeTransform(m_originFrameIndex,
                                                                                           m_robotLinkingFrameIndex);
        // non const transform
        iDynTree::Transform humanLeftFoot_T_humanHand = m_humanComputations.getRelativeTransform(m_humanLinkingFrameIndex,
                                                                                                 m_transformedFrameIndex);

        this->setTransform((robotArm_T_leftSole * m_constTransformFromFixture * humanLeftFoot_T_humanHand).inverse());
        //matrixTransform is the matrix humanHand_T_robotArm that converts forces in robot frames
        //into forces in human ones.
        
        return GenericFrameTransformer::transformForceFrame(inputForce,
                                                            transformedForce,
                                                            transformedExpressedFrame);
    }
}
