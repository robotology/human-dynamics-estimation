/*!
 * @file RobotFrameTransformer.cpp
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#include "RobotFrameTransformer.h"

#include <yarp/os/Log.h>
#include <yarp/dev/IEncoders.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/yarp/YARPConversions.h>


namespace human
{
    RobotFrameTransformer::RobotFrameTransformer(const iDynTree::Transform &constTransformFromFixture,
                                                 const std::string &robotArm,
                                                 const std::string &robotSole,
                                                 const std::string &humanFoot,
                                                 const std::string &humanHand,
                                                 yarp::dev::IEncoders &robotJointConfiguration_encoder,
                                                 yarp::os::BufferedPort<HumanState> &humanJointConfiguration_port)
    : GenericFrameTransformer(robotArm,humanHand)
    , m_constTransformFromFixture(constTransformFromFixture)
    , m_robotSole(robotSole)
    , m_humanFoot(humanFoot)
    , m_humanHand(humanHand)
    , m_robotJointConfiguration_encoder(robotJointConfiguration_encoder)
    , m_humanJointConfiguration_port(humanJointConfiguration_port){}
    
    
    bool RobotFrameTransformer::init(const iDynTree::Model &humanModel,
                                     const iDynTree::Model &robotModel)
    {
        if (!m_humanComputations.loadRobotModel(humanModel))
        {
            yError("Error in reading the human model!");
            return false;
        }
        
        m_humanConfiguration.resize(m_humanComputations.getNrOfDegreesOfFreedom());
        m_humanVelocity.resize(m_humanComputations.getNrOfDegreesOfFreedom());
        m_humanVelocity.zero();
        
        if (!m_robotComputations.loadRobotModel(robotModel))
        {
            yError("Error in reading the robot model!");
            return false;
        }
        
        m_robotConfiguration.resize(m_robotComputations.getNrOfDegreesOfFreedom());
        m_robotVelocity.resize(m_robotComputations.getNrOfDegreesOfFreedom());
        m_robotVelocity.zero();
        
        return true;
        
    }
    
    
    bool RobotFrameTransformer::transformForceFrame(const yarp::sig::Vector &inputforce,
                                                    yarp::sig::Vector &transformedForce,
                                                    std::string &transformedExpressedFrame)
    {
        // UPDATE MATRIX
        
        iDynTree::Vector3 worldGravity;
        worldGravity.zero(); worldGravity(2) = -9.81;

        // -----read human configuration
        human::HumanState *tmp = m_humanJointConfiguration_port.read();
        
        // transform yarp Vector in iDynTree vector
        if (!iDynTree::toiDynTree(tmp->positions,m_humanConfiguration))
        {
            yError("Somenthing wrong in the convertion from YARP to iDynTree vector!");
            return false;
        };
        //TODO: read tmp->velocities.
        //Let's assume for the moment m_humanVelocity = 0;
        
        // -----set human configuration
        m_humanComputations.setRobotState(m_humanConfiguration,
                                          m_humanVelocity,
                                          worldGravity);
        
        // -----read robot configuration
        m_robotJointConfiguration_encoder.getEncoders(m_robotConfiguration.data());
        
        // -----set robot configuration
        m_robotComputations.setRobotState(m_robotConfiguration,
                                          m_robotVelocity,
                                          worldGravity);
        
        // non const transform
        iDynTree::Transform robotArm_T_leftSole;
        robotArm_T_leftSole = m_robotComputations.getRelativeTransform(m_robotArm, m_robotSole);
        
        // non const transform
        iDynTree::Transform humanLeftFoot_T_humanHand;
        humanLeftFoot_T_humanHand = m_humanComputations.getRelativeTransform(m_humanFoot, m_humanHand);
        
        iDynTree::Transform matrixTransform = (robotArm_T_leftSole * m_constTransformFromFixture *humanLeftFoot_T_humanHand).inverse();
        
        this->setTransform(matrixTransform);
        //matrixTransform is the matrix humanHand_T_robotArm that converts forces in robot frames
        //into forces in human ones.
        
        return GenericFrameTransformer::transformForceFrame(inputforce,
                                                            transformedForce,
                                                            transformedExpressedFrame);
    }
}