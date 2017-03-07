/*!
 * @file RobotFrameTransformer.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */

/*! @brief Apply a generic transform to a given force. It differs from 
 *  GenericFrameTransformer since at each module timestamp the transform 
 *  has to be updated and the conversion to be re-computed.
 *  To do this it is necessary:
 *  - the human configuration (coming from the human-state-provider module);
 *  - the robot configuration for the joints that we need.
 */


#ifndef HUMAN_ROBOTFRAMETRANSFORMER_H
#define HUMAN_ROBOTFRAMETRANSFORMER_H

#include "GenericFrameTransformer.h"

#include <yarp/os/BufferedPort.h>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <thrifts/HumanState.h>


namespace human
{
    class RobotFrameTransformer;
    class HumanState;
}

namespace yarp
{
    namespace dev
    {
        class IEncoders;
    }
}


class human::RobotFrameTransformer : public human::GenericFrameTransformer
{
private:
    iDynTree::Transform m_matrixTransform;
    
    iDynTree::KinDynComputations m_humanComputations;
    iDynTree::VectorDynSize m_humanConfiguration;
    iDynTree::VectorDynSize m_humanVelocity;
    
    iDynTree::KinDynComputations m_robotComputations;
    iDynTree::VectorDynSize m_robotConfiguration;
    iDynTree::VectorDynSize m_robotVelocity;
    
    // for transforms
    iDynTree::Transform m_constTransformFromFixture;
    std::string m_robotArm;
    std::string m_robotSole;
    std::string m_humanFoot;
    std::string m_humanHand;
    
    yarp::dev::IEncoders &m_robotJointConfiguration_encoder;
    yarp::os::BufferedPort<HumanState> &m_humanJointConfiguration_port;
    
public:
    RobotFrameTransformer(const iDynTree::Transform &constTransformFromFixture,
                          const std::string &robotArm,
                          const std::string &robotSole,
                          const std::string &humanFoot,
                          const std::string &humanHand,
                          yarp::dev::IEncoders &robotJointConfiguration_encoder,
                          yarp::os::BufferedPort<HumanState> &humanJointConfiguration_port);
    
    bool init(const iDynTree::Model &humanModel,
              const iDynTree::Model &robotModel);
    
    virtual bool transformForceFrame(const yarp::sig::Vector &inputforce,
                                     yarp::sig::Vector &transformedForce,
                                     std::string &transformedExpressedFrame);
};

#endif /* HUMAN_ROBOTFRAMETRANSFORMER_H */