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



/*! @brief Implementation of the FrameTransformer interface for the
 *  conversion of the robot forces into human forces. This class is in
 *  charge of converting the forces that are not constant during the
 *  physical human-robot interaction.  It computes at each timestamp
 *  an updating of the transform matrix by means of the human and robot
 *  joint configurations.
 */
class human::RobotFrameTransformer : public human::GenericFrameTransformer
{
private:
    iDynTree::Transform m_matrixTransform;
    iDynTree::Transform m_constTransformFromFixture;
    std::string m_robotLinkingFrame;
    std::string m_humanLinkingFrame;
    //mandatory indeces for on-line getRelativeTransform()
    iDynTree::FrameIndex m_originFrameIndex;
    iDynTree::FrameIndex m_robotLinkingFrameIndex;
    iDynTree::FrameIndex m_humanLinkingFrameIndex;
    iDynTree::FrameIndex m_transformedFrameIndex;
    
    
    iDynTree::KinDynComputations m_humanComputations;
    iDynTree::VectorDynSize m_humanConfiguration;
    iDynTree::VectorDynSize m_humanVelocity;
    
    iDynTree::KinDynComputations m_robotComputations;
    iDynTree::VectorDynSize m_robotConfiguration;
    iDynTree::VectorDynSize m_robotVelocity;
    
    yarp::dev::IEncoders &m_robotJointConfiguration_encoder;
    yarp::os::BufferedPort<HumanState> &m_humanJointConfigurationPort;
        
public:
    /*!
     * Constructor from a constant iDynTree Transform object,
     * constant strings for both human and robot parameters,
     * an encoder for the robot configuration and a buffered
     * YARP port for the human configuration.
     */
    RobotFrameTransformer(const iDynTree::Transform &constTransformFromFixture,
                          const std::string &inputFrame,
                          const std::string &robotLinkingFrame,
                          const std::string &humanLinkingFrame,
                          const std::string &outputFrame,
                          yarp::dev::IEncoders &robotJointConfiguration_encoder,
                          yarp::os::BufferedPort<HumanState> &humanJointConfigurationPort);
    /*!
     * Initialize the human and robot models.
     * @param[in] humanModel iDynTree model for the human
     * @param[in] robotModel idynTree model for the robot
     * @return true if the models initialization was successfull, false otherwise
     */
    bool init(const iDynTree::Model &humanModel,
              const iDynTree::Model &robotModel);
    
    virtual bool transformForceFrame(const yarp::sig::Vector &inputForce,
                                     yarp::sig::Vector &transformedForce,
                                     std::string &transformedExpressedFrame);
};

#endif /* HUMAN_ROBOTFRAMETRANSFORMER_H */
