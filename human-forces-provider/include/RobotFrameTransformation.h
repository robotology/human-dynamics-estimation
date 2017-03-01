/*!
 * @file RobotFrameTransformation.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */

/* ROBOTFRAMETRANSFORMATION.H is an implementation of the interface 
 * <FrameTransformer.h> that transforms forces read by the interface 
 * <ForceReader.h> from robot reference frames into the human ones. 
 * To do this it is necessary:
 * - the human configuration (coming from the human-state-provider module);
 * - the robot configuration for the joints that we need.
 */

#ifndef HUMAN_ROBOTFRAMETRANSFORMATION_H
#define HUMAN_ROBOTFRAMETRANSFORMATION_H

#include <FrameTransformer.h>

#include <iDynTree/Core/Transform.h>

namespace human
{
    class RobotFrameTeransformation;
}

namespace iDynTree
{
    namespace core
    {
        class Transform;
    }
}

/*! @brief Implementation of the FrameTransformer interface for transforming forces given non constant robot transformation matrices. */

class human::RobotFrameTeransformation : public human::FrameTransformer
{
private:
    std::string m_transformedExpressedFrame;
    iDynTree::Matrix6x6 m_matrixTransformed;
    
public:
    
};


#endif /* HUMAN_ROBOTFRAMETRANSFORMATION_H */
