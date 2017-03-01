/*!
 * @file ConstFrameTransformation.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#ifndef HUMAN_CONSTFRAMETRANSFORMATION_H
#define HUMAN_CONSTFRAMETRANSFORMATION_H

#include "FrameTransformer.h"

#include <yarp/sig/Vector.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/MatrixFixSize.h>


namespace human
{
    class ConstFrameTransformation;
}

namespace iDynTree
{
    namespace Core
    {
        class Transform;
    }
}

/*! @brief Implementation of the FrameTransformer interface for transforming forces given a constant transformation matrix. */
class human::ConstFrameTransformation : public human::FrameTransformer
{
private:
    std::string m_transformedExpressedFrame;
    iDynTree::Matrix6x6 m_matrixTransformed;
    
public:
    /*!
     * Constructor from a frame and an iDynTree Transform object.
     */
    ConstFrameTransformation (const std::string &transformedExpressedFrame,
                              const iDynTree::Transform &matrixTransform);
    virtual bool transformForceFrame(const yarp::sig::Vector &inputforce,
                                     yarp::sig::Vector &transformedForce,
                                     std::string &tranformedExpressedFrame);
};


#endif /* HUMAN_CONSTFRAMETRANSFORMATION_H */
