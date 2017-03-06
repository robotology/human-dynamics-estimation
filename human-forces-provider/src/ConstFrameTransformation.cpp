/*!
 * @file ConstFrameTransformation.cpp
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */

#include "ConstFrameTransformation.h"

#include <yarp/sig/Vector.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/EigenHelpers.h>

namespace human
{
    ConstFrameTransformation::ConstFrameTransformation (const std::string &transformedExpressedFrame,
                                                        const iDynTree::Transform &matrixTransform)
    : m_transformedExpressedFrame(transformedExpressedFrame)
    , m_matrixTransformed(matrixTransform.asAdjointTransformWrench()){}

    
    bool ConstFrameTransformation::transformForceFrame(const yarp::sig::Vector &inputforce,
                                                       yarp::sig::Vector &transformedForce,
                                                       std::string &tranformedExpressedFrame)
    {
        using namespace iDynTree;
        using namespace Eigen;
        if (inputforce.size() < 6) return false;
        transformedForce.resize(6);
        
        //Eigen Map types on Vector of YARP
        Map<const VectorXd> inputForceEigen(inputforce.data(), 6);
        Map<VectorXd> outputForceEigen(transformedForce.data(), 6);
        
        //multiplied by -1 (as the force applied on the human is exactly the
        //opposite of the one excerted on the robot)
        outputForceEigen = -1 * toEigen(m_matrixTransformed) * inputForceEigen;
        
        tranformedExpressedFrame = m_transformedExpressedFrame;

        return true;
    }
}