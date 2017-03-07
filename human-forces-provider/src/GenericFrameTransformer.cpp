/*!
 * @file GenericFrameTransformer.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#include <GenericFrameTransformer.h>

#include <yarp/sig/Vector.h>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/EigenHelpers.h>


namespace human
{
    GenericFrameTransformer::GenericFrameTransformer (const std::string &originExpressedFrame,
                                                      const std::string &transformedExpressedFrame)
    : m_transformedExpressedFrame(originExpressedFrame)
    , m_originExpressedFrame(transformedExpressedFrame)
    , m_matrixTransform(iDynTree::Transform()) {}
    
    
    void GenericFrameTransformer::setTransform(const iDynTree::Transform &matrixTransform)
    {
        m_matrixTransform = matrixTransform;
    }

    
    const std::string GenericFrameTransformer::getOriginFrame()
    {
        return m_originExpressedFrame;
    }
    
    
    const std::string GenericFrameTransformer::getTransformedFrame()
    {
        return m_transformedExpressedFrame;
    }
    
    const iDynTree::Transform& GenericFrameTransformer::getTransform()
    {
        return m_matrixTransform;
    }
    
    
    bool GenericFrameTransformer::transformForceFrame(const yarp::sig::Vector &inputforce,
                                                       yarp::sig::Vector &transformedForce,
                                                       std::string &transformedExpressedFrame)
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
        outputForceEigen = -1 * toEigen(m_matrixTransform.asAdjointTransformWrench()) * inputForceEigen;
        
        transformedExpressedFrame = m_transformedExpressedFrame;
        
        return true;
    }
}