/*!
 * @file GenericFrameTransformer.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#ifndef HUMAN_GENERICFORCETRANSFORMER_H
#define HUMAN_GENERICFORCETRANSFORMER_H

#include "FrameTransformer.h"

#include <iDynTree/Core/Transform.h>


namespace human
{
    class GenericFrameTransformer;
}


/*! @brief 
 * Implementation of the FrameTransformer interface for the
 * conversion of the robot forces into human forces.
 * It applies a generic transform to a given force.
 */
class human::GenericFrameTransformer : public human::FrameTransformer
{
private:
    std::string m_originExpressedFrame;
    std::string m_transformedExpressedFrame;
    iDynTree::Transform m_matrixTransform;
    
public:
    /*!
     * Constructor from a frame and an iDynTree Transform object.
     */
    GenericFrameTransformer(const std::string &originExpressedFrame,
                            const std::string &transformedExpressedFrame);
    /*!
     * Get the origin frame name.
     */
    const std::string& getOriginFrame();

    /*!
     * Get the transformed frame name.
     */
    const std::string& getTransformedFrame();
    
    /*!
     * Set the frame transform.
     */
    void setTransform(const iDynTree::Transform &matrixTransform);
    /*!
     * Return the frame transform.
     */
    const iDynTree::Transform& getTransform();
    
    virtual bool transformForceFrame(const yarp::sig::Vector &inputForce,
                                     yarp::sig::Vector &transformedForce,
                                     std::string &transformedExpressedFrame);
};

#endif /* HUMAN_GENERICFORCETRANSFORMER_H */
