/*!
 * @file GenericFrameTransformer.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#ifndef HUMAN_GENERICFORCETRANSFORMER_H
#define HUMAN_GENERICFORCETRANSFORMER_H

#include <FrameTransformer.h>

#include <iDynTree/core/Transform.h>


namespace human
{
    class GenericFrameTransformer;
}


/*! @brief Apply a generic transform to a given force.
 *
 * Given two generic frames A and B, it basically implements the following convertion:
 * \f[
 *  {}^Bf_B = -{}^B T_A {}^A f_B
 * \f]
 * where  is the adjoint transformation
 * for the wrenchces.
 *
 *  @note The multiplication by -1 is mandatory since the force applied on the human is exactly the
 *  opposite of the one excerted on external sensor (both forceplates and robot).
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
     * Get origin frame name.
     */
    const std::string getOriginFrame();

    /*!
     * Get transformed frame name.
     */
    const std::string getTransformedFrame();
    
    /*!
     * Set the frame transform.
     */
    void setTransform(const iDynTree::Transform &matrixTransform);
    /*!
     * Return the frame transform.
     */
    const iDynTree::Transform& getTransform();
    
    virtual bool transformForceFrame(const yarp::sig::Vector &inputforce,
                                     yarp::sig::Vector &transformedForce,
                                     std::string &transformedExpressedFrame);
};

#endif /* HUMAN_GENERICFORCETRANSFORMER_H */