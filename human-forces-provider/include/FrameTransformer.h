/*!
 * @file ForceTransformer.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#ifndef HUMAN_FRAMETRANSFORMER_H
#define HUMAN_FRAMETRANSFORMER_H

#include <string>


namespace human
{
    class FrameTransformer;
}

namespace yarp
{
    namespace sig
    {
        template<class T> class VectorOf;
        typedef VectorOf<double> Vector;
    }
}


/*! 
 * @brief Interface for transforming forces into human frames.
 * In general, given two generic frames A and B, it basically implements the following transform:
 * \f[
 *  {}^Bf_B = -{}^B T_A {}^A f_B
 * \f]
 * where T is the adjoint transformation for the forces.
 *
 *  @note Multiply by -1 is mandatory since the force applied on the human is exactly the
 *  opposite of the one excerted on external sensor (both forceplates and robot).
 */
class human::FrameTransformer
{
public:
    /*!
     * Transform a force in a given frame.
     * @param[in]  inputForce force to be transformed
     * @param[out] transformedForce force transformed
     * @param[out] tranformedExpressedFrame frame in which the force is transformed
     * @return true if the transform was successfull, false otherwise
     *
     * @note The code considers both the case in which the transform matrix is constant
     * (in GenericFrameTransformer) and the case in which the transform is changing at
     * each timestamp (in RobotFrameTransformer).
     */
    virtual bool transformForceFrame(const yarp::sig::Vector &inputForce,
                                     yarp::sig::Vector &transformedForce,
                                     std::string &tranformedExpressedFrame) = 0;
    /*!
     * Destructor.
     */
    virtual ~FrameTransformer();
};

#endif /* HUMAN_FRAMETRANSFORMER_H */
