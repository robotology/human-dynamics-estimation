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
        class Vector;
    }
}

/*! @brief Interface for transforming forces into human frames. */
class human::FrameTransformer
{
public:
    virtual bool transformForceFrame(const yarp::sig::Vector &inputforce,
                                     yarp::sig::Vector &transformedForce,
                                     std::string &tranformedExpressedFrame) = 0;
    /*!
     * Destructor.
     */
    virtual ~FrameTransformer();
};

#endif /* HUMAN_FRAMETRANSFORMER_H */
