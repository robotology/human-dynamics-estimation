//
//  FrameTransformer.h
//  HumanDynamicsEstimation
//
//  Created by Claudia Latella on 22/02/17.
//
//

/* FRAMETRANSFORMER.H is an interface for transforming forces from a defined 
 * frame with respect to another given one.
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


class human::FrameTransformer
{
public:
    virtual bool transformForceFrame(const yarp::sig::Vector &inputforce,
                                     yarp::sig::Vector &transformedForce,
                                     std::string &tranformedExpressedFrame) = 0;
    virtual ~FrameTransformer();
};

#endif /* HUMAN_FRAMETRANSFORMER_H */
