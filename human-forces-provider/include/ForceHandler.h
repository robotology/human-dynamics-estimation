//
//  ForceHandler.h
//  HumanDynamicsEstimation
//
//  Created by Claudia Latella on 28/02/17.
//
//

/* FORCEHANDLER.H is a base class mainly used for implementing code for both the 
 * derived classes <FTForceReader> and <PortForceReader>.
 */

#ifndef HUMAN_FORCEHANDLER_H
#define HUMAN_FORCEHANDLER_H

#include "ForceReader.h"

#include <yarp/sig/Vector.h>

namespace human
{
    class ForceHandler;
    class Force6D;
    class FrameTransformer;
}

namespace yarp
{
    namespace sig
    {
        class Vector;
    }
}


class human::ForceHandler : public human::ForceReader
{
private:
    FrameTransformer *m_frameTransformer;
    std::string m_attachedLink;
    std::string m_referenceFrame;
    yarp::sig::Vector m_packedForce;
    
protected:
    virtual bool lowLevelRead(yarp::sig::Vector& force) = 0;
    
public:
    ForceHandler(std::string attachedLink,
                 std::string referenceFrame);
    
    virtual void setTransformer(FrameTransformer *frameTransformer);
    virtual FrameTransformer* getTransformer();
    
    virtual bool readForce(Force6D &readForce);
};

#endif /* HUMAN_FORCEHANDLER_H */
