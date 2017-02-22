//
//  PortForceReader.h
//  HumanDynamicsEstimation
//
//  Created by Claudia Latella on 22/02/17.
//
//

/* PORTFORCEREADER.H is an implementation of the interface <ForceReader.h> 
 * for reading forces coming from a port.
 */

#ifndef HUMAN_PORTFORCEREADER_H
#define HUMAN_PORTFORCEREADER_H

#include "ForceHandler.h"

#include <yarp/sig/Vector.h>


namespace human
{
    class PortForceReader;
}

namespace yarp
{
    namespace os
    {
        template <typename T>
        class BufferedPort;
    }
}

class human::PortForceReader : public human::ForceHandler
{
private:
    yarp::os::BufferedPort<yarp::sig::Vector> &m_bufferedPort;
    yarp::sig::Vector m_readForces;
    
protected:
    virtual bool lowLevelRead(yarp::sig::Vector& force);
    
public:
    PortForceReader(std::string attachedLink,
                    std::string referenceFrame,
                    yarp::os::BufferedPort<yarp::sig::Vector> &bufferedPort);
    
};


#endif /* HUMAN_PORTFORCEREADER_H */
