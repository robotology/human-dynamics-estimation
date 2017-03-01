/*!
 * @file PortForceReader.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */

#ifndef HUMAN_PORTFORCEREADER_H
#define HUMAN_PORTFORCEREADER_H

#include "AbstractForceReader.h"

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

/*! @brief Implementation of the ForceReader interface for handling forces coming from a YARP port. */
class human::PortForceReader : public human::AbstractForceReader
{
private:
    yarp::os::BufferedPort<yarp::sig::Vector> &m_bufferedPort;
    
protected:
    /*!
     * Read a force coming from a port.
     */
    virtual bool lowLevelRead(yarp::sig::Vector &force);
    
public:
    /*!
     * Constructor from a link, a frame and a YARP port.
     */
    PortForceReader(std::string attachedLink,
                    std::string referenceFrame,
                    yarp::os::BufferedPort<yarp::sig::Vector> &bufferedPort);
    
};


#endif /* HUMAN_PORTFORCEREADER_H */
