/*!
 * @file PortForceReader.cpp
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#include "PortForceReader.h"

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>


namespace human
{
    PortForceReader::PortForceReader(std::string attachedLink,
                                     std::string referenceFrame,
                                     yarp::os::BufferedPort<yarp::sig::Vector> &bufferedPort)
    : AbstractForceReader(attachedLink, referenceFrame)
    , m_bufferedPort(bufferedPort)
    , m_internal_buffer(6, 0.0){}
   
    
    bool PortForceReader::lowLevelRead(yarp::sig::Vector &force)
    {
        yarp::sig::Vector *tmp = m_bufferedPort.read(false);
        
        if(tmp != NULL && tmp->size() == 6)
        {
            m_internal_buffer = *tmp;
        }
        
        force = m_internal_buffer;
        return true;
    }
}
