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
    , m_bufferedPort(bufferedPort){}
   
    
    bool PortForceReader::lowLevelRead(yarp::sig::Vector &force)
    {
        yarp::sig::Vector *tmp = m_bufferedPort.read();
        
        //check if the pointer is pointing to a null force OR the dimension is not 6
        if (tmp == NULL || tmp->size() != 6)
        {
            yError("Something wrong in the read force: check it!!");
            return false;
        }
        force = *tmp;
        return true;
    }
}
