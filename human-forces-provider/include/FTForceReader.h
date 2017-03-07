/*!
 * @file FTForceReader.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#ifndef HUMAN_FTFORCEREADER_H
#define HUMAN_FTFORCEREADER_H

#include "AbstractForceReader.h"


namespace human
{
    class FTForceReader;
}

namespace yarp
{
    namespace dev
    {
        class IAnalogSensor;
    }
}


/*! @brief Implementation of the ForceReader interface for handling forces coming from devices. */
class human::FTForceReader : public human::AbstractForceReader
{
private:
    yarp::dev::IAnalogSensor &m_sensor;
    
protected:
    /*!
     * Read a force coming from a YARP interface for handlig polydriver
     * (see the documentation [here]( http://www.yarp.it/classyarp_1_1dev_1_1IAnalogSensor.html#details )).
     */
    virtual bool lowLevelRead(yarp::sig::Vector& force);
    
public:
    /*!
     * Constructor from a link, a frame and a sensor (useful for the polydriver).
     */
    FTForceReader(std::string attachedLink,
                  std::string referenceFrame,
                  yarp::dev::IAnalogSensor &sensor);
};

#endif /* HUMAN_FTFORCEREADER_H */