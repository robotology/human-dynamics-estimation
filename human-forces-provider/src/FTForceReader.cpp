/*!
 * @file FTForceReader.cpp
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#include "FTForceReader.h"

#include <yarp/dev/IAnalogSensor.h>


namespace human
{
    FTForceReader::FTForceReader(std::string attachedLink,
                                 std::string referenceFrame,
                                 yarp::dev::IAnalogSensor &sensor)
    : AbstractForceReader(attachedLink, referenceFrame)
    , m_sensor(sensor) {}
    
    bool FTForceReader::lowLevelRead(yarp::sig::Vector& force)
    {
        if (m_sensor.read(force) != yarp::dev::IAnalogSensor::AS_OK) return false;
        return true;
    }
}
