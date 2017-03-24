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
        int result = m_sensor.read(force);
        if (result != yarp::dev::IAnalogSensor::AS_OK)
        {
            yWarning("AnalogSensor returned %d status", result);
            return false;
        }
        return true;
    }
}
