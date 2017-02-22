//
//  FTForceReader.cpp
//  HumanDynamicsEstimation
//
//  Created by Claudia Latella on 15/02/17.
//
//

#include "FTForceReader.h"

#include <yarp/dev/IAnalogSensor.h>

namespace human
{
    FTForceReader::FTForceReader(std::string attachedLink,
                                 std::string referenceFrame,
                                 yarp::dev::IAnalogSensor &sensor)
    : ForceHandler(attachedLink, referenceFrame)
    , m_sensor(sensor) {}
    
    bool FTForceReader::lowLevelRead(yarp::sig::Vector& force)
    {
        if (m_sensor.read(force) != yarp::dev::IAnalogSensor::AS_OK) return false;
        return true;
    }
}
