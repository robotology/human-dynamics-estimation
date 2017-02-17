//
//  FTForceReader.cpp
//  HumanDynamicsEstimation
//
//  Created by Claudia Latella on 15/02/17.
//
//

#include "FTForceReader.h"
#include <thrifts/Force6D.h>
#include <yarp/dev/IAnalogSensor.h>

namespace human {
    FTForceReader::FTForceReader(std::string attachedLink,
                                 std::string referenceFrame,
                                 yarp::dev::IAnalogSensor &sensor)
    : m_attachedLink(attachedLink)
    , m_referenceFrame(referenceFrame)
    , m_sensor(sensor)
    , m_readForces(6, 0.0) {}
    
    bool FTForceReader::readForce(Force6D &readForce)
    {
        readForce.appliedLink = m_attachedLink;
        readForce.expressedFrame = m_referenceFrame;

        if (m_sensor.read(m_readForces) != yarp::dev::IAnalogSensor::AS_OK) return false;
        
        readForce.fx = m_readForces(0);
        readForce.fy = m_readForces(1);
        readForce.fz = m_readForces(2);
        readForce.ux = m_readForces(3);
        readForce.uy = m_readForces(4);
        readForce.uz = m_readForces(5);

        
        return true;
    }
}
