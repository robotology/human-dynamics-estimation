//
//  FTForceReader.h
//  HumanDynamicsEstimation
//
//  Created by Claudia Latella on 15/02/17.
//
//

/* FTFORCEREADER.H is an implementation of the interface <ForceReader.h> 
 * for reading forces coming from devices.
 * This is useful because the forces read by this class are then handled by 
 * a polydriver.
 */

#ifndef HUMAN_FTFORCEREADER_H
#define HUMAN_FTFORCEREADER_H

#include "ForceHandler.h"

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


class human::FTForceReader : public human::ForceHandler
{
private:
    yarp::dev::IAnalogSensor &m_sensor;
    
protected:
    virtual bool lowLevelRead(yarp::sig::Vector& force);
    
public:
    FTForceReader(std::string attachedLink,
                  std::string referenceFrame,
                  yarp::dev::IAnalogSensor &sensor);
};


#endif /* HUMAN_FTFORCEREADER_H */
