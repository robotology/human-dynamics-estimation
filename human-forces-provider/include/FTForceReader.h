//
//  FTForceReader.h
//  HumanDynamicsEstimation
//
//  Created by Claudia Latella on 15/02/17.
//
//

#ifndef HUMAN_FTFORCEREADER_H
#define HUMAN_FTFORCEREADER_H

#include "ForceReader.h"
#include <yarp/sig/Vector.h>
#include <string>

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
                  
class human::FTForceReader : public human::ForceReader
{
private:
    std::string m_attachedLink;
    std::string m_referenceFrame;
    yarp::dev::IAnalogSensor &m_sensor;
    yarp::sig::Vector m_readForces;
    
public:
    FTForceReader(std::string attachedLink,
                  std::string referenceFrame,
                  yarp::dev::IAnalogSensor &sensor);
    
    virtual bool readForce(Force6D &readForce);
    
};


#endif /* HUMAN_FTFORCEREADER_H */
