//
//  HumanForcesProvider.h
//  HumanForcesProvider
//
//  Created by Claudia Latella on 14/02/17.
//  Copyright © 2017 Claudia Latella. All rights reserved.

#ifndef HUMANFORCESPROVIDER_H
#define HUMANFORCESPROVIDER_H


#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <thrifts/HumanForces.h>
#include <yarp/dev/PolyDriver.h>


namespace human
{
    class HumanForces;
    class ForceReader;
}

namespace yarp
{
    namespace dev
    {
        class IAnalogSensor;
    }
}


class HumanForcesProvider : public yarp::os::RFModule {
    
private:
    double m_period;
    
    //buffered port/out from module <human-forces-provider>
    yarp::os::BufferedPort<human::HumanForces> m_outputPort;
    
    //polydriver
    yarp::dev::PolyDriver m_forcePoly1;
    yarp::dev::PolyDriver m_forcePoly2;
    
    std::vector<human::ForceReader*> m_readers;
    
public:
    HumanForcesProvider();
    virtual ~HumanForcesProvider();

    double getPeriod();
    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();
    bool close();
    
};

#endif /* end of include guard: HUMANFORCESPROVIDER_H */
