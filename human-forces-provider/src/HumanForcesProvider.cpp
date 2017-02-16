
//
//  HumanForcesProvider.cpp
//  HumanForcesProvider
//
//  Created by Claudia Latella on 14/02/17.
//  Copyright © 2017 Claudia Latella. All rights reserved.

#include "HumanForcesProvider.h"

#include "ForceReader.h"
#include "FTForceReader.h"

#include <yarp/os/LogStream.h>  //for using yError()
#include <yarp/dev/IAnalogSensor.h>

#include <iostream>


HumanForcesProvider::HumanForcesProvider()
: m_period(0.1) {}

HumanForcesProvider::~HumanForcesProvider() {}

//---------------------------------------------------------------------
double HumanForcesProvider::getPeriod()
{
    return m_period;
}

//---------------------------------------------------------------------
bool HumanForcesProvider::configure(yarp::os::ResourceFinder &rf)
{
    /*
     * ------Configure module name
     */
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("humanForceProvider"),
                                      "Checking module name").asString();
    setName(moduleName.c_str());
    
    
    /*
     * ------Configure module periodicty
     */
    int periodInMs = rf.check("period",
                              yarp::os::Value(100),
                              "Checking period in [ms]").asInt();
    m_period = periodInMs / 1000.0;
    
    
    /*
     * ------Configure force plates devices
     */
    std::string appliedLink_fp1 = rf.find("appliedLink_fp1").asString();
    std::string expressedFrame_fp1 = rf.find("expressedFrame_fp1").asString();
    
    std::string ft1LocalName = rf.check("ft1_localName",
                                        yarp::os::Value("/" + getName() + "/FTSensor1:i"),
                                        "Checking FP1 port local name").asString();
    
    
    
    std::string appliedLink_fp2 = rf.find("appliedLink_fp2").asString();
    std::string expressedFrame_fp2 = rf.find("expressedFrame_fp2").asString();
 
    std::string ft1RemoteName = rf.check("ft1_remoteName",
                                         yarp::os::Value("/amti/first/analog:o"),
                                         "Checking FP1 port remote name").asString();
    
    
    /*
     * ------Configure the polydriver [in AnalogSensorClient.cpp of github/YARP]
     */
    yarp::os::Property options;
    options.put("device", "analogsensorclient");
    
    // -----------------------SENSOR 1 : FP1-----------------------//
    options.put("local", ft1LocalName);          //local port name
    options.put("remote", ft1RemoteName);         //device port name where we connect to
    
    bool ok = m_forcePoly1.open(options);
    if (!ok)
    {
        yError("Error in opening FT1 Sensor device!");
        close();
        return false;
    }

    yarp::dev::IAnalogSensor *firstSensor = 0;
    if (!m_forcePoly1.view(firstSensor) || !firstSensor)
    {
        yError("Error in viewing IAnalogSensor1!");
        close();
        return false;
    }
    
    human::FTForceReader *forceReader1 = new human::FTForceReader(appliedLink_fp1,
                                                                  expressedFrame_fp1,
                                                                  *firstSensor);
    m_readers.push_back(forceReader1);
    
    // -----------------------SENSOR 2 : FP2-----------------------//
    
    std::string ft2LocalName = rf.check("ft2_localName",
                                        yarp::os::Value("/" + getName() + "/FTSensor2:i"),
                                        "Checking FP2 port local name").asString();
    std::string ft2RemoteName = rf.check("ft2_remoteName",
                                         yarp::os::Value("/amti/second/analog:o"),
                                         "Checking FP2 port remote name").asString();
    
    options.put("local" , ft2LocalName);          //local port name
    options.put("remote", ft2RemoteName);         //device port name where we connect to
    
    ok = m_forcePoly2.open(options);
    if (!ok)
    {
        yError("Error in opening FT2 Sensor device!");
        close();
        return false;
    }
    
    yarp::dev::IAnalogSensor *secondSensor = 0;
    if (!m_forcePoly2.view(secondSensor) || !secondSensor)
    {
        yError("Error in viewing IAnalogSensor2!");
        close();
        return false;
    }

    human::FTForceReader *forceReader2 = new human::FTForceReader(appliedLink_fp2,
                                                                  expressedFrame_fp2,
                                                                  *secondSensor);
    m_readers.push_back(forceReader2);
    

    /*
     * ------Open port for the module output (to the next module human-dynamics-estimation)
     */
    if (!m_outputPort.open("/"+ getName() + "/forces:o"))
    {
        yError() << "Unable to open port " << (getName() + "/forces:o");
        return false;
    }
    
    human::HumanForces &allForces = m_outputPort.prepare();
    allForces.forces.reserve(2);
    m_outputPort.unprepare();
    return true;
}

//---------------------------------------------------------------------
bool HumanForcesProvider::updateModule()
{
    human::HumanForces &allForces = m_outputPort.prepare();
    std::vector<human::Force6D> &forcesVector = allForces.forces;
    
    forcesVector.resize(m_readers.size());
    for (std::vector<human::ForceReader*>::iterator it(m_readers.begin());
         it != m_readers.end(); ++it) {
        unsigned index = std::distance(m_readers.begin(), it);
        (*it)->readForce(forcesVector[index]);
    }
    
    
    /*
     * ------Write data on output port
     */
    m_outputPort.write();
    return true;
}

//---------------------------------------------------------------------
/*
 * Close function, to perform cleanup.
 */
bool HumanForcesProvider::close()
{
    m_outputPort.close();
    
    for (std::vector<human::ForceReader*>::iterator it(m_readers.begin());
         it != m_readers.end(); ++it) {
        delete *it;
    }
    m_readers.clear();
    m_forcePoly2.close();
    m_forcePoly1.close();
    
    return true;
}
