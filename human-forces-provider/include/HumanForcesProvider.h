/*!
 * @file HumanForcesProvider.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#ifndef HUMANFORCESPROVIDER_H
#define HUMANFORCESPROVIDER_H

#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include <thrifts/HumanForces.h>
#include <thrifts/HumanState.h>


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


class HumanForcesProvider : public yarp::os::RFModule
{    
private:
    double m_period;
    
    //buffered port:i from the <human-state-provider> module
    //for the human configuration (together with the robot joint configuration
    //it will be useful for converting the forces from the robot to the human frames.)
    yarp::os::BufferedPort<human::HumanState> m_humanJointConfiguration_port;
    
    //polydriver for forceplates handling
    yarp::dev::PolyDriver m_forcePoly1;
    yarp::dev::PolyDriver m_forcePoly2;
    
    //polidriver for robot configuration handling
    yarp::dev::PolyDriver m_PolyRobot;
    
    //buffered port:i from the robot forces estimated in the two arms
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotLeftArmForce_port;
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotRightArmForce_port;
    
    //buffered port:o from <human-forces-provider> module
    yarp::os::BufferedPort<human::HumanForces> m_output_port;

    std::vector<human::ForceReader*> m_readers;
    
public:
    /*!
     * Default constructor.
     */
    HumanForcesProvider();
    /*!
     * Destructor.
     */
    virtual ~HumanForcesProvider();
    /*!
     * Return the module period.
     */
    double getPeriod();
    /*!
     * Module configuration by means of:
     * - generic parameters (name, periodicity)
     * - specific force plates parameters
     * - specific robot parameters
     */
    bool configure(yarp::os::ResourceFinder &rf);
    /*!
     * Module updating: at each timestamp it returns a transformed force.
     */
    bool updateModule();
    /*!
     * Close module and perform cleanup.
     */
    bool close();
};

#endif /* end of include guard: HUMANFORCESPROVIDER_H */