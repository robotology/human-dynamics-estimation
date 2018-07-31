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

#include <yarp/rosmsg/geometry_msgs/WrenchStamped.h>


namespace human
{
    class HumanForces;
    class ForceReader;
}

namespace yarp
{
    namespace os
    {
        class Node;

        template<typename T>
        class Publisher;
    }
    namespace dev
    {
        class IAnalogSensor;
        class IEncoders;
    }
}


class HumanForcesProvider : public yarp::os::RFModule
{    
private:
    double m_period;
    
    //buffered port:i from the <human-state-provider> module
    //for the human configuration (together with the robot joint configuration
    //it will be useful for converting the forces from the robot to the human frames.)
    bool m_humanConfigured;
    yarp::os::BufferedPort<human::HumanState> m_humanJointConfigurationPort;
    
    //polidriver for robot configuration handling
    bool m_robotConfigured;
    yarp::dev::PolyDriver m_robot;
    
    //buffered port:o from <human-forces-provider> module
    yarp::os::BufferedPort<human::HumanForces> m_outputPort;
    

    std::vector<human::ForceReader*> m_readers;
    std::vector<yarp::dev::PolyDriver*> m_drivers;
    std::vector<yarp::os::BufferedPort<yarp::sig::Vector>*> m_ports;

    yarp::os::Node *m_rosNode;
    std::string m_tfPrefix;
    std::vector<yarp::os::Publisher<yarp::rosmsg::geometry_msgs::WrenchStamped>*> m_topics;
    unsigned m_rosSequence;
    double m_rosForcesScale;

    yarp::os::BufferedPort<human::HumanState>* getHumanStatePort(const yarp::os::Searchable& config);
    bool getRobotEncodersInterface(const yarp::os::Searchable& config,
                                   const std::vector<std::string>& robot_jointList,
                                   yarp::dev::IEncoders *& encoders);
    
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
    double getPeriod() override;
    /*!
     * Module configuration by means of:
     * - generic parameters (name, periodicity)
     * - specific force plates parameters
     * - specific robot parameters
     */
    bool configure(yarp::os::ResourceFinder &rf) override;
    /*!
     * Module updating: at each timestamp it writes a transformed force on
     * an output YARP port.
     */
    bool updateModule() override;
    /*!
     * Close module, release allocated memory and perform cleanup.
     */
    bool close() override;
};

#endif /* end of include guard: HUMANFORCESPROVIDER_H */
