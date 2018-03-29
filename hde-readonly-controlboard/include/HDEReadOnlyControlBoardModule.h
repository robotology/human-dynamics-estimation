#ifndef HDEREADONLYCONTROLBOARDMODULE_H
#define HDEREADONLYCONTROLBOARDMODULE_H

#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/dev/Wrapper.h>

#include <HDEReadOnlyDriver.h>
#include <human-forces-provider/thrifts/HumanForces.h>
#include <human-state-provider/thrifts/HumanState.h>
#include <human-state-provider/thrifts/HumanStateProviderService.h>
#include <human-dynamics-estimator/thrifts/HumanDynamics.h>

namespace yarp {
    namespace dev {
        class IMultipleWrapper;
    }
}

class HDEReadOnlyControlBoardModule:public yarp::os::RFModule
{

private:

    std::string rpc_port_name;
    std::string hde_state_port_name;
    std::string hde_forces_port_name;
    std::string hde_dynamics_port_name;

    yarp::os::RpcServer rpc_port;
    yarp::os::BufferedPort<human::HumanState> state_port;
    yarp::os::BufferedPort<human::HumanForces> forces_port;
    yarp::os::BufferedPort<human::HumanDynamics> dynamics_port;

public:

    yarp::dev::IMultipleWrapper* iWrapper;

    yarp::dev::PolyDriver wrapper;
    yarp::os::Bottle wrapper_properties;

    yarp::dev::PolyDriver hde_readonly_driver;
    yarp::os::Property driver_parameters;

    yarp::dev::PolyDriverList driver_list;

    HDEReadOnlyControlBoardModule();
    virtual ~HDEReadOnlyControlBoardModule();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool configure(yarp::os::ResourceFinder& rf);
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();

};

#endif // HDEREADONLYCONTROLBOARDMODULE_H
