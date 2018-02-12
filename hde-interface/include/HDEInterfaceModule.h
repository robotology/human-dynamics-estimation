/*
 * Copyright (c) 2018, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef HDEINTERFACEMODULE_H
#define HDEINTERFACEMODULE_H

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

//#include <HDEForceTorqueDriver.h>
#include <HDEControlBoardDriver.h>
#include <human-forces-provider/thrifts/HumanForces.h>
#include <human-state-provider/thrifts/HumanState.h>
#include <human-state-provider/thrifts/HumanStateProviderService.h>
#include <human-dynamics-estimator/thrifts/HumanDynamics.h>

namespace yarp {
    namespace dev {
        class IMultipleWrapper;
    }
}

class HDEInterfaceModule:public yarp::os::RFModule
{
    yarp::os::RpcServer hde_interface_rpc_port;
    yarp::os::BufferedPort<human::HumanState> state_port;
    yarp::os::BufferedPort<human::HumanForces> forces_port;
    yarp::os::BufferedPort<human::HumanDynamics> dynamics_port;
    
    //yarp::dev::HDEForeceTorqueDriver hde_ft_driver;
    
public:
    
    yarp::dev::IMultipleWrapper* iWrapper;
    
    yarp::dev::PolyDriver wrapper;
    yarp::os::Property wrapper_parameters;
    
    yarp::dev::PolyDriver hde_controlboard_driver;
    yarp::os::Property driver_parameters;
    
    yarp::dev::PolyDriverList driver_list;
    
    HDEInterfaceModule();
    virtual ~HDEInterfaceModule();
    
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool configure(yarp::os::ResourceFinder& rf);
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();
    
};

#endif // HDEINTERFACEMODULE_H
