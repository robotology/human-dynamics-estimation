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

#include <hdedriver.h>
#include <human-forces-provider/thrifts/HumanForces.h>

class HDEInterfaceModule:public yarp::os::RFModule
{
    yarp::os::RpcServer hde_interface_rpc_port;
    yarp::os::BufferedPort<yarp::os::Bottle> state_port;
    yarp::os::BufferedPort<human::HumanForces> forces_port;
    yarp::os::BufferedPort<yarp::os::Bottle> dynamics_port;
    
    yarp::dev::HDEDriver hde_driver;
    
public:
    
    bool configure(yarp::os::ResourceFinder& rf)
    {
        if(!yarp::os::Network::initialized())
            yarp::os::Network::init();
        
        if(hde_interface_rpc_port.open("/hde-interface/rpc:i"))
            attach(hde_interface_rpc_port);
        
        if(state_port.open("/hde-interface/state:i"))
        {
            if(!yarp::os::Network::connect("/human-state-provider/state:o",state_port.getName().c_str()))
            {
                yError() << "HDEInterfaceModule: Failed to connect /human-state-provider/state:o and /hde-interface/state:i ports";
                return false;
            }
        }
        else
        {
            yError() << "HDEInterfaceModule: Failed to open /hde-interface/state:i port";
            return false;
        }
        
        if(forces_port.open("/hde-interface/forces:i"))
        {
            if(!yarp::os::Network::connect("/human-forces-provider/forces:o",forces_port.getName().c_str()))
            {
                yError() << "HDEInterfaceModule: Failed to connect /human-forces-provider/forces:o and /hde-interface/forces:i ports";
                return false;
            }
        }
        else
        {
            yError() << "HDEInterfaceModule: Failed to open /hde-interface/forces:i port";
            return false;
        }
        
        if(dynamics_port.open("/hde-interface/dynamicsEstimation:i"))
        {
            if(!yarp::os::Network::connect("/human-dynamics-estimator/dynamicsEstimation:o",dynamics_port.getName().c_str()))
            {
                yError() << "HDEInterfaceModule: Failed to connect /human-dynamics-estimator/dynamicsEstimation:o and /hde-interface/dynamicsEstimation:i ports";
                return false;
            }
        }
        else
        {
            yError() << "HDEInterfaceModule: Failed to open /hde-interface/dynamicsEstimation:i port";
            return false;
        }
        
        hde_driver.open(rf);
        
        return true;
    }
    
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        if (command.get(0).asString()=="quit")
            return false;     
        else
            reply=command;
        
        return true;
    }
    
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();
    
};

#endif // HDEINTERFACEMODULE_H
