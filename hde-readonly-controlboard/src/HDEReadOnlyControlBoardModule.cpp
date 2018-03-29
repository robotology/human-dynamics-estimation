#include "HDEReadOnlyControlBoardModule.h"

double HDEReadOnlyControlBoardModule::getPeriod()
{
    return 0.01;
}

HDEReadOnlyControlBoardModule::HDEReadOnlyControlBoardModule(): iWrapper(0) {}

HDEReadOnlyControlBoardModule::~HDEReadOnlyControlBoardModule()
{
    if(iWrapper)
    {
        iWrapper->detachAll();
        iWrapper = 0;
    }
    if(wrapper.isValid())
    {
        wrapper.close();
    }

    if(hde_readonly_driver.isValid())
    {
        hde_readonly_driver.close();
    }

    yarp::os::Network::fini();
}

bool HDEReadOnlyControlBoardModule::configure(yarp::os::ResourceFinder& rf)
{
    if(!yarp::os::Network::initialized())
    {
        yarp::os::Network::init();
    }

    std::string rpc_port_name = rf.find("name").asString();

    if(rpc_port.open(rpc_port_name+"/rpc:i"))
    {
        attach(rpc_port);
    }

    if(state_port.open(rpc_port_name+"/state:i"))
    {
        if(!yarp::os::Network::connect("/human-state-provider/state:o",state_port.getName().c_str()))
        {
            yError() << "HDEReadOnlyControlBoardModule: Failed to connect /human-state-provider/state:o and /hde-interface/state:i ports";
            return false;
        }
    }
    else
    {
        yError() << "HDEReadOnlyControlBoardModule: Failed to open /hde-interface/state:i port";
        return false;
    }

    if(forces_port.open(rpc_port_name+"/forces:i"))
    {
        if(!yarp::os::Network::connect("/human-forces-provider/forces:o",forces_port.getName().c_str()))
        {
            yError() << "HDEReadOnlyControlBoardModule: Failed to connect /human-forces-provider/forces:o and /hde-interface/forces:i ports";
            return false;
        }
    }
    else
    {
        yError() << "HDEReadOnlyControlBoardModule: Failed to open /hde-interface/forces:i port";
        return false;
    }

    if(dynamics_port.open(rpc_port_name+"/dynamicsEstimation:i"))
    {
        if(!yarp::os::Network::connect("/human-dynamics-estimator/dynamicsEstimation:o",dynamics_port.getName().c_str()))
        {
            yError() << "HDEReadOnlyControlBoardModule: Failed to connect /human-dynamics-estimator/dynamicsEstimation:o and /hde-interface/dynamicsEstimation:i ports";
            return false;
        }
    }
    else
    {
        yError() << "HDEReadOnlyControlBoardModule: Failed to open /hde-interface/dynamicsEstimation:i port";
        return false;
    }

    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::HDEReadOnlyDriver>("hde_readonly_driver", "controlboardwrapper2", "HDEReadOnlyDriver"));

    std::string device_name = rf.find("device").asString();
    driver_parameters.put("device",device_name);
    hde_readonly_driver.open(driver_parameters);

    yarp::dev::HDEReadOnlyDriver* hde_readonly_driver_ptr = dynamic_cast<yarp::dev::HDEReadOnlyDriver*>(hde_readonly_driver.getImplementation());

    hde_readonly_driver_ptr->number_of_dofs = rf.find("joints").asInt();

    hde_readonly_driver_ptr->joint_positions_rad.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_velocities_rad.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_accelerations_rad.resize(hde_readonly_driver_ptr->number_of_dofs);

    hde_readonly_driver_ptr->joint_positions.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_velocities.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_accelerations.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_torques.resize(hde_readonly_driver_ptr->number_of_dofs);


    yarp::os::Bottle joints = rf.findGroup("joint_name_list");

    if(!joints.check("joint_name_list"))
    {
        yError() << "HDEReadOnlyControlBoardModule: Failed to read joints name list";
        return false;
    }
    else
    {
        if(joints.size()-1 != hde_readonly_driver_ptr->number_of_dofs)
        {
            yError() << "HDEReadOnlyControlBoardModule: mismatch between the joints number and joint name list size from the config file";
            return false;

        }
        else
        {
            hde_readonly_driver_ptr->joint_name_list.resize(hde_readonly_driver_ptr->number_of_dofs);
            for(int i=0; i < hde_readonly_driver_ptr->number_of_dofs; i++)
            {
                hde_readonly_driver_ptr->joint_name_list.at(i) = joints.get(i+1).asString();
            }
        }
    }

    wrapper_properties = rf.findGroup("WRAPPER");
    wrapper.open(wrapper_properties);

    if(!wrapper.view(iWrapper))
    {
        yError() << "HDEReadOnlyControlBoardModule: Error while loading the wrapper";
        return false;
    }

    driver_list.push(&hde_readonly_driver,"HDE");

    if(!iWrapper->attachAll(driver_list))
    {
        yError() << "HDEReadOnlyControlBoardModule: Error while attaching the device to the wrapper interface";
        return false;
    }

    return true;
}

bool HDEReadOnlyControlBoardModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    if (command.get(0).asString()=="quit")
        return false;
    else
        reply=command;

    return true;
}

bool HDEReadOnlyControlBoardModule::updateModule()
{
    yarp::dev::HDEReadOnlyDriver* hde_readonly_driver_ptr = dynamic_cast<yarp::dev::HDEReadOnlyDriver*>(hde_readonly_driver.getImplementation());

    //Human-state-provider
    human::HumanState *input_state = state_port.read();

    if(input_state->positions.size() == hde_readonly_driver_ptr->number_of_dofs)
    {
        hde_readonly_driver_ptr->joint_positions_rad = input_state->positions;
        //yInfo() << "Joint Positions: " << hde_readonly_driver_ptr->joint_positions.toString();

        hde_readonly_driver_ptr->joint_velocities_rad = input_state->velocities;
        //yInfo() << "Joint Velocities:: " << hde_readonly_driver_ptr->joint_velocities.toString();
    }
    else
    {
        yError() << "HDEReadOnlyControlBoardModule: DoFs mismatch between the config file and human state port";
        return false;
    }

    //Human-dynamics-estimation
    human::HumanDynamics *input_dynamics = dynamics_port.read();

    std::vector<human::JointDynamicsEstimation> input_joint_dynamics = input_dynamics->jointVariables;

    if(input_joint_dynamics.size() == hde_readonly_driver_ptr->number_of_dofs)
    {
        for(int j = 0; j < input_joint_dynamics.size(); j++)
        {
            if(input_joint_dynamics.at(j).jointName == hde_readonly_driver_ptr->joint_name_list.at(j))
            {
                double joint_acceleration = input_joint_dynamics.at(j).acceleration[0];
                hde_readonly_driver_ptr->joint_accelerations_rad[j] = joint_acceleration;


                double joint_torque = input_joint_dynamics.at(j).torque[0];
                hde_readonly_driver_ptr->joint_torques[j] = joint_torque;

            }
            else
            {
                yError() << "HDEReadOnlyControlBoardModule: Joint name mismatch while getting jonit torques";
                return false;
            }
        }
    }
    else
    {
        yError() << "HDEReadOnlyControlBoardModule: DoFs mismatch between config file and human joint dynamics";
        return false;
    }

    for(int j=1; j <= hde_readonly_driver_ptr->number_of_dofs; j++)
    {
        hde_readonly_driver_ptr->joint_positions[j] = hde_readonly_driver_ptr->joint_positions_rad[j]*(180/M_PI);
        hde_readonly_driver_ptr->joint_velocities[j] = hde_readonly_driver_ptr->joint_velocities_rad[j]*(180/M_PI);
        hde_readonly_driver_ptr->joint_accelerations[j] = hde_readonly_driver_ptr->joint_accelerations_rad[j]*(180/M_PI);
    }

    //yInfo() << "Joint Accelerations: " << hde_readonly_driver_ptr->joint_accelerations.toString();
    //yInfo() << "Joint Torques: " << hde_readonly_driver_ptr->joint_torques.toString();

    return true;
}

 bool HDEReadOnlyControlBoardModule::interruptModule()
 {
     yInfo() << "HDEReadOnlyControlBoardModule: Interrupting module for port cleanup";
     return true;
}

bool HDEReadOnlyControlBoardModule::close()
{
    yInfo() << "HDEReadOnlyControlBoardModule: Calling close function";

    yarp::os::Network::disconnect("/human-state-provider/state:o",state_port.getName().c_str());
    yarp::os::Network::disconnect("/human-forces-provider/forces:o",forces_port.getName().c_str());
    yarp::os::Network::disconnect("/human-dynamics-estimator/dynamicsEstimation:o",dynamics_port.getName().c_str());

    rpc_port.close();
    return true;
}
