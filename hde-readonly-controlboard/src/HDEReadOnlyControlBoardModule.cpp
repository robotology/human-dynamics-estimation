#include "HDEReadOnlyControlBoardModule.h"

const std::string LogPrefix = "HDEReadOnlyControlBoardModule : ";

HDEReadOnlyControlBoardModule::HDEReadOnlyControlBoardModule(): iWrapper(0) {}

bool HDEReadOnlyControlBoardModule::configure(yarp::os::ResourceFinder& rf)
{
    if(!yarp::os::Network::initialized())
    {
        yarp::os::Network::init();
    }

    // ================
    // CHECK PARAMETERS
    // ================

    if (!rf.check("name")) {
        yError() << LogPrefix << "Module name is wrong or missing";
    }
    const std::string moduleName = rf.find("name").asString();
    rpc_port_name = "/" + moduleName + "/rpc:i";
    setName(moduleName.c_str());
    // Check that the name matches the module name in order to avoid
    // passing a wrong configuration file
    if (moduleName != "hde-readonly-controlboard") {
        yError() << LogPrefix
                 << "The moduleName parameter of the passed configuration is not hde-readonly-controlboard";
        return false;
    }

    // MODULE PARAMETERS
    // =================

    if (!(rf.check("period") && rf.find("period").isInt())) {
        yError() << LogPrefix << "Parameter 'period' missing or invalid";
        return false;
    }

    // HDE PARAMETERS
    // =================

    if (!(rf.check("hde_state_port_name"))) {
        yError() << LogPrefix << "Parameter 'hde_state_port_name' missing or invalid";
        return false;
    }

    if (!(rf.check("hde_dynamics_port_name"))) {
        yError() << LogPrefix << "Parameter 'hde_dynamics_port_name' missing or invalid";
        return false;
    }

    // HUMAN PARAMETERS
    // =================

    if (!(rf.check("human_joints") && rf.find("human_joints").isInt())) {
        yError() << LogPrefix << "Parameter 'joints' missing or invalid";
        return false;
    }

    if (!(rf.check("human_joints_name_list"))) {
        yError() << LogPrefix << "Parameter 'human_joints_name_list' missing or invalid";
        return false;
    }

    // HDEReadOnlyDriver PARAMETERS
    // =================

    if (!(rf.check("device"))) {
        yError() << LogPrefix << "Parameter 'device' missing or invalid";
        return false;
    }

    // ControlBoard WRAPPER PARAMETERS
    // =================
    if (!(rf.check("WRAPPER"))) {
        yError() << LogPrefix << "Parameter '[WRAPPER]' missing or invalid";
        return false;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    // MODULE PARAMETERS
    period = rf.find("period").asInt() / 1000.0;
    autoconnect = rf.find("autoconnect").asBool();

    // HDE PARAMETERS
    hde_state_port_name = rf.find("hde_state_port_name").asString();
    hde_dynamics_port_name = rf.find("hde_dynamics_port_name").asString();

    // HUMAN PARAMETERS
    human_dofs = rf.find("human_joints").asInt();
    joints = rf.findGroup("human_joints_name_list");

    // HDEReadOnlyDriver PARAMETERS
    device_name = rf.find("device").asString();

    // ControlBoard WRAPPER PARAMETERS
    wrapper_properties = rf.findGroup("WRAPPER");

    // =================================
    // INITIALIZE AND CONNECT YARP PORTS
    // =================================

    if(rpc_port.open(rpc_port_name))
    {
        attach(rpc_port);
    }

    if(state_port.open("/"+moduleName+"/state:i"))
    {
        if(!yarp::os::Network::connect(hde_state_port_name,state_port.getName().c_str()))
        {
            yError() << LogPrefix << "Failed to connect " << hde_state_port_name << " and " << state_port.getName().c_str();
            return false;
        }
    }
    else
    {
        yError() << LogPrefix << "Failed to open " << state_port.getName().c_str();
        return false;
    }

    if(dynamics_port.open("/"+moduleName+"/dynamicsEstimation:i"))
    {
        if(!yarp::os::Network::connect(hde_dynamics_port_name,dynamics_port.getName().c_str()))
        {
            yError() << LogPrefix << "Failed to connect " << hde_dynamics_port_name << " and " << dynamics_port.getName().c_str();
            return false;
        }
    }
    else
    {
        yError() << LogPrefix << "Failed to open " << dynamics_port.getName().c_str();
        return false;
    }

    // ============================
    // OPEN HDEReadOnlyDriver
    // ============================

    // Add HDEReadOnlyDriver to drivers factory
    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::HDEReadOnlyDriver>("hde_readonly_driver", "controlboardwrapper2", "HDEReadOnlyDriver"));

    // Open HDEReadOnlyDriver
    driver_parameters.put("device",device_name);
    hde_readonly_driver.open(driver_parameters);

    // Create HDEReadOnlyDriver pointer
    yarp::dev::HDEReadOnlyDriver* hde_readonly_driver_ptr = dynamic_cast<yarp::dev::HDEReadOnlyDriver*>(hde_readonly_driver.getImplementation());

    // ==========================
    // INITIALIZE JOINT VARIABLES
    // ==========================
    hde_readonly_driver_ptr->number_of_dofs = human_dofs;

    hde_readonly_driver_ptr->joint_positions_rad.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_velocities_rad.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_accelerations_rad.resize(hde_readonly_driver_ptr->number_of_dofs);

    hde_readonly_driver_ptr->joint_positions.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_velocities.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_accelerations.resize(hde_readonly_driver_ptr->number_of_dofs);
    hde_readonly_driver_ptr->joint_torques.resize(hde_readonly_driver_ptr->number_of_dofs);

    // ==========================
    // READ HUMAN JOINT NAME LIST
    // ==========================
    if(!joints.check("human_joints_name_list"))
    {
        yError() << LogPrefix << "Failed to read human joints name list";
        return false;
    }
    else
    {
        if(joints.size()-1 != hde_readonly_driver_ptr->number_of_dofs)
        {
            yError() << LogPrefix << "Mismatch between the joints number and joint name list size from the config file";
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

    // =========================
    // OPEN CONTROLBOARD WRAPPER
    // =========================
    wrapper.open(wrapper_properties);

    if(!wrapper.view(iWrapper))
    {
        yError() << LogPrefix << "Error while loading the wrapper";
        return false;
    }

    // ================================================
    // ATTACH HDEReadOnlyDriver TO CONTROLBOARD WRAPPER
    // ================================================
    driver_list.push(&hde_readonly_driver,"HDE");

    if(!iWrapper->attachAll(driver_list))
    {
        yError() << LogPrefix << "Error while attaching the device to the wrapper interface";
        return false;
    }

    return true;
}

bool HDEReadOnlyControlBoardModule::updateModule()
{
    // Create HDEReadOnlyDriver pointer
    yarp::dev::HDEReadOnlyDriver* hde_readonly_driver_ptr = dynamic_cast<yarp::dev::HDEReadOnlyDriver*>(hde_readonly_driver.getImplementation());

    //READ Human-state-provider
    human::HumanState *input_state = state_port.read();

    if(input_state->positions.size() == hde_readonly_driver_ptr->number_of_dofs)
    {
        // Read human joint positions(radians) vector
        hde_readonly_driver_ptr->joint_positions_rad = input_state->positions;

        // Read human joint velocities(radians) vector
        hde_readonly_driver_ptr->joint_velocities_rad = input_state->velocities;
    }
    else
    {
        yError() << LogPrefix << "DoFs mismatch between the config file and human state port";
        return false;
    }

    //READ Human-dynamics-estimation
    human::HumanDynamics *input_dynamics = dynamics_port.read();

    std::vector<human::JointDynamicsEstimation> input_joint_dynamics = input_dynamics->jointVariables;

    bool joint_present = false;

    if (input_joint_dynamics.size() == hde_readonly_driver_ptr->number_of_dofs)
    {
        for (int j = 0; j < input_joint_dynamics.size(); j++)
        {
            for (int joint_index = 0; joint_index < input_joint_dynamics.size(); joint_index++ )
            {
                if (input_joint_dynamics.at(joint_index).jointName == hde_readonly_driver_ptr->joint_name_list.at(j)) //TODO Update this logic
                {

                    joint_present = true;

                    // Read single human joint acceleration(radians)
                    double joint_acceleration = input_joint_dynamics.at(joint_index).acceleration[0];
                    hde_readonly_driver_ptr->joint_accelerations_rad[j] = joint_acceleration;

                    // Read single human joint torque
                    double joint_torque = input_joint_dynamics.at(joint_index).torque[0];
                    hde_readonly_driver_ptr->joint_torques[j] = joint_torque;

                }

            }
            if(!joint_present)
            {
                yError() << LogPrefix << "Joint not found while getting jonit torques";
                return false;
            }
            else {joint_present =  false;}
        }
    }
    else {
        yError() << LogPrefix << "DoFs mismatch between config file and human joint dynamics";
        return false;
    }


    // JOINT ANGLES RAD2DEG CONVERSION
    for(int j=1; j <= hde_readonly_driver_ptr->number_of_dofs; j++)
    {
        hde_readonly_driver_ptr->joint_positions[j] = hde_readonly_driver_ptr->joint_positions_rad[j]*(180/M_PI);
        hde_readonly_driver_ptr->joint_velocities[j] = hde_readonly_driver_ptr->joint_velocities_rad[j]*(180/M_PI);
        hde_readonly_driver_ptr->joint_accelerations[j] = hde_readonly_driver_ptr->joint_accelerations_rad[j]*(180/M_PI);
    }

    // DEBUG PRINTOUT
    //yInfo() << "Joint Positions: " << hde_readonly_driver_ptr->joint_positions.toString();
    //yInfo() << "Joint Velocities:: " << hde_readonly_driver_ptr->joint_velocities.toString();
    //yInfo() << "Joint Accelerations: " << hde_readonly_driver_ptr->joint_accelerations.toString();
    //yInfo() << "Joint Torques: " << hde_readonly_driver_ptr->joint_torques.toString();

    return true;
}

double HDEReadOnlyControlBoardModule::getPeriod()
{
    return period;
}

bool HDEReadOnlyControlBoardModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    if (command.get(0).asString()=="quit")
        return false;
    else
        reply=command;

    return true;
}

 bool HDEReadOnlyControlBoardModule::interruptModule()
 {
     yInfo() << LogPrefix << "Interrupting module for port cleanup";

     // Interrupt yarp ports
     rpc_port.interrupt();
     state_port.interrupt();
     dynamics_port.interrupt();

     // Disconnect yarp ports
     yarp::os::Network::disconnect(hde_state_port_name,state_port.getName().c_str());
     yarp::os::Network::disconnect(hde_dynamics_port_name,dynamics_port.getName().c_str());

     return true;
}

bool HDEReadOnlyControlBoardModule::close()
{
    yInfo() << LogPrefix << "Calling close function";

    // Close yarp ports
    rpc_port.close();
    state_port.close();
    dynamics_port.close();

    return true;
}

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
