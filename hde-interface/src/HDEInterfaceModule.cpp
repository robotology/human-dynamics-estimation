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

#include "HDEInterfaceModule.h"

double HDEInterfaceModule::getPeriod()
{
    return 1;
}

bool HDEInterfaceModule::updateModule()
{

    /*
    //Human-forces-provider 
    human::HumanForces *input_forces = forces_port.read();
    
    if(input_forces != NULL)
    {
        human::HumanForces::Editor input_forces_editor(*input_forces);

        std::vector<human::Force6D> forces6d_vec = input_forces_editor.get_forces();
        
        for(int i = 0; i < forces6d_vec.size(); i++)
        {            
            human::Force6D::Editor *force6d_editor;
            force6d_editor = new human::Force6D::Editor(forces6d_vec.at(i));
            
            if(force6d_editor->get_expressedFrame() == hde_ft_driver.getFTFrameName(i))
            {   
                int index = 6*(i+1);
                
                hde_ft_driver.setFTValues(force6d_editor->get_fx(),index-6);
                hde_ft_driver.setFTValues(force6d_editor->get_fy(),index-5);
                hde_ft_driver.setFTValues(force6d_editor->get_fz(),index-4);
            
                hde_ft_driver.setFTValues(force6d_editor->get_ux(),index-3);
                hde_ft_driver.setFTValues(force6d_editor->get_uy(),index-2);
                hde_ft_driver.setFTValues(force6d_editor->get_uz(),index-1);
                
            }
            else
            {
                yError() << "HDEInterfaceModule: FT frames do not match";
                return false;
            }
            delete force6d_editor;
            
        }
        
    }
    else
    {
        yError() << "HDEInterfaceModule: Failed to read forces port";
        return false;
    }

    */
    
    //Human-state-provider
    human::HumanState *input_state = state_port.read();

    if(input_state->positions.size() == hde_controlboard_driver.getNumberOfDofs())
    {
        hde_controlboard_driver.setJointPositionVec(input_state->positions);
        hde_controlboard_driver.setJointVelocityVec(input_state->velocities);
    }
    else
    {
        yError() << "HDEInterfaceModule: DoFs mismatch between the config file and human state port";
        return false;
    }
    
    //Human-dynamics-estimation
    human::HumanDynamics *input_dynamics = dynamics_port.read();
    
    std::vector<human::JointDynamicsEstimation> input_joint_dynamics = input_dynamics->jointVariables;
    
    if(input_joint_dynamics.size() == hde_controlboard_driver.getNumberOfDofs())
    {
        for(int j = 0; j < input_joint_dynamics.size(); j++)
        {
            if(input_joint_dynamics.at(j).jointName == hde_controlboard_driver.getJointName(j))
            {
                double joint_torque = input_joint_dynamics.at(j).torque[0];
                hde_controlboard_driver.setJointTorque(j,joint_torque);
                
            }
            else
            {
                yError() << "HDEInterfaceModule: Joint name mismatch while getting jonit torques";
                return false;
            }
        }
    }
    else
    {
        yError() << "HDEInterfaceModule: DoFs mismatch between config file and human joint dynamics";
        return false;
    }
        
    return true;
}

 bool HDEInterfaceModule::interruptModule()
 {
     yInfo() << "HDEInterfaceModule: Interrupting module for port cleanup";
     return true;
}

bool HDEInterfaceModule::close()
{
    yInfo() << "HDEInterfaceModule: Calling close function";
    hde_interface_rpc_port.close();
    return true;
}

