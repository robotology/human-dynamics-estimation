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

#ifndef HDEDCONTROLBOARDDRIVER_H
#define HDEDCONTROLBOARDDRIVER_H

#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/ITorqueControl.h>

namespace yarp
{
    namespace dev
    {
        class HDEControlBoardDriver;
    }
}

class yarp::dev::HDEControlBoardDriver:
    public yarp::dev::DeviceDriver,
    public yarp::dev::IEncoders,
    public yarp::dev::ITorqueControl
{
    
private:
    
public:
    
    int number_of_dofs;
    std::vector<std::string> joint_name_list;
    
    yarp::sig::Vector joint_zero_positions;
    yarp::sig::Vector joint_positions;
    yarp::sig::Vector joint_velocities;
    yarp::sig::Vector joint_accelerations;
    
    yarp::sig::Vector joint_torques;
    
    HDEControlBoardDriver() {};
    ~HDEControlBoardDriver() {};

    //Device Driver
    bool open(yarp::os::Searchable& config)
    {
        number_of_dofs = config.find("number_of_dofs").asInt();
        
        yarp::os::Bottle joints = config.findGroup("joint_name_list");
        if(!joints.check("joint_name_list"))
        {
            yError() << "HDEControlBoardDriver: Failed to read joints name list";
            return false;
        }
        else
        {
            joint_name_list.resize(number_of_dofs);
            for(int i=0; i < number_of_dofs; i++)
            {
                joint_name_list.at(i) = joints.get(i+1).asString();
            }
            
            joint_zero_positions.resize(number_of_dofs);
            joint_positions.resize(number_of_dofs);
            joint_velocities.resize(number_of_dofs);
            joint_accelerations.resize(number_of_dofs);
            joint_torques.resize(number_of_dofs);
            
        }
        
        return true;
    }

    bool close()
    {
        return yarp::dev::DeviceDriver::close();
    }
    
    std::string getJointName(int& i)
    {
        return joint_name_list.at(i);
    }
    
    //Encoders
    virtual bool getAxes(int *ax);
    virtual bool getEncoder(int j, double* v);
    virtual bool getEncoders(double* encs);
    virtual bool resetEncoder(int j);
    virtual bool resetEncoders();
    virtual bool setEncoder(int j, double val);
    virtual bool setEncoders(const double* vals);
    virtual bool getEncoderSpeed(int j, double* sp);
    virtual bool getEncoderSpeeds(double* spds);
    virtual bool getEncoderAcceleration(int j, double* spds);
    virtual bool getEncoderAccelerations(double* accs);
    
    //Torque Control
    virtual bool setRefTorque(int j, double t);
    virtual bool setRefTorques(const double *t);
    virtual bool setTorqueMode();
    virtual bool getRefTorque(int j, double *t);
    virtual bool getRefTorques(double *t);
    virtual bool setRefTorques(const int n_joint, const int *joints, const double *t);
    virtual bool getTorque(int j, double *t);
    virtual bool getTorques(double *t);

    virtual bool getBemfParam(int j, double *bemf);
    virtual bool setBemfParam(int j, double bemf);
    virtual bool getTorqueRange(int j, double *min, double *max);
    virtual bool getTorqueRanges(double *min, double *max);
    virtual bool getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params);
    virtual bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params);
    
};

#endif // HDEDCONTROLBOARDDRIVER_H
