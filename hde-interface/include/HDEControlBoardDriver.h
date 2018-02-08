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
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IEncodersTimed.h>
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
    public yarp::dev::IPositionControl,
    public yarp::dev::IVelocityControl,
    public yarp::dev::IEncodersTimed,
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
        /*
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
            
        }*/
        
        number_of_dofs = 66;
        
        joint_name_list.resize(number_of_dofs);
        joint_zero_positions.resize(number_of_dofs);
        joint_positions.resize(number_of_dofs);
        joint_velocities.resize(number_of_dofs);
        joint_accelerations.resize(number_of_dofs);
        joint_torques.resize(number_of_dofs);
        
        return true;
    }
    
    int getNrOfDOFS()
    {
        return number_of_dofs;
    }

    bool close()
    {
        return yarp::dev::DeviceDriver::close();
    }
    
    //Position Control
    bool stop() override;
    bool stop(int j) override;
    bool getAxes(int *ax) override;
    bool positionMove(int j, double ref) override;
    bool positionMove(const double* refs) override;
    bool setRefSpeed(int j, double sp) override;
    bool setRefSpeeds(const double* spds) override;
    bool getRefSpeed(int j, double* ref) override;
    bool getRefSpeeds(double* spds) override;
    bool relativeMove(int j, double delta) override;
    bool relativeMove(const double* deltas) override;
    bool checkMotionDone(int j, bool* flag) override;
    bool checkMotionDone(bool* flag) override;
    
    //Velocity Control
    bool velocityMove(int j, double sp) override;
    bool velocityMove(const double* sp) override;
    bool setRefAcceleration(int j, double acc) override;
    bool setRefAccelerations(const double* accs) override;
    bool getRefAcceleration(int j, double* acc) override;
    bool getRefAccelerations(double* accs) override;
    
    //Encoders
    bool getEncoder(int j, double* v) override;
    bool getEncoders(double* encs) override;
    bool resetEncoder(int j) override;
    bool resetEncoders() override;
    bool setEncoder(int j, double val) override;
    bool setEncoders(const double* vals) override;
    bool getEncoderSpeed(int j, double* sp) override;
    bool getEncoderSpeeds(double* spds) override;
    bool getEncoderAcceleration(int j, double* spds) override;
    bool getEncoderAccelerations(double* accs) override;
    
    //Encoders Timed
    bool getEncoderTimed(int j, double* encs, double* time) override;
    bool getEncodersTimed(double* encs, double* time) override;
    
    //Torque Control
    bool setRefTorque(int j, double t) override;
    bool setRefTorques(const double *t) override;
    bool setTorqueMode();
    bool getRefTorque(int j, double *t) override;
    bool getRefTorques(double *t) override;
    bool setRefTorques(const int n_joint, const int *joints, const double *t) override;
    bool getTorque(int j, double *t) override;
    bool getTorques(double *t) override;

    bool getBemfParam(int j, double *bemf);
    bool setBemfParam(int j, double bemf);
    bool getTorqueRange(int j, double *min, double *max);
    bool getTorqueRanges(double *min, double *max);
    bool getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params);
    bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params);
    
};

#endif // HDEDCONTROLBOARDDRIVER_H
