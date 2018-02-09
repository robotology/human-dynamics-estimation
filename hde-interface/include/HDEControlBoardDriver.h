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
#include <yarp/dev/ControlBoardInterfaces.h>
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
    public yarp::dev::IAxisInfo,
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
        
        joint_name_list.at(0) = "jC1Head_rotx";
        joint_name_list.at(1) = "jC1Head_roty"; 
        joint_name_list.at(2) = "jC1Head_rotz";
        joint_name_list.at(3) = "jL1T12_rotx";
        joint_name_list.at(4) = "jL1T12_roty";
        joint_name_list.at(5) = "jL1T12_rotz";
        joint_name_list.at(6) = "jL4L3_rotx";
        joint_name_list.at(7) = "jL4L3_roty";
        joint_name_list.at(8) = "jL4L3_rotz";
        joint_name_list.at(9) = "jL5S1_rotx";
        joint_name_list.at(10) = "jL5S1_roty";
        joint_name_list.at(11) = "jL5S1_rotz";
        joint_name_list.at(12) = "jLeftAnkle_rotx";
        joint_name_list.at(13) = "jLeftAnkle_roty";
        joint_name_list.at(14) = "jLeftAnkle_rotz";
        joint_name_list.at(15) = "jLeftBallFoot_rotx";
        joint_name_list.at(16) = "jLeftBallFoot_roty";
        joint_name_list.at(17) = "jLeftBallFoot_rotz";
        joint_name_list.at(18) = "jLeftC7Shoulder_rotx";
        joint_name_list.at(19) = "jLeftC7Shoulder_roty";
        joint_name_list.at(20) = "jLeftC7Shoulder_rotz";
        joint_name_list.at(21) = "jLeftElbow_rotx";
        joint_name_list.at(22) = "jLeftElbow_roty";
        joint_name_list.at(23) = "jLeftElbow_rotz";
        joint_name_list.at(24) = "jLeftHip_rotx";
        joint_name_list.at(25) = "jLeftHip_roty";
        joint_name_list.at(26) = "jLeftHip_rotz";
        joint_name_list.at(27) = "jLeftKnee_rotx";
        joint_name_list.at(28) = "jLeftKnee_roty";
        joint_name_list.at(29) = "jLeftKnee_rotz";
        joint_name_list.at(30) = "jLeftShoulder_rotx";
        joint_name_list.at(31) = "jLeftShoulder_roty";
        joint_name_list.at(32) = "jLeftShoulder_rotz";
        joint_name_list.at(33) = "jLeftWrist_rotx";
        joint_name_list.at(34) = "jLeftWrist_roty";
        joint_name_list.at(35) = "jLeftWrist_rotz";
        joint_name_list.at(36) = "jRightAnkle_rotx";
        joint_name_list.at(37) = "jRightAnkle_roty";
        joint_name_list.at(38) = "jRightAnkle_rotz";
        joint_name_list.at(39) = "jRightBallFoot_rotx";
        joint_name_list.at(40) = "jRightBallFoot_roty";
        joint_name_list.at(41) = "jRightBallFoot_rotz";
        joint_name_list.at(42) = "jRightC7Shoulder_rotx";
        joint_name_list.at(43) = "jRightC7Shoulder_roty";
        joint_name_list.at(44) = "jRightC7Shoulder_rotz";
        joint_name_list.at(45) = "jRightElbow_rotx";
        joint_name_list.at(46) = "jRightElbow_roty";
        joint_name_list.at(47) = "jRightElbow_rotz";
        joint_name_list.at(48) = "jRightHip_rotx";
        joint_name_list.at(49) = "jRightHip_roty";
        joint_name_list.at(50) = "jRightHip_rotz";
        joint_name_list.at(51) = "jRightKnee_rotx";
        joint_name_list.at(52) = "jRightKnee_roty";
        joint_name_list.at(53) = "jRightKnee_rotz";
        joint_name_list.at(54) = "jRightShoulder_rotx";
        joint_name_list.at(55) = "jRightShoulder_roty";
        joint_name_list.at(56) = "jRightShoulder_rotz";
        joint_name_list.at(57) = "jRightWrist_rotx";
        joint_name_list.at(58) = "jRightWrist_roty";
        joint_name_list.at(59) = "jRightWrist_rotz";
        joint_name_list.at(60) = "jT1C7_rotx";
        joint_name_list.at(61) = "jT1C7_roty";
        joint_name_list.at(62) = "jT1C7_rotz";
        joint_name_list.at(63) = "jT9T8_rotx";
        joint_name_list.at(64) = "jT9T8_roty";
        joint_name_list.at(65) = "jT9T8_rotz";
        
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
    
    //Axis Info
    virtual bool getAxisName(int axis, yarp::os::ConstString& name);
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type);
    
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
