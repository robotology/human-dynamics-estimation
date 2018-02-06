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

#include "HDEControlBoardDriver.h"

using namespace yarp::dev;

bool HDEControlBoardDriver::setRefTorque(int j, double t)
{
    return true;
}

bool HDEControlBoardDriver::setRefTorques(const double* t)
{
    return true;
}

bool HDEControlBoardDriver::setRefTorques(const int n_joint, const int *joints, const double *t)
{
    return true;
}

bool HDEControlBoardDriver::setTorqueMode()
{
    return true;
}

bool HDEControlBoardDriver::getRefTorque(int j, double* t)
{
    return true;
}

bool HDEControlBoardDriver::getRefTorques(double* t)
{
    return true;
}

bool HDEControlBoardDriver::getTorque(int j, double* t)
{
    if(t && j >= 0 && static_cast<size_t>(j) < number_of_dofs)
    {
        *t = joint_torques[j];
        return true;
    }
    else return false;
}

bool HDEControlBoardDriver::getTorques(double* t)
{
    if (!t) return false;
    for(size_t j = 0; j < number_of_dofs; ++j)
    {
        t[j] = joint_torques[j];
    }
    return true;
}

bool HDEControlBoardDriver::getTorqueRange(int, double*, double *)
{
    return false;
}

bool HDEControlBoardDriver::getTorqueRanges(double *, double *)
{
    return false;
}

bool HDEControlBoardDriver::getBemfParam(int , double *)
{
    return false;
}

bool HDEControlBoardDriver::setBemfParam(int , double )
{
    return false;
}

bool HDEControlBoardDriver::getMotorTorqueParams(int ,  yarp::dev::MotorTorqueParameters *)
{
    return false;
}

bool HDEControlBoardDriver::setMotorTorqueParams(int , const yarp::dev::MotorTorqueParameters)
{
    return false;
}

void HDEControlBoardDriver::setJointTorque(int& i,double tau)
{
    joint_torques[i] = tau;
}