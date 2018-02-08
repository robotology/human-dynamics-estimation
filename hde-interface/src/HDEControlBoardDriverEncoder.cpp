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

bool HDEControlBoardDriver::getEncoder(int j, double *v)
{
    if(v && j >= 0 && static_cast<std::size_t>(j) < number_of_dofs)
    {
        *v = joint_positions[j];
    }
    return true;
}

bool HDEControlBoardDriver::getEncoders(double *encs)
{
    if(!encs) return false;
    for(std::size_t i = 0; i < number_of_dofs; i++)
    {
        encs[i] = joint_positions[i];
    }
    return true;
}

bool HDEControlBoardDriver::resetEncoder(int j)
{
    return false;
}

bool HDEControlBoardDriver::resetEncoders()
{
    return false;
}

bool HDEControlBoardDriver::setEncoder(int j, double val)
{
    return false;
}

bool HDEControlBoardDriver::setEncoders(const  double *vals)
{
    return false;
}

bool HDEControlBoardDriver::getEncoderSpeed(int j, double *sp)
{
    return false;
}

bool HDEControlBoardDriver::getEncoderSpeeds(double *spds)
{
    return false;
}

bool HDEControlBoardDriver::getEncoderAcceleration(int j, double *spds)
{
    return false;
}

bool HDEControlBoardDriver::getEncoderAccelerations(double *accs)
{
    return false;
}

bool HDEControlBoardDriver::getEncoderTimed(int j, double* encs, double* time)
{
    return false;
}

bool HDEControlBoardDriver::getEncodersTimed(double* encs, double* time)
{
    return false;
}