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

#include "hdedriver.h"

using namespace yarp::dev;

const unsigned ForceTorqueChannelsNumber = 6;

int HDEDriver::read(yarp::sig::Vector& out)
{
    if(force_torque_vector.size() != number_of_channels)
    {
        yError() << "HDEDriver: ForceTorqueVector size is not the same as the total number of channels";
        return AS_ERROR;
    }
    
    if(out.size() != number_of_channels)
    {
        out.resize(number_of_channels);
    }
    
    data_mutex.wait();
    for(int n = 0; n < number_of_channels; n++)
    {
        out[n] = force_torque_vector[n];
    }
    data_mutex.post();
    
    return AS_OK;
}

bool HDEDriver::close()
{
    return true;
}

int HDEDriver::getChannels()
{
    return number_of_channels;
}

int HDEDriver::getState(int)
{
    return AS_OK;
}

int HDEDriver::calibrateSensor()
{
    return AS_OK;
}

int HDEDriver::calibrateSensor(const yarp::sig::Vector&)
{
    return AS_OK;
}

int HDEDriver::calibrateChannel(int)
{
    return AS_OK;
}

int HDEDriver::calibrateChannel(int, double)
{
    return AS_OK;
}

