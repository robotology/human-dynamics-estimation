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

bool HDEControlBoardDriver::stop()
{
    return false;
}

bool HDEControlBoardDriver::stop(int j)
{
    return false;
}

bool HDEControlBoardDriver::getAxes(int *ax)
{
    if (!ax) return false;
    *ax = number_of_dofs;
    return true;
}

bool HDEControlBoardDriver::positionMove(int j, double ref)
{
    return false;
}

bool HDEControlBoardDriver::positionMove(const double *refs)
{
    return false;
}

bool HDEControlBoardDriver::setRefSpeed(int j, double sp)
{
    return false;
}

bool HDEControlBoardDriver::setRefSpeeds(const double* spds)
{
    return false;
}

bool HDEControlBoardDriver::getRefSpeed(int j, double* ref)
{
    return false;
}

bool HDEControlBoardDriver::getRefSpeeds(double* spds)
{
    return false;
}

bool HDEControlBoardDriver::relativeMove(int j, double delta)
{
    return false;
}

bool HDEControlBoardDriver::relativeMove(const double* deltas)
{
    return false;
}

bool HDEControlBoardDriver::checkMotionDone(int j, bool* flag)
{
    return false;
}

bool HDEControlBoardDriver::checkMotionDone(bool* flag)
{
    return false;
}