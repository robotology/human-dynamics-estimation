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

#include "hdeinterfacemodule.h"

double HDEInterfaceModule::getPeriod()
{
    return 1;
}

bool HDEInterfaceModule::updateModule()
{

    human::HumanForces *forces = forces_port.read();    
    
    if(forces != NULL)
    {
        yInfo() << "Forces " << forces->toString();
    }
    else yError() << "HDEInterfaceModule: Failed to read forces port";
    /*if(forces->size() != hde_driver.getTotalSensorsSize())
    {
        yError() << "HDEInterfaceModule: Total number of FT data does not match as defined in the config file";
        return false;
    }
    else
    {
        for(int i=0; i < forces->size(); i++)
        {
            if(forces->get(i+1).asString() == hde_driver.getFTFrameName(i))
            {
                yInfo() << forces->get(i+1).asString();
            }
        }
    }*/
    
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

