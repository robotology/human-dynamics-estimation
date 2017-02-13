//
//  main.cpp
//  HumanDynamicsEstimator
//
//  Created by Claudia Latella on 03/02/17.
//  Copyright © 2017 Claudia Latella. All rights reserved.
//

#include <iostream>
#include <yarp/os/LogStream.h>  //for using yError()
#include <yarp/os/Network.h>
#include "HumanDynamicsEstimator.h"

int main(int argc, char * argv[])
{
    // YARP setting
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(5.0))
    {
        yError() << " YARP server not available!";
        return EXIT_FAILURE;
    }
    
    // Configure ResourceFinder
    yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setVerbose(true);
    rf.setDefaultContext("HumanDynamicsEstimator"); //when no parameters are given to the module this is the default context
    rf.setDefaultConfigFile("HumanDynamicsEstimator.ini"); //default config file.ini -->to be done
    rf.configure(argc, argv);
    
    // Configure the module 
    HumanDynamicsEstimator module;
    std::cout<<"Configure & start module..."<<std::endl;
    return module.runModule(rf);
}

