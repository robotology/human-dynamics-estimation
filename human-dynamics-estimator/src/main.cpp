//
//  main.cpp
//  HumanDynamicsEstimator
//
//  Created by Claudia Latella on 03/02/17.
//  Copyright © 2017 Claudia Latella. All rights reserved.
//

#include <yarp/os/LogStream.h>
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
    rf.setDefaultContext("human-dynamics-estimation");
    rf.setDefaultConfigFile("human-dynamics-estimator.ini");
    rf.configure(argc, argv);
    
    // Configure the module 
    HumanDynamicsEstimator module;
    return module.runModule(rf);
}
