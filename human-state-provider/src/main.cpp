//
//  main.cpp
//  human-state-provider
//
//  Created by Francesco Romano on 20/02/17.
//  Copyright © 2017 Francesco Romano. All rights reserved.
//

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include "HumanStateProvider.h"


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
    rf.configure(argc, argv);
    
    // Configure the module
    HumanStateProvider module;
    return module.runModule(rf);
}
