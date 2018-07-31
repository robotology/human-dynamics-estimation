#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/ResourceFinder.h>

#include "HumanRobotPose.h"

#include <iostream>

int main(int argc, char* argv[])
{
    // Initialize YARP network
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(5.0)) {
        yError() << " YARP server not available!";
        return EXIT_FAILURE;
    }

    // Configure ResourceFinder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setVerbose(true);
    rf.setDefaultContext("human-dynamic-estimation");
    rf.setDefaultConfigFile("human-robot-pose.ini");
    rf.configure(argc, argv);

    // Configure the module
    HumanRobotPose module;
    module.runModule(rf);

    // Terminate YARP network
    yarp::os::Network::fini();

    return EXIT_SUCCESS;
}
