/*!
 * @file main.cpp
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#include "HumanForcesProvider.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>


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
    rf.setDefaultContext("human-dynamic-estimation"); //when no parameters are given to the module this is the default context
    rf.setDefaultConfigFile("human-force-provider.ini"); //default config file.ini
    rf.configure(argc, argv);
    
    // Configure the module
    HumanForcesProvider module;
    return module.runModule(rf);
}
