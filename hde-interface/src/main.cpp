#include <hdeinterfacemodule.h>

int main(int argc, char **argv) {
    
    yarp::os::Network yarp_network;
    HDEInterfaceModule module;
    yarp::os::ResourceFinder rf;
    
    rf.configure(argc,argv);
    
    if(!module.runModule(rf))
    {
        yError() << "HDEInterfaceModule: Failed to run the module";
        return 1;
    }
    
    return 0;
}
