#include <HDEReadOnlyControlBoardModule.h>

int main(int argc, char **argv) {

    yarp::os::Network yarp_network;
    HDEReadOnlyControlBoardModule module;
    yarp::os::ResourceFinder rf;

    rf.configure(argc,argv);

    if(!module.runModule(rf))
    {
        yError() << "HDEReadOnlyControlBoardModule: Failed to run the module";
        return 1;
    }

    return 0;
}
