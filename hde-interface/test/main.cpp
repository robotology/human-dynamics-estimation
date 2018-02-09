#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

int main(int argc, const char **argv)
{
    Network yarp;
    IEncoders *ienc;
    IPositionControl *ipos;
    ITorqueControl *itau;
    int jnts;

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/motortest");
    options.put("remote", "/human/hde");

    PolyDriver device;
    device.open(options);
    
    device.view(ipos);
    device.view(ienc);
    device.view(itau);

    ipos->getAxes(&jnts);
    yInfo() << "Joints: " << jnts;

    Vector pos(jnts);
    Vector vel(jnts);
    Vector acc(jnts);
    Vector tau(jnts);

    while(true)
    {
        ienc->getEncoders(pos.data());
        //yInfo() << "Read Position Values: " << pos.toString();

        ienc->getEncoderSpeeds(vel.data());
        //yInfo() << "Read Velocity Values: " << vel.toString();
        //yInfo() << "Read Velocities Size: " << vel.size();

        ienc->getEncoderAccelerations(acc.data());
        //yInfo() << "Read Acceleration Values: " << acc.toString();
        //yInfo() << "Read Accelerations Size: " << acc.size();

        itau->getTorques(tau.data());
        //yInfo() << "Read Torque Values: " << tau.toString();
        //yInfo() << "Read Torques Size: " << tau.size();
    }
    device.close();

    return 0;
}

