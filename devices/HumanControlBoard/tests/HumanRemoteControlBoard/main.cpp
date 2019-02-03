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
    options.put("remote", "/Human/HumanControlBoardData");

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

    bool zero_check = true;

    while(true)
    {
        ienc->getEncoders(pos.data());
        ienc->getEncoderSpeeds(vel.data());
        ienc->getEncoderAccelerations(acc.data());
        itau->getTorques(tau.data());

        for(int i = 1; i <= jnts; i++)
        {
            if(pos[i] > 1e-10)
                zero_check = false;
        }

        if(zero_check == false)
        {
            yInfo() << "Read Position Values: " << pos.toString();


            yInfo() << "Read Velocity Values: " << vel.toString();
            //yInfo() << "Read Velocities Size: " << vel.size();


            yInfo() << "Read Acceleration Values: " << acc.toString();
            //yInfo() << "Read Accelerations Size: " << acc.size();


            yInfo() << "Read Torque Values: " << tau.toString();
            //yInfo() << "Read Torques Size: " << tau.size();
        }
    }
    device.close();

    return 0;
}

