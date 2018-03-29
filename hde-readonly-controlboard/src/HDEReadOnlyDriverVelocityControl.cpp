#include "HDEReadOnlyDriver.h"

using namespace yarp::dev;

bool HDEReadOnlyDriver::velocityMove(int j, double sp)
{
    return false;
}

bool HDEReadOnlyDriver::velocityMove(const double* sp)
{
    return false;
}

bool HDEReadOnlyDriver::setRefAcceleration(int j, double acc)
{
    return false;
}

bool HDEReadOnlyDriver::setRefAccelerations(const double* accs)
{
    return false;
}

bool HDEReadOnlyDriver::getRefAcceleration(int j, double* acc)
{
    return false;
}

bool HDEReadOnlyDriver::getRefAccelerations(double* accs)
{
    return false;
}
