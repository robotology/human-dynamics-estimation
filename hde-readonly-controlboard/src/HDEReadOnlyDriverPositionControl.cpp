#include "HDEReadOnlyDriver.h"

using namespace yarp::dev;

bool HDEReadOnlyDriver::stop()
{
    return false;
}

bool HDEReadOnlyDriver::stop(int j)
{
    return false;
}

bool HDEReadOnlyDriver::getAxes(int *ax)
{
    if (!ax) return false;
    *ax = number_of_dofs;
    return true;
}

bool HDEReadOnlyDriver::positionMove(int j, double ref)
{
    return false;
}

bool HDEReadOnlyDriver::positionMove(const double *refs)
{
    return false;
}

bool HDEReadOnlyDriver::setRefSpeed(int j, double sp)
{
    return false;
}

bool HDEReadOnlyDriver::setRefSpeeds(const double* spds)
{
    return false;
}

bool HDEReadOnlyDriver::getRefSpeed(int j, double* ref)
{
    return false;
}

bool HDEReadOnlyDriver::getRefSpeeds(double* spds)
{
    return false;
}

bool HDEReadOnlyDriver::relativeMove(int j, double delta)
{
    return false;
}

bool HDEReadOnlyDriver::relativeMove(const double* deltas)
{
    return false;
}

bool HDEReadOnlyDriver::checkMotionDone(int j, bool* flag)
{
    return false;
}

bool HDEReadOnlyDriver::checkMotionDone(bool* flag)
{
    return false;
}
