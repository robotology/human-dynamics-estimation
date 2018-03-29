#include "HDEReadOnlyDriver.h"

using namespace yarp::dev;

bool HDEReadOnlyDriver::getEncoder(int j, double *v)
{
    if(v && j >= 0 && static_cast<std::size_t>(j) < number_of_dofs)
    {
        *v = joint_positions[j];
    }
    return true;
}

bool HDEReadOnlyDriver::getEncoders(double *encs)
{
    if(!encs) return false;
    for(std::size_t i = 0; i < number_of_dofs; i++)
    {
        encs[i] = joint_positions[i];
    }
    return true;
}

bool HDEReadOnlyDriver::resetEncoder(int j)
{
    return false;
}

bool HDEReadOnlyDriver::resetEncoders()
{
    return false;
}

bool HDEReadOnlyDriver::setEncoder(int j, double val)
{
    return false;
}

bool HDEReadOnlyDriver::setEncoders(const  double *vals)
{
    return false;
}

bool HDEReadOnlyDriver::getEncoderSpeed(int j, double *sp)
{
    if (sp && j >= 0 && static_cast<size_t>(j) < number_of_dofs)
    {
        *sp = joint_velocities[j];
        return true;
    }
    return false;
}

bool HDEReadOnlyDriver::getEncoderSpeeds(double *spds)
{
    if (!spds) return false;
    for (size_t i = 0; i < number_of_dofs; ++i)
    {
        spds[i] = joint_velocities[i];
    }
    return true;
}

bool HDEReadOnlyDriver::getEncoderAcceleration(int j, double *spds)
{
    if (spds && j >= 0 && static_cast<size_t>(j) < number_of_dofs)
    {
        *spds = joint_accelerations[j];
        return true;
    }
    return false;
}

bool HDEReadOnlyDriver::getEncoderAccelerations(double *accs)
{
    if (!accs) return false;
    for (size_t i = 0; i < number_of_dofs; ++i)
    {
        accs[i] = joint_accelerations[i];
    }
    return true;
}

bool HDEReadOnlyDriver::getEncoderTimed(int j, double* encs, double* time)
{
    return false;
}

bool HDEReadOnlyDriver::getEncodersTimed(double* encs, double* time)
{
    return false;
}
