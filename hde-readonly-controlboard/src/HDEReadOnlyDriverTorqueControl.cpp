#include "HDEReadOnlyDriver.h"

using namespace yarp::dev;

bool HDEReadOnlyDriver::setRefTorque(int j, double t)
{
    return false;
}

bool HDEReadOnlyDriver::setRefTorques(const double* t)
{
    return false;
}

bool HDEReadOnlyDriver::setRefTorques(const int n_joint, const int *joints, const double *t)
{
    return false;
}

bool HDEReadOnlyDriver::setTorqueMode()
{
    return false;
}

bool HDEReadOnlyDriver::getRefTorque(int j, double* t)
{
    return false;
}

bool HDEReadOnlyDriver::getRefTorques(double* t)
{
    return false;
}

bool HDEReadOnlyDriver::getTorque(int j, double* t)
{
    if(t && j >= 0 && static_cast<size_t>(j) < number_of_dofs)
    {
        *t = joint_torques[j];
        return true;
    }
    else return false;
}

bool HDEReadOnlyDriver::getTorques(double* t)
{
    if (!t) return false;
    for(size_t j = 0; j < number_of_dofs; ++j)
    {
        t[j] = joint_torques[j];
    }
    return true;
}

bool HDEReadOnlyDriver::getTorqueRange(int, double*, double *)
{
    return false;
}

bool HDEReadOnlyDriver::getTorqueRanges(double *, double *)
{
    return false;
}

bool HDEReadOnlyDriver::getBemfParam(int , double *)
{
    return false;
}

bool HDEReadOnlyDriver::setBemfParam(int , double )
{
    return false;
}

bool HDEReadOnlyDriver::getMotorTorqueParams(int ,  yarp::dev::MotorTorqueParameters *)
{
    return false;
}

bool HDEReadOnlyDriver::setMotorTorqueParams(int , const yarp::dev::MotorTorqueParameters)
{
    return false;
}
