#include "HDEReadOnlyDriver.h"

using namespace yarp::dev;

bool HDEReadOnlyDriver::getAxisName(int axis, yarp::os::ConstString& name)
{
    if (axis < 0 || static_cast<size_t>(axis) >= number_of_dofs) return false;

    name = yarp::os::ConstString(joint_name_list.at(axis));
    return true;
}

bool HDEReadOnlyDriver::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    return true;
}
