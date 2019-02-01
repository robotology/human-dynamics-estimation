#include "HumanControlBoard.h"

using namespace yarp::dev;

bool HumanControlBoard::getAxisName(int axis, yarp::os::ConstString& name)
{
    if (axis < 0 || static_cast<size_t>(axis) >= number_of_dofs) return false;

    name = yarp::os::ConstString(joint_name_list.at(axis));
    return true;
}

bool HumanControlBoard::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    return true;
}
