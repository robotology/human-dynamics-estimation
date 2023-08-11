# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_add_idl(STRING_MSGS_FILES String.msg)
yarp_add_idl(GEOMETRY_MSGS_FILES
    geometry_msgs/Quaternion.msg
    geometry_msgs/Transform.msg
    geometry_msgs/TransformStamped.msg
    geometry_msgs/Vector3.msg
    geometry_msgs/Wrench.msg
    geometry_msgs/WrenchStamped.msg)
yarp_add_idl(SENSOR_MSGS_FILES
    sensor_msgs/JointState.msg
    sensor_msgs/Temperature.msg)
yarp_add_idl(STD_MSGS_FILES std_msgs/Header.msg)
yarp_add_idl(TF_MSGS_FILES tf/tfMessage.msg)
yarp_add_idl(TF2_MSGS_FILES tf2_msgs/TFMessage.msg)

add_library(HumanROSMsgs INTERFACE)

target_include_directories(HumanROSMsgs INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)
