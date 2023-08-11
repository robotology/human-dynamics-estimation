# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(YARP COMPONENTS rosmsg REQUIRED)
find_package(iDynTree REQUIRED)

yarp_prepare_plugin(human_robot_pose_publisher
    TYPE hde::publishers::HumanRobotPosePublisher
    INCLUDE HumanRobotPosePublisher.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanRobotPosePublisher
    HumanRobotPosePublisher.cpp
    HumanRobotPosePublisher.h)

target_include_directories(HumanRobotPosePublisher PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanRobotPosePublisher PRIVATE
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    YARP::YARP_math
    YARP::YARP_rosmsg
    iDynTree::idyntree-core)

yarp_install(
    TARGETS HumanRobotPosePublisher
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/human_robot_pose_publisher.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

