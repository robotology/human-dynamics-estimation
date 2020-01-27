# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(IWear REQUIRED)
find_package(iDynTree REQUIRED)
find_package(ICUB REQUIRED)

yarp_prepare_plugin(robot_position_controller
    TYPE hde::devices::RobotPositionController
    INCLUDE RobotPositionController.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(RobotPositionController
    RobotPositionController.cpp
    RobotPositionController.h)

target_include_directories(RobotPositionController PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

add_definitions(-D_USE_MATH_DEFINES)

target_link_libraries(RobotPositionController PUBLIC
    IHumanState
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    iDynTree::idyntree-model
    ctrlLib)

yarp_install(
    TARGETS RobotPositionController
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/robot_position_controller.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
