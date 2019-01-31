# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(IWear REQUIRED)
find_package(iDynTree REQUIRED)

yarp_prepare_plugin(human_control_board
    TYPE hde::devices::HumanControlBoard
    INCLUDE HumanControlBoard.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanControlBoard
    HumanControlBoard.cpp
    HumanControlBoard.h)

target_include_directories(HumanControlBoard PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanControlBoard PUBLIC
    IHumanState
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    iDynTree::idyntree-model)

yarp_install(
    TARGETS HumanControlBoard
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/human_control_board.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})