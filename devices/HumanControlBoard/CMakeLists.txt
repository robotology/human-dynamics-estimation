# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

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

add_definitions(-D_USE_MATH_DEFINES)

target_link_libraries(HumanControlBoard PUBLIC
    IHumanState
    IHumanDynamics
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    iDynTree::idyntree-model
    iDynTree::idyntree-estimation)

yarp_install(
    TARGETS HumanControlBoard
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
