# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_prepare_plugin(human_dynamics_wrapper
    TYPE hde::wrappers::HumanDynamicsWrapper
    INCLUDE HumanDynamicsWrapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanDynamicsWrapper
    HumanDynamicsWrapper.cpp
    HumanDynamicsWrapper.h)

target_include_directories(HumanDynamicsWrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanDynamicsWrapper PUBLIC
    IHumanDynamics
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS HumanDynamicsWrapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/human_dynamics_wrapper.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
