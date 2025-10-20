# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(human_dynamics_nws_yarp
    TYPE hde::servers::HumanDynamics_nws_yarp
    INCLUDE HumanDynamics_nws_yarp.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanDynamics_nws_yarp
    HumanDynamics_nws_yarp.cpp
    HumanDynamics_nws_yarp.h)

target_include_directories(HumanDynamics_nws_yarp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanDynamics_nws_yarp PUBLIC
    IHumanDynamics
    trintrin::msgs
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS HumanDynamics_nws_yarp
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
