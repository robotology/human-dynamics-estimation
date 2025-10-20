# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(human_state_nws_yarp
    TYPE hde::servers::HumanState_nws_yarp
    INCLUDE HumanState_nws_yarp.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanState_nws_yarp
    HumanState_nws_yarp.cpp
    HumanState_nws_yarp.h)

target_include_directories(HumanState_nws_yarp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanState_nws_yarp PUBLIC
    IHumanState
    trintrin::msgs
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS HumanState_nws_yarp
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
