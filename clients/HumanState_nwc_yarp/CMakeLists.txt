# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(human_state_nwc_yarp
    TYPE hde::devices::HumanState_nwc_yarp
    INCLUDE HumanState_nwc_yarp.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanState_nwc_yarp
    HumanState_nwc_yarp.cpp
    HumanState_nwc_yarp.h)

target_include_directories(HumanState_nwc_yarp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanState_nwc_yarp PUBLIC
    IHumanState
    HumanStateMsg
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS HumanState_nwc_yarp
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
