# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(human_state_remapper
    TYPE hde::devices::HumanStateRemapper
    INCLUDE HumanStateRemapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanStateRemapper
    HumanStateRemapper.cpp
    HumanStateRemapper.h)

target_include_directories(HumanStateRemapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanStateRemapper PUBLIC
    IHumanState
    HumanStateMsg
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS HumanStateRemapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
