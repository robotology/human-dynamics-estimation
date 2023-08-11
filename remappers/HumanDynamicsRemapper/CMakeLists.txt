# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(human_dynamics_remapper
    TYPE hde::devices::HumanDynamicsRemapper
    INCLUDE HumanDynamicsRemapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanDynamicsRemapper
    HumanDynamicsRemapper.cpp
    HumanDynamicsRemapper.h)

target_include_directories(HumanDynamicsRemapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanDynamicsRemapper PUBLIC
    IHumanDynamics
    HumanDynamicsMsg
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS HumanDynamicsRemapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
    
