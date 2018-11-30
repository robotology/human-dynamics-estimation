# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(IWear REQUIRED)
find_package(iDynTree REQUIRED)

yarp_prepare_plugin(xsens_human_state_provider
    TYPE hde::devices::XsensHumanStateProvider
    INCLUDE XsensHumanStateProvider.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(XsensHumanStateProvider
    XsensHumanStateProvider.cpp
    XsensHumanStateProvider.h)

target_include_directories(XsensHumanStateProvider PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(XsensHumanStateProvider PUBLIC
    IHumanState
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    IWear::IWear
    iDynTree::idyntree-model
    iDynTree::idyntree-high-level)

yarp_install(
    TARGETS XsensHumanStateProvider
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/xsens_human_state_provider.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
