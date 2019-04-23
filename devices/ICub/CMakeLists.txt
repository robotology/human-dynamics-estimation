# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(iDynTree REQUIRED)

yarp_prepare_plugin(icub_wearable_device
    TYPE wearable::devices::ICub
    INCLUDE ICub.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(ICub
    src/ICub.cpp
    include/ICub.h)

target_include_directories(ICub PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(ICub PUBLIC
    Wearable::IWear
    YARP::YARP_dev
    YARP::YARP_init
    iDynTree::idyntree-model
    iDynTree::idyntree-high-level)

yarp_install(
    TARGETS ICub
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/icub_wearable_device.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

