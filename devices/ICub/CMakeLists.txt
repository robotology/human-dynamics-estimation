# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_prepare_plugin(icub_device
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
    IWear YARP::YARP_dev)

yarp_install(
    TARGETS ICub
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})
