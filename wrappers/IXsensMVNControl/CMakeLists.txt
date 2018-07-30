# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_prepare_plugin(ixsensmvncontrol_wrapper
    TYPE wearable::wrappers::IXsensMVNControlWrapper
    INCLUDE IXsensMVNControlWrapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(IXsensMVNControlWrapper
    src/IXsensMVNControlWrapper.cpp
    include/IXsensMVNControlWrapper.h)

target_include_directories(IXsensMVNControlWrapper PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(IXsensMVNControlWrapper PUBLIC
    XsensSuitControl IXsensMVNControl YARP::YARP_dev)

yarp_install(
    TARGETS IXsensMVNControlWrapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/ixsensmvncontrol_wrapper.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
