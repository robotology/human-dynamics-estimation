# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


yarp_prepare_plugin(ixsensmvncontrol_wrapper
    TYPE wearable::wrappers::IXsensMVNControlWrapper
    INCLUDE include/IXsensMVNControlWrapper.h
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
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
