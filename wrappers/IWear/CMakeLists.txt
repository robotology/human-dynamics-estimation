# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_prepare_plugin(iwear_wrapper
    TYPE wearable::wrappers::IWearWrapper
    INCLUDE IWearWrapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(IWearWrapper
    src/IWearWrapper.cpp
    include/IWearWrapper.h)

target_include_directories(IWearWrapper PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(IWearWrapper PUBLIC
    IWear WearableData YARP::YARP_dev)

yarp_install(
    TARGETS IWearWrapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
