# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_prepare_plugin(iwear_remapper
    TYPE wearable::devices::IWearRemapper
    INCLUDE include/IWearRemapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(IWearRemapper
    src/IWearRemapper.cpp
    include/IWearRemapper.h)

target_include_directories(IWearRemapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(IWearRemapper PUBLIC
    Wearable::IWear
    Wearable::WearableData
    Wearable::SensorsImpl
    YARP::YARP_dev
    YARP::YARP_OS
    YARP::YARP_init)

yarp_install(
    TARGETS IWearRemapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
