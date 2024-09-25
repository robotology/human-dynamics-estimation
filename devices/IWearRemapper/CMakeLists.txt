# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


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
   IWear::IWear
    Wearable::WearableData
    Wearable::SensorsImpl
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init)

yarp_install(
    TARGETS IWearRemapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
