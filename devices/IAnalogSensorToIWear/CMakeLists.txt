# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_prepare_plugin(ianalogsensor_to_iwear
    TYPE wearable::devices::IAnalogSensorToIWear
    INCLUDE IAnalogSensorToIWear.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(IAnalogSensorToIWear
    src/IAnalogSensorToIWear.cpp
    include/IAnalogSensorToIWear.h)

target_include_directories(IAnalogSensorToIWear PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(IAnalogSensorToIWear PUBLIC
    Wearable::IWear YARP::YARP_dev)

yarp_install(
    TARGETS IAnalogSensorToIWear
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/ianalogsensor_to_iwear.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
