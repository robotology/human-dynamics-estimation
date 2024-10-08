# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


yarp_prepare_plugin(ianalogsensor_to_iwear
    TYPE wearable::devices::IAnalogSensorToIWear
    INCLUDE include/IAnalogSensorToIWear.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(IAnalogSensorToIWear
    src/IAnalogSensorToIWear.cpp
    include/IAnalogSensorToIWear.h)

target_include_directories(IAnalogSensorToIWear PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(IAnalogSensorToIWear PUBLIC
    IWear::IWear YARP::YARP_dev YARP::YARP_init)

yarp_install(
    TARGETS IAnalogSensorToIWear
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
