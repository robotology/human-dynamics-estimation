# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


yarp_prepare_plugin(iwearactuators_wrapper
    TYPE wearable::wrappers::IWearActuatorsWrapper
    INCLUDE include/IWearActuatorsWrapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(IWearActuatorsWrapper
    src/IWearActuatorsWrapper.cpp
    include/IWearActuatorsWrapper.h)

target_include_directories(IWearActuatorsWrapper PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(IWearActuatorsWrapper PUBLIC
    IWear WearableActuators YARP::YARP_dev YARP::YARP_os YARP::YARP_init)

yarp_install(
    TARGETS IWearActuatorsWrapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
