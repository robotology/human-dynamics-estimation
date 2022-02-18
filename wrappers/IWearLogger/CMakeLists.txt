# Copyright (C) 2018-2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(YARP COMPONENTS telemetry dev QUIET)

if (${YARP_FOUND})

yarp_prepare_plugin(iwear_logger
    TYPE wearable::wrappers::IWearLogger
    INCLUDE include/IWearLogger.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(IWearLogger
    src/IWearLogger.cpp
    include/IWearLogger.h)

target_include_directories(IWearLogger PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(IWearLogger PUBLIC
    IWear WearableData YARP::YARP_dev YARP::YARP_init YARP::YARP_telemetry)

yarp_install(
    TARGETS IWearLogger
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

else()
    message(WARNING "YARP-telemetry is missing. IWearLogger will not be installed.")
endif()

