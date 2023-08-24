# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(YARP COMPONENTS dev REQUIRED)
find_package(robometry REQUIRED)

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
    IWear WearableData YARP::YARP_dev YARP::YARP_init robometry::robometry)

yarp_install(
    TARGETS IWearLogger
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
