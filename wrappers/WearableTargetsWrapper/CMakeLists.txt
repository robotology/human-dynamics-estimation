# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(wearable_targets_wrapper
    TYPE hde::wrappers::WearableTargetsWrapper
    INCLUDE WearableTargetsWrapper.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(WearableTargetsWrapper
WearableTargetsWrapper.cpp
WearableTargetsWrapper.h)

target_include_directories(WearableTargetsWrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(WearableTargetsWrapper PUBLIC
    IWearableTargets
    WearableTargetsMsg
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS WearableTargetsWrapper
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
