# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


yarp_prepare_plugin(icub_wearable_device
    TYPE wearable::devices::ICub
    INCLUDE include/ICub.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(ICub
    src/ICub.cpp
    include/ICub.h)

target_include_directories(ICub PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(ICub
    PUBLIC
        IWear::IWear
        YARP::YARP_dev
        YARP::YARP_init
    PRIVATE
        iDynTree::idyntree-model
        iDynTree::idyntree-high-level)

yarp_install(
    TARGETS ICub
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
