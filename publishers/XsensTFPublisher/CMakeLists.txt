# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(YARP COMPONENTS rosmsg REQUIRED)
find_package(IWear REQUIRED)
find_package(iDynTree REQUIRED)

yarp_prepare_plugin(xsens_tf_publisher
    TYPE hde::publishers::XsensTFPublisher
    INCLUDE XsensTFPublisher.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(XsensTFPublisher
    XsensTFPublisher.cpp
    XsensTFPublisher.h)

target_include_directories(XsensTFPublisher PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(XsensTFPublisher PRIVATE
    HumanROSMsgs
    IWear::IWear
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    YARP::YARP_math
    YARP::YARP_rosmsg
    iDynTree::idyntree-core)

yarp_install(
    TARGETS XsensTFPublisher
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/xsens_tf_publisher.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
