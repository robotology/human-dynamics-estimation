# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(YARP COMPONENTS rosmsg REQUIRED)
find_package(iDynTree REQUIRED)

yarp_prepare_plugin(human_dynamics_publisher
    TYPE hde::publishers::HumanDynamicsPublisher
    INCLUDE HumanDynamicsPublisher.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanDynamicsPublisher
    HumanDynamicsPublisher.cpp
    HumanDynamicsPublisher.h)

target_include_directories(HumanDynamicsPublisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanDynamicsPublisher PUBLIC
    IHumanDynamics
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    YARP::YARP_rosmsg
    iDynTree::idyntree-modelio-urdf)

yarp_install(
    TARGETS HumanDynamicsPublisher
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/human_dynamics_publisher.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
