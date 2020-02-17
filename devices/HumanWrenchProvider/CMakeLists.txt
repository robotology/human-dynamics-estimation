# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(IWear REQUIRED)
find_package(iDynTree REQUIRED)
find_package(Eigen3 REQUIRED)

yarp_prepare_plugin(human_wrench_provider
    TYPE hde::devices::HumanWrenchProvider
    INCLUDE HumanWrenchProvider.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanWrenchProvider
    HumanWrenchProvider.cpp
    HumanWrenchProvider.h
    WrenchFrameTransformers.cpp
    WrenchFrameTransformers.h)

target_include_directories(HumanWrenchProvider PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(HumanWrenchProvider PUBLIC
    IHumanState
    IHumanWrench
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_sig
    YARP::YARP_init
    IWear::IWear
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-high-level)

yarp_install(
    TARGETS HumanWrenchProvider
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/human_wrench_provider.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
