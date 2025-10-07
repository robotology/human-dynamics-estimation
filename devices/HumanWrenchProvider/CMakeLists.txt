# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanWrenchProvider PUBLIC
    IHumanState
    IHumanWrench
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_sig
    YARP::YARP_init
    IWear::IWear
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-estimation
    iDynTree::idyntree-high-level)
target_link_libraries(HumanWrenchProvider PRIVATE Eigen3::Eigen)

yarp_install(
    TARGETS HumanWrenchProvider
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
