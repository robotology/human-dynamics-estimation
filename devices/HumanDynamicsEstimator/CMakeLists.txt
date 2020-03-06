# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(iDynTree REQUIRED)
find_package(Eigen3 REQUIRED)

yarp_prepare_plugin(human_dynamics_estimator
    TYPE hde::devices::HumanDynamicsEstimator
    INCLUDE HumanDynamicsEstimator.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanDynamicsEstimator
    HumanDynamicsEstimator.cpp
    HumanDynamicsEstimator.h)

target_include_directories(HumanDynamicsEstimator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(HumanDynamicsEstimator PUBLIC
    IHumanState
    IHumanWrench
    IHumanDynamics
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_sig
    YARP::YARP_init
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-estimation)

yarp_install(
    TARGETS HumanDynamicsEstimator
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/human_dynamics_estimator.ini"
    DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
