# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

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
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_sig
    YARP::YARP_init
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-estimation)

if(MSVC)
  target_compile_options(HumanDynamicsEstimator PRIVATE /bigobj)
endif()

yarp_install(
    TARGETS HumanDynamicsEstimator
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
