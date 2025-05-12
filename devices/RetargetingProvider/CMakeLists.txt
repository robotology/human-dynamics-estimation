# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(iDynTree REQUIRED)
find_package(OsqpEigen 0.4.0 REQUIRED)


yarp_prepare_plugin(retargeting_provider
    TYPE hde::devices::RetargetingProvider
    INCLUDE RetargetingProvider.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(RetargetingProvider
    RetargetingProvider.cpp
    RetargetingProvider.h)

target_include_directories(RetargetingProvider PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(RetargetingProvider PUBLIC
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_init
    IWear::IWear
    iDynTree::idyntree-model
    iDynTree::idyntree-inverse-kinematics
    iDynTree::idyntree-high-level
    OsqpEigen::OsqpEigen
    IHumanState
    IWearableTargets
    HumanDynamicsEstimation::utils
    HumanDynamicsEstimation::algorithms
    )

if(MSVC)
  target_compile_options(RetargetingProvider PRIVATE /bigobj)
endif()

yarp_install(
    TARGETS RetargetingProvider
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
