# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_library(IWearableTargets INTERFACE)

find_package(iDynTree REQUIRED)
find_package(IWear REQUIRED)

target_sources(IWearableTargets INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hde/interfaces/IWearableTargets.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces/IWearableTargets.h>)

target_include_directories(IWearableTargets INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces>)

target_link_libraries(IWearableTargets INTERFACE
                      iDynTree::idyntree-core
                      IWear::IWear)

install(
    FILES hde/interfaces/IWearableTargets.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces)

