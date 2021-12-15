# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

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

