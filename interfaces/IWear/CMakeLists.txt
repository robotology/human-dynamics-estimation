# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(IWear LANGUAGES CXX VERSION ${PROJECT_VERSION})

include(GNUInstallDirs)

# ===============
# IWEAR INTERFACE
# ===============

# Policy to suppress An interface source of target "IWear" has a relative path warning
# Policy CMP0076 is not set: target_sources() command converts relative pathsto absolute.
cmake_policy(SET CMP0076 OLD)

# Group files with the correct path for the target_sources command
file(GLOB_RECURSE IWEAR_HEADERS_BUILD include/*.h)
file(GLOB_RECURSE IWEAR_HEADERS_INSTALL
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    *.h)

set(IWEAR_HEADERS
    $<BUILD_INTERFACE:${IWEAR_HEADERS_BUILD}>
    $<INSTALL_INTERFACE:${IWEAR_HEADERS_INSTALL}>)

add_library(${PROJECT_NAME} INTERFACE)
add_library(Wearable::IWear ALIAS IWear)
target_sources(${PROJECT_NAME} INTERFACE ${IWEAR_HEADERS})

target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT IWear)

if(MSVC)
    # Import definitions from standard cmath
    target_compile_definitions(${PROJECT_NAME} INTERFACE "_USE_MATH_DEFINES")
endif()

# Install all the directories preserving their structure
install(
    FILES include/Wearable/IWear/Common.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Wearable/IWear)
install(
    FILES include/Wearable/IWear/IWear.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Wearable/IWear)
install(
    FILES include/Wearable/IWear/Utils.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Wearable/IWear)
install(
    DIRECTORY include/Wearable/IWear/Sensors
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Wearable/IWear)
install(
    DIRECTORY include/Wearable/IWear/Actuators
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Wearable/IWear)

# Generate the PackageConfig.cmake file
install_basic_package_files(IWear
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT IWear
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
