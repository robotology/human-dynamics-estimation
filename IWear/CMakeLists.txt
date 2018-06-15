# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(IWear LANGUAGES CXX VERSION 0.1)

include(GNUInstallDirs)

# Group files with the correct path for the target_sources command
file(GLOB_RECURSE IWEAR_HEADERS_BUILD *.h)
file(GLOB_RECURSE IWEAR_HEADERS_INSTALL_TMP
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/../
    *.h)
foreach(HDR ${IWEAR_HEADERS_INSTALL_TMP})
    list(APPEND IWEAR_HEADERS_INSTALL "${CMAKE_INSTALL_INCLUDEDIR}/${HDR}")
endforeach()

set(IWEAR_HEADERS
    $<BUILD_INTERFACE:${IWEAR_HEADERS_BUILD}>
    $<INSTALL_INTERFACE:${IWEAR_HEADERS_INSTALL}>
)

add_library(${PROJECT_NAME} INTERFACE)
target_sources(${PROJECT_NAME} INTERFACE ${IWEAR_HEADERS})
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT IWear
)

if(MSVC)
    # Import definitions from standard cmath
    target_compile_definitions(${PROJECT_NAME} INTERFACE "_USE_MATH_DEFINES")
endif()

# Install all the directories preserving their structure
install(
    FILES IWear.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/IWear)
install(
    DIRECTORY Sensors
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/IWear)

# Generate the PackageConfig.cmake file
install_basic_package_files(IWear
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    TARGETS ${PROJECT_NAME}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
