# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(HumanDynamicsEstimation CXX)

# =====================
# PROJECT CONFIGURATION
# =====================

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(BUILD_DOCS)
    add_subdirectory(doc)
    return()
endif()

include(GNUInstallDirs)
include(CTest)

option(BUILD_TESTING "Build tests" OFF)

# Find packages needed for this main CMakeLists.txt.
# Subfolders will find the dependencies they need.
find_package(YARP 3.0 REQUIRED)
find_package(YCM REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(iDynTree REQUIRED)

# Import cmake utilities
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(AddWarningsConfigurationToTarget)

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Shared/Dynamic or Static library?
if(MSVC)
    option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" OFF)
else()
    option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Plugins are force to be Shared/Dynamic Library 
set(YARP_FORCE_DYNAMIC_PLUGINS ON)

# Control where binaries and libraries are placed in the build folder.
# This simplifies tests running in Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(NOT MSVC)
    option(HDE_ENABLE_RPATH "Enable rpath" TRUE)
    mark_as_advanced(HDE_ENABLE_RPATH)
    include(AddInstallRPATHSupport)
    add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                              LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                              DEPENDS HDE_ENABLE_RPATH
                              USE_LINK_PATH)
endif()

# Advanced Eigen assertions
# TODO: this is for the old IK and HDE. Is it necessary for HDE?
option(HUMAN_DYNAMICS_ESTIMATION_DEBUG_MALLOC "Enable Eigen assertions on malloc" OFF)

# Add the uninstall target
include(AddUninstallTarget)

# ===================
# PROJECT DIRECTORIES
# ===================

# add_subdirectory(ros-msgs)

#Add IK solver
#find_package(IPOPT)
#option(HDE_USE_IPOPT "Set if compile IPOPT dependent projects" ${IPOPT_FOUND})
#if (HDE_USE_IPOPT)
#  add_subdirectory(inverse-kinematics)
#endif()

<<<<<<< HEAD
=======
# add_subdirectory(human-state-provider)
# add_subdirectory(human-forces-provider)
# add_subdirectory(human-dynamics-estimator)
# add_subdirectory(human-viz-bridge)
#add_subdirectory(human-robot-pose)

>>>>>>> remove dependency from robotology-playground/xsens-mvn
add_subdirectory(app)

add_subdirectory(yarp)

# HDE v2:
yarp_configure_plugins_installation(hde)
add_subdirectory(msgs)
add_subdirectory(interfaces)
add_subdirectory(devices)
add_subdirectory(wrappers)
add_subdirectory(publishers)
