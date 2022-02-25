# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)

set(PROJECT_VERSION "1.3.0")

set (WEARABLES_PROJECT_NAME Wearables)

project(${WEARABLES_PROJECT_NAME} VERSION ${PROJECT_VERSION}
                  LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Export all symbols in Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Add a postfix to Windows libraries compiled in debug
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

# Libraries type
if(MSVC)
    option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" OFF)
else()
    option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
endif()

# Control where binaries and libraries are placed in the build folder.
# This simplifies tests running in Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Enable RPATH support for installed binaries and libraries
include(AddInstallRPATHSupport)
add_install_rpath_support(LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"       # Libraries
                          BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"       # Binaries
                                   "${CMAKE_INSTALL_FULL_LIBDIR}/yarp"  # Plugins
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

include(InstallBasicPackageFiles)

# Tweak linker flags in Linux
if(UNIX AND NOT APPLE)
    get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
    if(${LINKER_BIN} STREQUAL "ld")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--unresolved-symbols=report-all")
    endif()
endif()

if(WIN32)
  #Add compile definition to suppress warning related to header <experimental/filesystem>
  add_compile_definitions(_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING)

  # Flag to enable XSensSuit wearable device
  option(ENABLE_XsensSuit "Flag that enables building XsensSuit wearable device" OFF)
  if(ENABLE_XsensSuit)
    find_package(XsensXME)
    if(NOT ${XsensXME_FOUND})
      message(FATAL_ERROR "XsensSuit device requires XSENS XME!")
    endif()
    add_subdirectory(XSensMVN)
  endif()
endif()

add_subdirectory(interfaces)
add_subdirectory(impl)

# Prepare YARP wrapper and devices
find_package(YARP 3.2 REQUIRED)
set(YARP_FORCE_DYNAMIC_PLUGINS ON)
yarp_configure_plugins_installation(wearable)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(msgs)
add_subdirectory(devices)
add_subdirectory(wrappers)
add_subdirectory(app)
add_subdirectory(modules)

# Flag to enable Paexo wearable device
option(ENABLE_Paexo "Flag that enables building Paexo wearable device" OFF)

# Flag to enable SenseGlove wearable device
option(ENABLE_HapticGlove "Flag that enables building Haptic Sense Glove wearable device" OFF)

# Flag to enable IWearFrameVisualizer module
option(ENABLE_FrameVisualizer "Flag that enables building IWearFrameVisualizer module" OFF)
