# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

project(Wear LANGUAGES CXX)
cmake_minimum_required(VERSION 3.5)

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

# Control where binaries and libraries are placed in the build folder.
# This simplifies tests running in Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

include(InstallBasicPackageFiles)

# Tweak linker flags in Linux
if(UNIX AND NOT APPLE)
    get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
    if(${LINKER_BIN} STREQUAL "ld")
        set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--unresolved-symbols=report-all")
    endif()
endif()

add_subdirectory(IWear)
if(WIN32)
  option(XSENS_MVN_USE_SDK "Build the driver and the wrapper for the real MVN system using the MVN SDK" ON)
  if(XSENS_MVN_USE_SDK)
    find_package(XsensXME REQUIRED)
    if(NOT ${XsensXME_FOUND})
      message("XSENS XME not found but option to use it is enabled.")
      return()
    else()
      add_subdirectory(XSensMVN)
      endif()
  endif()
endif()

#add_subdirectory(FTShoes)
#add_subdirectory(XSensSuit)
add_subdirectory(msgs)
