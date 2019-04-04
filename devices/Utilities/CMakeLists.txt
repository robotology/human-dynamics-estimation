# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(iDynTree REQUIRED)

set(UTILITY_LIBRARY_NAME UtilityLibrary)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# set cpp files
set(${UTILITY_LIBRARY_NAME}_SRC
  Utils.cpp
  )

# set hpp files
set(${UTILITY_LIBRARY_NAME}_HDR
  Utils.hpp
  )

# add an executable to the project using the specified source files.
add_library(${UTILITY_LIBRARY_NAME} ${${UTILITY_LIBRARY_NAME}_SRC} ${${UTILITY_LIBRARY_NAME}_HDR})

# add include directories to the build.
target_include_directories(${UTILITY_LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${UTILITY_LIBRARY_NAME}
  iDynTree::idyntree-high-level
  YARP::YARP_OS)
