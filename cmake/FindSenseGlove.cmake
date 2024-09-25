# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# Finds the Sense Glove SDK
#
# This will define the following variables::
#
#   SenseGlove_FOUND         - True if the system has the Sense Glove SDK
#   SenseGlove               - The name of the libraries to link against.
###############################

# Check Directory of the SenseGlove_DIR
if(NOT DEFINED ENV{SenseGlove_DIR})
  message( FATAL_ERROR "Environment variable {SenseGlove_DIR} is not defined." )
else()
  message(STATUS "Environment variable {SenseGlove_DIR}: $ENV{SenseGlove_DIR}" )
endif()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(BUILD_TYPE "Debug")
else()
  set(BUILD_TYPE "Release")
endif()
message(STATUS "SenseGlove is linking to BUILD_TYPE: ${BUILD_TYPE}")


if(WIN32)
  file(GLOB SenseGlove_LIB $ENV{SenseGlove_DIR}/Core/SGCoreCpp/lib/win/${BUILD_TYPE}/SGCoreCpp.lib )
  set(LIB_TYPE "STATIC")
else()
  file(GLOB SenseGlove_LIB $ENV{SenseGlove_DIR}/Core/SGCoreCpp/lib/linux/${BUILD_TYPE}/libSGCoreCpp.so )
  set(LIB_TYPE "SHARED")
endif()

set(SenseGlove_INCLUDE_DIRS $ENV{SenseGlove_DIR}/Core/SGCoreCpp/incl )

message(STATUS "Variable {SenseGlove_LIB}: ${SenseGlove_LIB}" )
message(STATUS "variable {SenseGlove_INCLUDE_DIRS}: ${SenseGlove_INCLUDE_DIRS}" )

##### Find SenseGlove #####

add_library(SenseGlove ${LIB_TYPE} IMPORTED GLOBAL ${SenseGlove_LIB})
set_target_properties(SenseGlove PROPERTIES IMPORTED_LOCATION ${SenseGlove_LIB})
target_include_directories(SenseGlove INTERFACE ${SenseGlove_INCLUDE_DIRS})

set(SenseGlove_FOUND TRUE)
