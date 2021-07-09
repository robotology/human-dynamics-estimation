# Copyright 2021 Istituto Italiano di Tecnologia (IIT)
# @author Kourosh Darvish <kourosh.darvish@iit.it>

# Finds the Sense Glove SDK
#
# This will define the following variables::
#
#   SenseGlove_FOUND         - True if the system has the Sense Glove SDK
#   SenseGlove_INCLUDE_DIR  - The include directory of the SDK
#   SenseGlove_LIBRARIES     - The name of the libraries to link against.
###############################

# Check Directory of the SenseGlove_DIR
if(NOT DEFINED ENV{SenseGlove_DIR})
  message( FATAL_ERROR "Environment variable {SenseGlove_DIR} is not defined." )
else()
  message(STATUS "Environment variable {SenseGlove_DIR}: $ENV{SenseGlove_DIR}" )
endif()

if(NOT DEFINED ENV{SenseGlove_INCLUDE_DIRS})
  message( FATAL_ERROR "Environment variable {SenseGlove_INCLUDE_DIRS} is not defined." )
else()
  message(STATUS "Environment variable {SenseGlove_INCLUDE_DIRS}: $ENV{SenseGlove_INCLUDE_DIRS}" )
endif()
##### Find SenseGlove #####

add_library(SenseGlove SHARED IMPORTED GLOBAL $ENV{SenseGlove_DIR}/libSGCoreCpp.so)
set_target_properties(SenseGlove PROPERTIES IMPORTED_LOCATION $ENV{SenseGlove_DIR}/libSGCoreCpp.so)
target_include_directories(SenseGlove INTERFACE $ENV{SenseGlove_INCLUDE_DIRS})

set(SenseGlove_FOUND TRUE)
