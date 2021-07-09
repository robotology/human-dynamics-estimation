# Copyright 2021 Istituto Italiano di Tecnologia (IIT)
# @author Kourosh Darvish <kourosh.darvish@iit.it>

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

set(SenseGlove_LIB_DIRS $ENV{SenseGlove_DIR}/Core/SGCoreCpp/lib/linux )
set(SenseGlove_INCLUDE_DIRS $ENV{SenseGlove_DIR}/Core/SGCoreCpp/incl )

message(STATUS "Variable {SenseGlove_LIB_DIRS}: ${SenseGlove_LIB_DIRS}" )
message(STATUS "variable {SenseGlove_INCLUDE_DIRS}: ${SenseGlove_INCLUDE_DIRS}" )

##### Find SenseGlove #####

add_library(SenseGlove SHARED IMPORTED GLOBAL ${SenseGlove_LIB_DIRS}/libSGCoreCpp.so)
set_target_properties(SenseGlove PROPERTIES IMPORTED_LOCATION ${SenseGlove_LIB_DIRS}/libSGCoreCpp.so)
target_include_directories(SenseGlove INTERFACE ${SenseGlove_INCLUDE_DIRS})

set(SenseGlove_FOUND TRUE)
