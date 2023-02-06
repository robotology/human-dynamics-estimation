# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# List the subdirectory
# http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()


set(EXE_TARGET_NAME IWearFrameVisualizerModule)

find_package(YARP REQUIRED)
find_package(iDynTree REQUIRED)

add_executable(${EXE_TARGET_NAME} src/main.cpp)

target_link_libraries(${EXE_TARGET_NAME} PUBLIC
    IWear
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    YARP::YARP_math
    iDynTree::idyntree-visualization
    )

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

file(GLOB TARGET_INI_FILES conf/*.ini)

install(FILES ${TARGET_INI_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${WEARABLES_PROJECT_NAME})




