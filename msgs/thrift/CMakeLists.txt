# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# TODO: these libraries are enforced to be static. In order to let the devices
#       find them on Windows from the build directory, their Release / Debug
#       folder should be added to the PATH. However, this does not work as expected.

# ============
# WearableData
# ============

yarp_add_idl(WEARABLEDATA_FILES WearableData.thrift)

add_library(WearableData ${WEARABLEDATA_FILES} WearableData.thrift)
add_library(Wearable::WearableData ALIAS WearableData)
target_link_libraries(WearableData YARP::YARP_OS)

# Extract the include directory from the files names
foreach(file ${WEARABLEDATA_FILES})
    STRING(REGEX MATCH ".+\\.h?h$" file ${file})
    if(file)
        get_filename_component(include_dir ${file} DIRECTORY)
        list(APPEND WEARABLEDATA_INCLUDE_DIRS ${include_dir})
        list(REMOVE_DUPLICATES WEARABLEDATA_INCLUDE_DIRS)
    endif()
endforeach()

foreach(dir ${WEARABLEDATA_INCLUDE_DIRS})
    get_filename_component(parent_dir_name ${dir} NAME)
    if(${parent_dir_name} STREQUAL thrift)
        list(REMOVE_ITEM WEARABLEDATA_INCLUDE_DIRS ${dir})
        get_filename_component(parent_dir_path ${dir} DIRECTORY)
        list(APPEND WEARABLEDATA_INCLUDE_DIRS ${parent_dir_path})
    endif()
endforeach()

# Setup the include directories
target_include_directories(WearableData PUBLIC
    $<BUILD_INTERFACE:${WEARABLEDATA_INCLUDE_DIRS}>)
target_include_directories(WearableData PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

install(TARGETS WearableData
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# =======================
# XsensSuitControlService
# =======================

yarp_add_idl(XSENSSUITCONTROL XsensSuitControlService.thrift)

add_library(XsensSuitControl ${XSENSSUITCONTROL} XsensSuitControlService.thrift)
target_link_libraries(XsensSuitControl YARP::YARP_OS)

# Extract the include directory from the files names
foreach(file ${XSENSSUITCONTROL})
    STRING(REGEX MATCH ".+\\.h?h$" file ${file})
    if(file)
        get_filename_component(include_dir ${file} DIRECTORY)
        list(APPEND XSENSSUITCONTROL_INCLUDE_DIRS ${include_dir})
        list(REMOVE_DUPLICATES XSENSSUITCONTROL_INCLUDE_DIRS)
    endif()
endforeach()

foreach(dir ${XSENSSUITCONTROL_INCLUDE_DIRS})
    get_filename_component(parent_dir_name ${dir} NAME)
    if(${parent_dir_name} STREQUAL thrift)
        list(REMOVE_ITEM XSENSSUITCONTROL_INCLUDE_DIRS ${dir})
        get_filename_component(parent_dir_path ${dir} DIRECTORY)
        list(APPEND XSENSSUITCONTROL_INCLUDE_DIRS ${parent_dir_path})
    endif()
endforeach()

# Setup the include directories
target_include_directories(XsensSuitControl PUBLIC
    $<BUILD_INTERFACE:${XSENSSUITCONTROL_INCLUDE_DIRS}>)

install(TARGETS XsensSuitControl
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
