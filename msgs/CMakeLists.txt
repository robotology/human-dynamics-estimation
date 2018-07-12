# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_add_idl(WEARABLEDATA_FILES thrift/WearableData.thrift)

add_library(WearableData ${WEARABLEDATA_FILES} thrift/WearableData.thrift)
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

install(TARGETS WearableData
        RUNTIME DESTINATION ${CMAKE_INSTALL_FULL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
