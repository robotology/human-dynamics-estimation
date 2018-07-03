# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(YARP REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

yarp_add_idl(WEARDATA_FILES thrift/WearData.thrift)

add_library(WearData ${WEARDATA_FILES} thrift/WearData.thrift)
#set_target_properties(WearData PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(WearData YARP::YARP_OS)

# Extract the include directory from the files names
foreach(file ${WEARDATA_FILES})
    STRING(REGEX MATCH ".+\\.h?h$" file ${file})
    if(file)
        get_filename_component(include_dir ${file} DIRECTORY)
        list(APPEND WEARDATA_INCLUDE_DIRS ${include_dir})
        list(REMOVE_DUPLICATES WEARDATA_INCLUDE_DIRS)
    endif()
endforeach()

foreach(dir ${WEARDATA_INCLUDE_DIRS})
    get_filename_component(parent_dir_name ${dir} NAME)
    if(${parent_dir_name} STREQUAL thrift)
        list(REMOVE_ITEM WEARDATA_INCLUDE_DIRS ${dir})
        get_filename_component(parent_dir_path ${dir} DIRECTORY)
        list(APPEND WEARDATA_INCLUDE_DIRS ${parent_dir_path})
    endif()
endforeach()

# Setup the include directories
target_include_directories(WearData PUBLIC
    $<BUILD_INTERFACE:${WEARDATA_INCLUDE_DIRS}>)

install(TARGETS WearData
        RUNTIME DESTINATION ${CMAKE_INSTALL_FULL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
