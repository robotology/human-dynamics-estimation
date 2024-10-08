# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# TODO: these libraries are enforced to be static. In order to let the devices
#       find them on Windows from the build directory, their Release / Debug
#       folder should be added to the PATH. However, this does not work as expected.

# ============
# WearableData
# ============

yarp_add_idl(WEARABLEDATA_FILES thrift/WearableData.thrift)

add_library(WearableData ${WEARABLEDATA_FILES} thrift/WearableData.thrift)
add_library(WearableData::WearableData ALIAS WearableData)
target_link_libraries(WearableData YARP::YARP_os)

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
        EXPORT WearableData
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install_basic_package_files(WearableData
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
        EXPORT WearableData
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
        DEPENDENCIES YARP)

install(FILES ${WEARABLEDATA_FILES}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/thrift)

# =================
# WearableActuators
# =================

yarp_add_idl(WEARABLEACTUATORS_FILES thrift/WearableActuators.thrift)

add_library(WearableActuators ${WEARABLEACTUATORS_FILES} thrift/WearableActuators.thrift)
add_library(WearableActuators::WearableActuators ALIAS WearableActuators)
target_link_libraries(WearableActuators YARP::YARP_os)

# Extract the include directory from the files names
foreach(file ${WEARABLEACTUATORS_FILES})
    STRING(REGEX MATCH ".+\\.h?h$" file ${file})
    if(file)
        get_filename_component(include_dir ${file} DIRECTORY)
        list(APPEND WEARABLEACTUATORS_INCLUDE_DIRS ${include_dir})
        list(REMOVE_DUPLICATES WEARABLEACTUATORS_INCLUDE_DIRS)
    endif()
endforeach()

foreach(dir ${WEARABLEACTUATORS_INCLUDE_DIRS})
    get_filename_component(parent_dir_name ${dir} NAME)
    if(${parent_dir_name} STREQUAL thrift)
        list(REMOVE_ITEM WEARABLEACTUATORS_INCLUDE_DIRS ${dir})
        get_filename_component(parent_dir_path ${dir} DIRECTORY)
        list(APPEND WEARABLEACTUATORS_INCLUDE_DIRS ${parent_dir_path})
    endif()
endforeach()

# Setup the include directories
target_include_directories(WearableActuators PUBLIC
    $<BUILD_INTERFACE:${WEARABLEACTUATORS_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS WearableActuators
        EXPORT WearableActuators
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install_basic_package_files(WearableActuators
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
        EXPORT WearableActuators
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
        DEPENDENCIES YARP)

install(FILES ${WEARABLEACTUATORS_FILES}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/thrift)

# =======================
# XsensSuitControlService
# =======================

yarp_add_idl(XSENSSUITCONTROL thrift/XsensSuitControlService.thrift)

add_library(XsensSuitControl ${XSENSSUITCONTROL} thrift/XsensSuitControlService.thrift)
target_link_libraries(XsensSuitControl YARP::YARP_os)

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

add_subdirectory(yarp)
add_subdirectory(ros)
