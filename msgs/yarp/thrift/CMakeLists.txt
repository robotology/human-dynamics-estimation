# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(LIBRARY_TARGET_NAME HumanStateMsg)

yarp_idl_to_dir(hde/msgs/HumanState.thrift
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/autogenerated
  SOURCES_VAR ${LIBRARY_TARGET_NAME}_GEN_SRC
  HEADERS_VAR ${LIBRARY_TARGET_NAME}_GEN_HDR
  INCLUDE_DIRS_VAR ${LIBRARY_TARGET_NAME}_INCLUDE_DIRS
  CMAKE_SCRIPTS_VAR ${LIBRARY_TARGET_NAME}_CMAKE_SCRIPTS)
foreach(_script ${${LIBRARY_TARGET_NAME}_CMAKE_SCRIPTS})
  include("${_script}")
endforeach()

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_GEN_SRC} ${${LIBRARY_TARGET_NAME}_GEN_HDR})
target_link_libraries(${LIBRARY_TARGET_NAME} YARP::YARP_OS YARP::YARP_init)

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES
                      PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_GEN_HDR}")

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                           "$<BUILD_INTERFACE:${${LIBRARY_TARGET_NAME}_INCLUDE_DIRS}>"
                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/msgs>")

set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY PUBLIC_HEADER ${${LIBRARY_TARGET_NAME}_GEN_HDR})

add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT ${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hde/msgs")



set(LIBRARY_TARGET_NAME HumanDynamicsMsg)

yarp_idl_to_dir(hde/msgs/HumanDynamics.thrift
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/autogenerated
  SOURCES_VAR ${LIBRARY_TARGET_NAME}_GEN_SRC
  HEADERS_VAR ${LIBRARY_TARGET_NAME}_GEN_HDR
  INCLUDE_DIRS_VAR ${LIBRARY_TARGET_NAME}_INCLUDE_DIRS
  CMAKE_SCRIPTS_VAR ${LIBRARY_TARGET_NAME}_CMAKE_SCRIPTS)
foreach(_script ${YARP_dev_idl_CMAKE_SCRIPTS})
  include("${_script}")
endforeach()

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_GEN_SRC} ${${LIBRARY_TARGET_NAME}_GEN_HDR})
target_link_libraries(${LIBRARY_TARGET_NAME} YARP::YARP_OS)

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES
                      PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_GEN_HDR}")

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                           "$<BUILD_INTERFACE:${${LIBRARY_TARGET_NAME}_INCLUDE_DIRS}>"
                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/msgs>")

set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY PUBLIC_HEADER ${${LIBRARY_TARGET_NAME}_GEN_HDR})

add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT ${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hde/msgs")



set(LIBRARY_TARGET_NAME WearableTargetsMsg)

yarp_idl_to_dir(hde/msgs/WearableTargets.thrift
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/autogenerated
  SOURCES_VAR ${LIBRARY_TARGET_NAME}_GEN_SRC
  HEADERS_VAR ${LIBRARY_TARGET_NAME}_GEN_HDR
  INCLUDE_DIRS_VAR ${LIBRARY_TARGET_NAME}_INCLUDE_DIRS
  CMAKE_SCRIPTS_VAR ${LIBRARY_TARGET_NAME}_CMAKE_SCRIPTS)
foreach(_script ${${LIBRARY_TARGET_NAME}_CMAKE_SCRIPTS})
  include("${_script}")
endforeach()

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_GEN_SRC} ${${LIBRARY_TARGET_NAME}_GEN_HDR})
target_link_libraries(${LIBRARY_TARGET_NAME} YARP::YARP_OS YARP::YARP_init)

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES
                      PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_GEN_HDR}")

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                            "$<BUILD_INTERFACE:${${LIBRARY_TARGET_NAME}_INCLUDE_DIRS}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/msgs>")

set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY PUBLIC_HEADER ${${LIBRARY_TARGET_NAME}_GEN_HDR})

add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT ${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hde/msgs")

set(LIBRARY_TARGET_NAME HumanWrenchMsg)

yarp_idl_to_dir(hde/msgs/HumanWrench.thrift
  OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/autogenerated
  SOURCES_VAR ${LIBRARY_TARGET_NAME}_GEN_SRC
  HEADERS_VAR ${LIBRARY_TARGET_NAME}_GEN_HDR
  INCLUDE_DIRS_VAR ${LIBRARY_TARGET_NAME}_INCLUDE_DIRS
  CMAKE_SCRIPTS_VAR ${LIBRARY_TARGET_NAME}_CMAKE_SCRIPTS)
foreach(_script ${${LIBRARY_TARGET_NAME}_CMAKE_SCRIPTS})
  include("${_script}")
endforeach()

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_GEN_SRC} ${${LIBRARY_TARGET_NAME}_GEN_HDR})
target_link_libraries(${LIBRARY_TARGET_NAME} YARP::YARP_OS YARP::YARP_init)

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES
                      PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_GEN_HDR}")

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                            "$<BUILD_INTERFACE:${${LIBRARY_TARGET_NAME}_INCLUDE_DIRS}>"
                            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/msgs>")

set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY PUBLIC_HEADER ${${LIBRARY_TARGET_NAME}_GEN_HDR})

add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT ${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hde/msgs")
