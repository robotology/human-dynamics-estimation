# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

function(add_hde_python_module)

  set(options )
  set(oneValueArgs NAME)
  set(multiValueArgs
    SOURCES
    HEADERS
    LINK_LIBRARIES
    TESTS
    TESTS_RUNTIME_CONDITIONS)

  set(prefix "hde")

  cmake_parse_arguments(${prefix}
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  set(name ${${prefix}_NAME})
  set(is_interface ${${prefix}_IS_INTERFACE})
  set(sources ${${prefix}_SOURCES})
  set(headers ${${prefix}_HEADERS})
  set(link_libraries ${${prefix}_LINK_LIBRARIES})
  set(subdirectories ${${prefix}_SUBDIRECTORIES})
  set(tests ${${prefix}_TESTS})
  set(tests_runtime_conditions ${${prefix}_TESTS_RUNTIME_CONDITIONS})

  foreach(file ${headers})
    set_property(GLOBAL APPEND PROPERTY pybind_headers ${CMAKE_CURRENT_SOURCE_DIR}/${file})
  endforeach()

  foreach(file ${sources})
    set_property(GLOBAL APPEND PROPERTY pybind_sources ${CMAKE_CURRENT_SOURCE_DIR}/${file})
  endforeach()

  set_property(GLOBAL APPEND PROPERTY pybind_include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/include)

  set_property(GLOBAL APPEND PROPERTY pybind_link_libraries ${link_libraries})

  message(STATUS "Added files for bindings named ${name} in ${PROJECT_NAME}.")

endfunction()
