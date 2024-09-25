# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(PYTHON_INSTDIR ${HDE_PYTHON_INSTALL_DIR}/wearables)

# Folder of the Python package within the build tree.
# It is used for the Python tests.
set(WEARABLES_PYTHON_PACKAGE "${CMAKE_BINARY_DIR}/wearables")

add_subdirectory(msgs)

get_property(pybind_headers GLOBAL PROPERTY pybind_headers)
get_property(pybind_sources GLOBAL PROPERTY pybind_sources)
get_property(pybind_include_dirs GLOBAL PROPERTY pybind_include_dirs)
get_property(pybind_link_libraries GLOBAL PROPERTY pybind_link_libraries)

pybind11_add_module(pybind11_wearables MODULE
  wearables.cpp
  ${pybind_sources}
  ${pybind_headers}
  )

target_include_directories(pybind11_wearables PUBLIC "$<BUILD_INTERFACE:${pybind_include_dirs}>")

target_link_libraries(pybind11_wearables PRIVATE
  ${pybind_link_libraries})

# # The generated Python dynamic module must have the same name as the pybind11
# # module, i.e. `bindings`.
set_target_properties(pybind11_wearables PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${WEARABLES_PYTHON_PACKAGE}"
  ARCHIVE_OUTPUT_DIRECTORY "${WEARABLES_PYTHON_PACKAGE}"
  RUNTIME_OUTPUT_DIRECTORY "${WEARABLES_PYTHON_PACKAGE}"
  PDB_OUTPUT_DIRECTORY "${WEARABLES_PYTHON_PACKAGE}"
  OUTPUT_NAME "bindings")

# Output package is:
#
# bipedal_locomotion
# |-- __init__.py (generated from main bindings CMake file)
# `-- bindings.<cpython_extension>

install(TARGETS pybind11_wearables DESTINATION ${PYTHON_INSTDIR})

file(GENERATE
  OUTPUT "${WEARABLES_PYTHON_PACKAGE}/__init__.py"
  CONTENT "from .bindings import *\n")
# Install the __init__.py file
install(FILES "${WEARABLES_PYTHON_PACKAGE}/__init__.py"
  DESTINATION ${PYTHON_INSTDIR})
# Install pip metadata files to ensure that WEARABLES installed via CMake is listed by pip list
# See https://packaging.python.org/specifications/recording-installed-packages/
# and https://packaging.python.org/en/latest/specifications/core-metadata/#core-metadata
option(HDE_PYTHON_PIP_METADATA_INSTALL "Use CMake to install Python pip metadata. Set to off if some other tool already installs it." ON)
mark_as_advanced(HDE_PYTHON_PIP_METADATA_INSTALL)
set(HDE_PYTHON_PIP_METADATA_INSTALLER "cmake" CACHE STRING "Specify the string to identify the pip Installer. Default: cmake, change this if you are using another tool.")
mark_as_advanced(HDE_PYTHON_PIP_METADATA_INSTALLER)
if(HDE_PYTHON_PIP_METADATA_INSTALL)
  get_filename_component(PYTHON_METADATA_PARENT_DIR ${PYTHON_INSTDIR} DIRECTORY)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/METADATA "")
  file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Metadata-Version: 2.1\n")
  file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Name: wearables\n")
  file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Version: ${PROJECT_VERSION}\n")
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/INSTALLER "${HDE_PYTHON_PIP_METADATA_INSTALLER}\n")
  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/METADATA" "${CMAKE_CURRENT_BINARY_DIR}/INSTALLER"
    DESTINATION ${PYTHON_METADATA_PARENT_DIR}/wearables-${PROJECT_VERSION}.dist-info)
endif()