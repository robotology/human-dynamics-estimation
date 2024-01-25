# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_subdirectory(msgs)

get_property(pybind_headers GLOBAL PROPERTY pybind_headers)
get_property(pybind_sources GLOBAL PROPERTY pybind_sources)
get_property(pybind_include_dirs GLOBAL PROPERTY pybind_include_dirs)
get_property(pybind_link_libraries GLOBAL PROPERTY pybind_link_libraries)

pybind11_add_module(pybind11_hde MODULE
  hde.cpp
  ${pybind_sources}
  ${pybind_headers}
  )

target_include_directories(pybind11_hde PUBLIC "$<BUILD_INTERFACE:${pybind_include_dirs}>")

target_link_libraries(pybind11_hde PRIVATE
  ${pybind_link_libraries})

# # The generated Python dynamic module must have the same name as the pybind11
# # module, i.e. `bindings`.
set_target_properties(pybind11_hde PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${HDE_PYTHON_PACKAGE}"
  OUTPUT_NAME "bindings")

# Output package is:
#
# bipedal_locomotion
# |-- __init__.py (generated from main bindings CMake file)
# `-- bindings.<cpython_extension>

install(TARGETS pybind11_hde DESTINATION ${PYTHON_INSTDIR})