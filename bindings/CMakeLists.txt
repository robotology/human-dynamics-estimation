# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# Detect if we are doing a standalone build of the bindings, using an external icub-main
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(HDE_BINDINGS_BUILD_STANDALONE TRUE)
else()
  set(HDE_BINDINGS_BUILD_STANDALONE FALSE)
endif()

if(HDE_BINDINGS_BUILD_STANDALONE)
  cmake_minimum_required(VERSION 3.16)
  # Extract project version from top project
  file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../CMakeLists.txt cmake_content)
  string(REGEX MATCH "VERSION[ ]+([0-9]+\\.[0-9]+\\.[0-9]+)" cmake_version_match ${cmake_content})
  set(top_project_version ${CMAKE_MATCH_1})
  project(HumanDynamicsEstimationBindings
          VERSION ${top_project_version})
  set(HDE_COMPILE_PYTHON_BINDINGS ON)
  find_package(YARP REQUIRED)
  find_package(WearableData REQUIRED)
  find_package(HumanDynamicsEstimation REQUIRED)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
endif()

if(HDE_COMPILE_PYTHON_BINDINGS)
  include(AddWearablesPythonModule)
  include(AddHDEPythonModule)

  find_package(Python3 3.6 COMPONENTS Interpreter Development REQUIRED)

  find_package(pybind11 2.4.3 CONFIG REQUIRED)

  option(HDE_DETECT_ACTIVE_PYTHON_SITEPACKAGES
    "Do you want HDE to detect and use the active site-package directory? (it could be a system dir)"
    FALSE)

  # Install the resulting Python package for the active interpreter
  if(NOT DEFINED HDE_PYTHON_INSTALL_DIR)
    if(HDE_DETECT_ACTIVE_PYTHON_SITEPACKAGES)
      set(HDE_PYTHON_INSTALL_DIR ${Python3_SITELIB})
    else()
      execute_process(COMMAND ${Python3_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
        OUTPUT_VARIABLE _PYTHON_INSTDIR)
      string(STRIP ${_PYTHON_INSTDIR} _PYTHON_INSTDIR_CLEAN)
      set(HDE_PYTHON_INSTALL_DIR ${_PYTHON_INSTDIR_CLEAN})
    endif()
  endif()

  # Add the bindings directory
  add_subdirectory(python-wearables)

  # Add the bindings directory
  add_subdirectory(python-hde)
endif()
