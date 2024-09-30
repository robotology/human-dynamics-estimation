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
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
endif()

if(HDE_COMPILE_PYTHON_BINDINGS)
  include(AddWearablesPythonModule)
  include(AddHDEPythonModule)

  find_package(Python3 3.6 COMPONENTS Interpreter Development REQUIRED)

  find_package(pybind11 2.4.3 CONFIG REQUIRED)

  option(HDE_DETECT_ACTIVE_PYTHON_SITEPACKAGES
    "Do you want HDE to detect and use the active site-package directory? (it could be a system dir)"
    FALSE)

  # Function to generate __init__.py file for both hde and wearables
  # It contains logic to ensure that it works fine on Windows when installing in arbitrary folders,
  # See https://docs.python.org/3.8/library/os.html#os.add_dll_directory
  function(hde_generate_init_py_file GENERATION_LOCATION_OF_INIT_PY_FILE INSTALLATION_LOCATION_OF_INIT_PY_FILE)
    file(WRITE ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "")
    if(WIN32 AND BUILD_SHARED_LIBS)
        if(IS_ABSOLUTE ${INSTALLATION_LOCATION_OF_INIT_PY_FILE})
            set(PYTHON_FULL_INSTDIR "${INSTALLATION_LOCATION_OF_INIT_PY_FILE}")
        else()
            set(PYTHON_FULL_INSTDIR "${CMAKE_INSTALL_PREFIX}/${INSTALLATION_LOCATION_OF_INIT_PY_FILE}")
        endif()
        file(RELATIVE_PATH RELATIVE_PATH_BETWEEN_INIT_PY_AND_DLL_DIRECTORY ${PYTHON_FULL_INSTDIR} ${CMAKE_INSTALL_FULL_BINDIR})
        file(APPEND ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "import os\n")
        file(APPEND ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "library_dll_path = os.path.join(os.path.dirname(__file__),'${RELATIVE_PATH_BETWEEN_INIT_PY_AND_DLL_DIRECTORY}')\n")
        file(APPEND ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "# Avoid to call add_dll_directory if not necessary,\n")
        file(APPEND ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "# for example if the library to find are already found in the proper location in a conda\n")
        file(APPEND ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "if(library_dll_path != os.path.join(os.environ.get('CONDA_PREFIX', ''),'Library','bin') and library_dll_path != os.path.join(os.environ.get('CONDA_PREFIX', ''),'bin')):\n")
        file(APPEND ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "    if(os.path.exists(library_dll_path)):\n")
        file(APPEND ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "        os.add_dll_directory(library_dll_path)\n")
        file(APPEND ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "\n")
    endif()

    file(APPEND ${GENERATION_LOCATION_OF_INIT_PY_FILE}/__init__.py "from .bindings import *\n")
  endfunction()

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
