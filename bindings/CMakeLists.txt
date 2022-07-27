# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(WEARABLES_COMPILE_PYTHON_BINDINGS)

  find_package(Python3 3.6 COMPONENTS Interpreter Development REQUIRED)

  find_package(pybind11 2.4.3 CONFIG REQUIRED)

  set(NEW_LINE "\n")

  option(WEARABLES_DETECT_ACTIVE_PYTHON_SITEPACKAGES
    "Do you want Wearables to detect and use the active site-package directory? (it could be a system dir)"
    FALSE)

  # Install the resulting Python package for the active interpreter
  if(WEARABLES_DETECT_ACTIVE_PYTHON_SITEPACKAGES)
    set(PYTHON_INSTDIR ${Python3_SITELIB}/wearables)
  else()
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
      OUTPUT_VARIABLE _PYTHON_INSTDIR)
    string(STRIP ${_PYTHON_INSTDIR} _PYTHON_INSTDIR_CLEAN)
    set(PYTHON_INSTDIR ${_PYTHON_INSTDIR_CLEAN}/wearables)
  endif()

  # Folder of the Python package within the build tree.
  # It is used for the Python tests.
  set(WEARABLES_PYTHON_PACKAGE "${CMAKE_BINARY_DIR}/wearables")

  # Add the bindings directory
  add_subdirectory(python)

  # Create the __init__.py file
  file(GENERATE
    OUTPUT "${WEARABLES_PYTHON_PACKAGE}/__init__.py"
    CONTENT "from .bindings import *${NEW_LINE}")

  # Install the __init__.py file
  install(FILES "${WEARABLES_PYTHON_PACKAGE}/__init__.py"
    DESTINATION ${PYTHON_INSTDIR})

  # Install pip metadata files to ensure that WEARABLES installed via CMake is listed by pip list
  # See https://packaging.python.org/specifications/recording-installed-packages/
  # and https://packaging.python.org/en/latest/specifications/core-metadata/#core-metadata
  option(WEARABLES_PYTHON_PIP_METADATA_INSTALL "Use CMake to install Python pip metadata. Set to off if some other tool already installs it." ON)
  mark_as_advanced(WEARABLES_PYTHON_PIP_METADATA_INSTALL)
  set(WEARABLES_PYTHON_PIP_METADATA_INSTALLER "cmake" CACHE STRING "Specify the string to identify the pip Installer. Default: cmake, change this if you are using another tool.")
  mark_as_advanced(WEARABLES_PYTHON_PIP_METADATA_INSTALLER)
  if(WEARABLES_PYTHON_PIP_METADATA_INSTALL)
    get_filename_component(PYTHON_METADATA_PARENT_DIR ${PYTHON_INSTDIR} DIRECTORY)
    if(WIN32)
      set(NEW_LINE "\n\r")
    else()
      set(NEW_LINE "\n")
    endif()
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/METADATA "")
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Metadata-Version: 2.1${NEW_LINE}")
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Name: wearables${NEW_LINE}")
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Version: ${Wearables_VERSION}${NEW_LINE}")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/INSTALLER "${WEARABLES_PYTHON_PIP_METADATA_INSTALLER}${NEW_LINE}")
    install(
      FILES "${CMAKE_CURRENT_BINARY_DIR}/METADATA" "${CMAKE_CURRENT_BINARY_DIR}/INSTALLER"
      DESTINATION ${PYTHON_METADATA_PARENT_DIR}/wearables-${Wearables_VERSION}.dist-info)
  endif()

endif()
