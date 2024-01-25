# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

if(HDE_COMPILE_PYTHON_BINDINGS)

  find_package(Python3 3.6 COMPONENTS Interpreter Development REQUIRED)

  find_package(pybind11 2.4.3 CONFIG REQUIRED)

  set(NEW_LINE "\n")

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
  set(PYTHON_INSTDIR ${HDE_PYTHON_INSTALL_DIR}/hde)

  # Folder of the Python package within the build tree.
  # It is used for the Python tests.
  set(HDE_PYTHON_PACKAGE "${CMAKE_BINARY_DIR}/hde")

  # Add the bindings directory
  add_subdirectory(python)

  # Create the __init__.py file
  file(GENERATE
    OUTPUT "${HDE_PYTHON_PACKAGE}/__init__.py"
    CONTENT "from .bindings import *${NEW_LINE}")

  # Install the __init__.py file
  install(FILES "${HDE_PYTHON_PACKAGE}/__init__.py"
    DESTINATION ${PYTHON_INSTDIR})

  # Install pip metadata files to ensure that HDE installed via CMake is listed by pip list
  # See https://packaging.python.org/specifications/recording-installed-packages/
  # and https://packaging.python.org/en/latest/specifications/core-metadata/#core-metadata
  option(HDE_PYTHON_PIP_METADATA_INSTALL "Use CMake to install Python pip metadata. Set to off if some other tool already installs it." ON)
  mark_as_advanced(HDE_PYTHON_PIP_METADATA_INSTALL)
  set(HDE_PYTHON_PIP_METADATA_INSTALLER "cmake" CACHE STRING "Specify the string to identify the pip Installer. Default: cmake, change this if you are using another tool.")
  mark_as_advanced(HDE_PYTHON_PIP_METADATA_INSTALLER)
  if(HDE_PYTHON_PIP_METADATA_INSTALL)
    get_filename_component(PYTHON_METADATA_PARENT_DIR ${PYTHON_INSTDIR} DIRECTORY)
    if(WIN32)
      set(NEW_LINE "\n\r")
    else()
      set(NEW_LINE "\n")
    endif()
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/METADATA "")
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Metadata-Version: 2.1${NEW_LINE}")
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Name: hde${NEW_LINE}")
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Version: ${hde_VERSION}${NEW_LINE}")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/INSTALLER "${HDE_PYTHON_PIP_METADATA_INSTALLER}${NEW_LINE}")
    install(
      FILES "${CMAKE_CURRENT_BINARY_DIR}/METADATA" "${CMAKE_CURRENT_BINARY_DIR}/INSTALLER"
      DESTINATION ${PYTHON_METADATA_PARENT_DIR}/hde-${hde_VERSION}.dist-info)
  endif()

endif()