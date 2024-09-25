# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

if(HDE_COMPILE_PYTHON_BINDINGS)

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
