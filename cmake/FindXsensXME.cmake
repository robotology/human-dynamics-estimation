# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

#.rst:
# FindXsensXME
# -----------
#
# Find the Xsens MVN Engine (XME) library API.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets if
# XME drivers has been found::
#
#   Xsens::XME
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module defines the following variables::
#
#   XsensXME_FOUND                - System has XME
#   XsensXME_INCLUDE_DIRS         - Include directories for XME
#   XsensXME_LIBRARIES            - libraries to link against XME
#
# Readed enviromental variables
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#
# This module reads hints about search locations from variables::
#
#    XsensXME_ROOT                 - Directory containing the SDK files.

if(WIN32)

  if(NOT CMAKE_SIZEOF_VOID_P STREQUAL "8")
    #32bit
    set(XME_ARCH "32")
    set(XME_DIR_PATH "Win32")
  else()
    #64bit
    set(XME_ARCH "64")
    set(XME_DIR_PATH "x64")
  endif()


  find_path(Xsens_INCLUDE_DIR xme.h
            HINTS "$ENV{XsensXME_ROOT}/${XME_DIR_PATH}/include")

  find_library(Xsens_xme_LIBRARY "xme${XME_ARCH}"
               HINTS "$ENV{XsensXME_ROOT}/${XME_DIR_PATH}/lib")

  find_library(Xsens_xstypes_LIBRARY "xstypes${XME_ARCH}"
               HINTS "$ENV{XsensXME_ROOT}/${XME_DIR_PATH}/lib")


  mark_as_advanced(Xsens_INCLUDE_DIR
                   Xsens_xme_LIBRARY
                   Xsens_xstypes_LIBRARY)

  if (Xsens_xme_LIBRARY
      AND Xsens_xstypes_LIBRARY
      AND Xsens_INCLUDE_DIR
      AND NOT TARGET Xsens::XME)

    add_library(Xsens::XME UNKNOWN IMPORTED)

    set_target_properties(Xsens::XME PROPERTIES
                          INTERFACE_INCLUDE_DIRECTORIES "${Xsens_INCLUDE_DIR}"
                          IMPORTED_LINK_INTERFACE_LANGUAGES "C"
                          IMPORTED_LOCATION "${Xsens_xme_LIBRARY}"
                          INTERFACE_LINK_LIBRARIES "${Xsens_xstypes_LIBRARY}")

    set(XsensXME_LIBRARIES Xsens::XME)
    set(XsensXME_INCLUDE_DIRS "${Xsens_INCLUDE_DIR}")
  endif()
else()
  set(XsensXME_FOUND FALSE)
endif()


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(XsensXME
                                  FOUND_VAR XsensXME_FOUND
                                  REQUIRED_VARS XsensXME_LIBRARIES XsensXME_INCLUDE_DIRS)

# Set package properties if FeatureSummary was included
if(COMMAND set_package_properties)
    set_package_properties(XsensXME PROPERTIES DESCRIPTION "API for Xsens MVN suit"
                                               URL "https://www.xsens.com/products/xsens-mvn/")
endif()
