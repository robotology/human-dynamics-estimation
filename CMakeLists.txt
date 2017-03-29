cmake_minimum_required(VERSION 3.0)

project(HumanDynamicsEstimation CXX)

include(GNUInstallDirs)
include(CTest)

option(BUILD_TESTING "Build tests" OFF)

## Find packages
find_package(YARP REQUIRED)
find_package(YCM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(YarpInstallationHelpers)
include(YarpIDL)

include(AddWarningsConfigurationToTarget)

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Shared/Dynamic or Static library?
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

include(AddInstallRPATHSupport)
#check discussion in YARP
option(HUMAN_DYNAMICS_ESTIMATION_ENABLE_RPATH "Enable rpath" TRUE)
mark_as_advanced(ENABLE_RPATH)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          DEPENDS HUMAN_DYNAMICS_ESTIMATION_ENABLE_RPATH
                          USE_LINK_PATH)

option(HUMAN_DYNAMICS_ESTIMATION_DEBUG_MALLOC "Enable Eigen assertions on malloc" OFF)

add_subdirectory(ros-msgs)
#Add IK solver
find_package(IPOPT)
option(HUMAN_DYNAMICS_ESTIMATION_USE_IPOPT "Set if compile IPOPT dependent projects" ${IPOPT_FOUND})
if (HUMAN_DYNAMICS_ESTIMATION_USE_IPOPT)
  add_subdirectory(inverse-kinematics)
endif()
add_subdirectory(human-state-provider)
add_subdirectory(human-forces-provider)
add_subdirectory(human-dynamics-estimator)
add_subdirectory(human-viz-bridge)

add_subdirectory(app)

add_subdirectory(yarp)

#### add the "uninstall" target
include(AddUninstallTarget)

