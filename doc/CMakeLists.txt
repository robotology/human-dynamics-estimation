# Provide a local project and cmake_minimum_required calls
# to permit to generate the documentation without configuring
# the complete project

# This variable should contain the argument of the project call in the main CMakeLists.txt
set(ORIGINAL_PROJECT_NAME ${PROJECT_NAME})
set(ORIGINAL_PROJECT_VERSION ${PROJECT_VERSION})

project("${ORIGINAL_PROJECT_NAME} Docs")

include(GNUInstallDirs)

find_package(Doxygen QUIET)

if(DOXYGEN_FOUND)
  set(DOXYGEN_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}"
      CACHE STRING "Build directory for Doxygen html output (absolute).")
  set(DOXYGEN_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/doc/${ORIGINAL_PROJECT_NAME}"
      CACHE STRING "Installation directory for Doxygen html output (relative to CMAKE_INSTALL_PREFIX).")

  # This variables are used inside the Doxyfile.in template
  set(DOXYGEN_PROJECT_BRIEF "Software repository for estimating human dynamics")
  set(DOXYGEN_SOURCE_DIR "${CMAKE_HOME_DIRECTORY}")
  set(DOXYGEN_DOC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  set(DOXYGEN_EXCLUDE_DIR "")

  # Generate the doxygen configuration file used by the
  # document generation. This step permits to use cmake variable
  # such as @ORIGINAL_PROJECT_NAME@ in the Doxyfile.in template
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  # Move other files required by doxygen in the build folder
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/customdoxygen.css ${CMAKE_CURRENT_BINARY_DIR}/ COPYONLY)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/DoxygenLayout.xml ${CMAKE_CURRENT_BINARY_DIR}/ COPYONLY)

  # The dox target will generate the doxygen documentation in ${DOXYGEN_BUILD_DIR}
  add_custom_target(dox COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
                        COMMENT "Generating doxygen documentation in ${DOXYGEN_BUILD_DIR}")

  # Note: the install will install the documentation if present,
  # but it will not trigger a regeneration of the documentation if it is outdated
  install(DIRECTORY ${DOXYGEN_BUILD_DIR}/html
          DESTINATION ${DOXYGEN_INSTALL_DIR}
          OPTIONAL)

endif()
