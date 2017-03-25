YARP_PREPARE_PLUGIN(readonlyremotecontrolboard TYPE yarp::dev::ReadOnlyRemoteControlBoard
                                               INCLUDE ReadOnlyRemoteControlBoard.h
                                               CATEGORY device)

if(ENABLE_readonlyremotecontrolboard)

  set(PLUGIN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ReadOnlyRemoteControlBoard.cpp"
                     "${CMAKE_CURRENT_SOURCE_DIR}/stateExtendedReader.cpp"
                     "${CMAKE_CURRENT_SOURCE_DIR}/jointData.cpp")

  set(PLUGIN_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/ReadOnlyRemoteControlBoard.h"
                     "${CMAKE_CURRENT_SOURCE_DIR}/stateExtendedReader.hpp"
                     "${CMAKE_CURRENT_SOURCE_DIR}/jointData.h")

  yarp_add_plugin(readonlyremotecontrolboard ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

  target_link_libraries(readonlyremotecontrolboard YARP::YARP_OS YARP::YARP_dev YARP::YARP_sig)
  target_include_directories(readonlyremotecontrolboard SYSTEM PUBLIC ${YARP_INCLUDE_DIRS})
  target_include_directories(readonlyremotecontrolboard PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

  yarp_install(TARGETS readonlyremotecontrolboard
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/readonlyremotecontrolboard.ini"  DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
