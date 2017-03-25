YARP_PREPARE_PLUGIN(readonlyremotecontrolboardremapper TYPE yarp::dev::ReadOnlyRemoteControlBoardRemapper
                                                       INCLUDE ReadOnlyRemoteControlBoardRemapper.h
                                                       CATEGORY device)

if(ENABLE_readonlyremotecontrolboardremapper)

  set(PLUGIN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ReadOnlyControlBoardRemapper.cpp"
                     "${CMAKE_CURRENT_SOURCE_DIR}/ReadOnlyRemoteControlBoardRemapper.cpp"
                     "${CMAKE_CURRENT_SOURCE_DIR}/ControlBoardRemapperHelpers.cpp")

  set(PLUGIN_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/ReadOnlyControlBoardRemapper.h"
                     "${CMAKE_CURRENT_SOURCE_DIR}/ReadOnlyRemoteControlBoardRemapper.h"
                     "${CMAKE_CURRENT_SOURCE_DIR}/ControlBoardRemapperHelpers.h")

  yarp_add_plugin(readonlyremotecontrolboardremapper ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

  target_link_libraries(readonlyremotecontrolboardremapper YARP::YARP_OS YARP::YARP_dev YARP::YARP_sig)
  target_include_directories(readonlyremotecontrolboardremapper SYSTEM PUBLIC ${YARP_INCLUDE_DIRS})
  target_include_directories(readonlyremotecontrolboardremapper PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

  yarp_install(TARGETS readonlyremotecontrolboardremapper
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/conf/readonlyremotecontrolboardremapper.ini"  DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
