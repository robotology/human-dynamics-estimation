add_library(yarp-experimental STATIC ${CMAKE_CURRENT_SOURCE_DIR}/IHumanSkeleton.cpp)
target_include_directories(yarp-experimental PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/yarp/dev)
target_include_directories(yarp-experimental PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(yarp-experimental YARP::YARP_sig)