cmake_minimum_required(VERSION 3.0)
project(human-ik CXX)

set(SOURCES src/InverseKinematics.cpp)

set(HEADERS include/human-ik/InverseKinematics.h)

set(PRIVATE_SOURCES src/InverseKinematicsIPOPT.cpp
                    src/InverseKinematicsV2IPOPT.cpp)

set(PRIVATE_HEADERS include/private/InverseKinematicsIPOPT.h
                    include/private/InverseKinematicsV2IPOPT.h)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS} ${PRIVATE_SOURCES} ${PRIVATE_HEADERS})

target_compile_definitions(${PROJECT_NAME} PRIVATE ${IPOPT_DEFINITIONS})
# target_compile_definitions(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Debug>:EIGEN_RUNTIME_NO_MALLOC>")

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/human-ik)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/private)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${iDynTree_LIBRARIES} ${IPOPT_LIBRARIES})

#?
# target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/inversekinematics>"
#  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

set_property(TARGET ${PROJECT_NAME} PROPERTY PUBLIC_HEADER ${HEADERS})


if(BUILD_TESTING)
   add_subdirectory(tests)
endif()


install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CMAKE_INSTALL_FULL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/human-ik/)