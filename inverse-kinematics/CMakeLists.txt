cmake_minimum_required(VERSION 3.0)
project(inverse-kinematics CXX)

set(SOURCES src/InverseKinematicsIPOPT.cpp
            src/InverseKinematicsV2IPOPT.cpp
            src/InverseKinematics.cpp)

set(HEADERS include/inversekinematics/InverseKinematicsIPOPT.h
            include/inversekinematics/InverseKinematicsV2IPOPT.h
            include/inversekinematics/InverseKinematics.h)


add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})


target_compile_definitions(${PROJECT_NAME} PUBLIC ${IPOPT_DEFINITIONS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Debug>:EIGEN_RUNTIME_NO_MALLOC>")

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/inversekinematics)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${iDynTree_LIBRARIES} ${IPOPT_LIBRARIES})

#?
# target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/inversekinematics>"
#  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

set_property(TARGET ${PROJECT_NAME} PROPERTY PUBLIC_HEADER include/InverseKinematics.h)


if(BUILD_TESTING)
   add_subdirectory(tests)
endif()

# install(TARGETS ${PROJECT_NAME}
#         LIBRARY DESTINATION lib
#         PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InverseKinematics/
#         )
