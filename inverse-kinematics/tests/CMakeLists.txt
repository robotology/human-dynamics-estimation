cmake_minimum_required(VERSION 2.8.12)

# set the project name
project(InverseKinematicsTest)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/data)

add_executable(InverseKinematicsTest testIK.cpp)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/data/URDFdir.h.in" "${CMAKE_CURRENT_BINARY_DIR}/data/URDFdir.h" @ONLY)

target_link_libraries(InverseKinematicsTest inverse-kinematics ${iDynTree_LIBRARIES})

add_test(NAME testIK
        COMMAND ${PROJECT_NAME})
        
        
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package (Matio)
                    
if(MATIO_FOUND)
    message(STATUS "MATIO found, adding Suit Test")
    include_directories (${MATIO_INCLUDE_DIRS})
    add_executable(IKSuitTest testSuit.cpp)
    target_link_libraries(IKSuitTest inverseKinematics ${iDynTree_LIBRARIES} ${MATIO_LIBRARIES})
    add_test(NAME testSuitIK
            COMMAND IKSuitTest)
endif()
