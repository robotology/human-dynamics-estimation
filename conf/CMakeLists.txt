# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set (HDE_PROJECT_NAME HumanDynamicsEstimation)

# HDE xml files installation
file (GLOB HDE_XML_FILES xml/*.xml)

install(FILES ${HDE_XML_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${HDE_PROJECT_NAME}/)

# HDE ini files installation
file (GLOB HDE_INI_FILES app/*.ini)

install(FILES ${HDE_INI_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${HDE_PROJECT_NAME}/)

# Install yarpmanager application files
file (GLOB HDE_APP_FILES xml/applications/*.xml)

yarp_install(FILES ${HDE_APP_FILES} DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})

# Install robot urdf files
file (GLOB ROBOT_URDF_FILES urdfs/*.urdf)
yarp_install(FILES ${ROBOT_URDF_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${HDE_PROJECT_NAME}/)

# Install custom constraint matrix
file(GLOB scripts config/*.ini)
yarp_install(FILES ${scripts} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR})


# ROS related installation
set (ROS_PROJECT_NAME HDERviz)
file (GLOB ROS_PACKAGE_FILE ros/package.xml)
file (GLOB ROS_LAUNCH_FILES ros/launch/*.launch)
file (GLOB ROS_RVIZ_FILES ros/rviz/*.rviz)

# Copy the package.xml, which is necessary to be treated as
# a ROS package
install(FILES ${ROS_PACKAGE_FILE}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${ROS_PROJECT_NAME}/)


# Install the .launch file and the config file
install(FILES ${ROS_LAUNCH_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${ROS_PROJECT_NAME}/launch/)

install(FILES ${ROS_RVIZ_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${ROS_PROJECT_NAME}/rviz/)

# Install robot urdf files
install(FILES ${ROBOT_URDF_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${ROS_PROJECT_NAME}/urdfs/)
