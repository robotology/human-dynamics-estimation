# HDE xml files installation
set (HDE_PROJECT_NAME HumanDynamicsEstimation)
file(GLOB HDE_XML_FILES xml/applications/*.xml)
install(FILES ${HDE_XML_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${HDE_PROJECT_NAME}/)

# Install yarpmanager application files
file(GLOB applications xml/applications/*.xml)
yarp_install(FILES ${applications} DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})

# Install robot urdf files
set(ROBOT_URDF_FILES urdfs/iCubGenova02.urdf
               urdfs/iCubGenova04.urdf
               urdfs/iCubGazeboV2_5.urdf
	       urdfs/teleoperation_iCub_model_V_2_5.urdf)

yarp_install(FILES ${ROBOT_URDF_FILES}
             DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/human-dynamic-estimation/urdfs)


# ROS related installation
set (ROS_PROJECT_NAME HDERviz)
set (ROS_PACKAGE_FILE ros/package.xml)
set (ROS_LAUNCH_FILES ros/launch/HDERviz.launch
                      ros/launch/iCubRviz.launch
                      ros/launch/twoHumansRviz.launch)
set (ROS_RVIZ_FILES ros/rviz/HDERviz.rviz
                    ros/rviz/iCubRviz.rviz
                    ros/rviz/twoHumansRviz.rviz)

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
