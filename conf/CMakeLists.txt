# HDE xml files installation
set (HDE_PROJECT_NAME HumanDynamicsEstimation)
set (HDE_XML_FILES xml/TransformServer.xml
                   xml/Human.xml
                   xml/pHRI.xml
                   xml/RobotPosePublisher.xml
                   xml/RobotStateProvider_Human.xml
                   xml/RobotStateProvider_iCub.xml
            		   xml/RobotStateProvider_iCub_Pole.xml
                   xml/RobotStateProvider_Nao.xml
                   xml/RobotStateProvider_Walkman.xml
                   xml/RobotStateProvider_Atlas.xml
                   xml/RobotStateProvider_Baxter.xml
                   xml/HumanJointTorquesYarpScope.xml
                   xml/applications/HumanDynamicsEstimation-HumanDumper.xml
                   )

install(FILES ${HDE_XML_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${HDE_PROJECT_NAME}/)

# Install yarpmanager application files
set(HDE_APP_FILES xml/applications/HumanDynamicsEstimation.xml
                  xml/applications/HumanDynamicsEstimation-Human.xml
                  xml/applications/HumanDynamicsEstimation-pHRI.xml
                  xml/applications/HumanDynamicsEstimation-Rviz.xml
                  xml/applications/HumanDynamicsEstimation-TransformServer.xml
                  xml/applications/HumanDynamicsEstimation-YarpScope.xml
                  xml/applications/XsensRetargetingVisualization.xml
		  xml/applications/XsensRetargetingPositionControlPole.xml)

yarp_install(FILES ${HDE_APP_FILES} DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})

# Install robot urdf files
set(ROBOT_URDF_FILES urdfs/iCubGenova02.urdf
               urdfs/iCubGenova04.urdf
               urdfs/iCubGazeboV2_5.urdf
	       urdfs/teleoperation_iCub_model_V_2_5.urdf)
yarp_install(FILES ${ROBOT_URDF_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${HDE_PROJECT_NAME}/)

# Install custom constraint matrix
file(GLOB scripts config/*.ini)
yarp_install(FILES ${scripts} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR})


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
