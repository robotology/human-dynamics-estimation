#HDE xml files installation
set (HDE_PROJECT_NAME HumanDynamicsEstimation)
set (HDE_XML_FILES xml/TransformServer.xml
                   xml/Human.xml
                   xml/pHRI.xml
                   xml/RobotPosePublisher.xml
                   xml/HumanJointTorquesYarpScope.xml)

install(FILES ${HDE_XML_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${HDE_PROJECT_NAME}/)

# install urdf files
set(URDF_FILES urdfs/iCubGenova02.urdf
               urdfs/iCubGenova04.urdf
               urdfs/iCubGazeboV2_5.urdf)

set(URDF_HUMAN_FILES urdfs/Marta48DoF.urdf
                     urdfs/Marta66DoF.urdf
                     urdfs/Meri48DoF.urdf
                     urdfs/Meri66DoF.urdf
                     urdfs/Claudia66DoF.urdf
                     urdfs/Claudia48DoF_Dec2017.urdf
                     urdfs/Claudia66DoF_Dec2017.urdf)

set(JOINT_INI_HUMAN_FILES urdfs/config/48DoFModel.ini
                          urdfs/config/66DoFModel.ini)

yarp_install(FILES ${URDF_FILES}
             DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/human-dynamic-estimation/urdfs)

yarp_install(FILES ${URDF_HUMAN_FILES}
             DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/human-dynamic-estimation/urdfs)

yarp_install(FILES ${JOINT_INI_HUMAN_FILES}
             DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/human-dynamic-estimation/urdfs/config)

# application files
# For now we don't install it as it must be configured on a cluster-basis
# and then we have duplicates in the Application manager
# set(APP_FILES conf/HumanDynamicsEstimation.xml)
# yarp_install(FILES ${APP_FILES}
#              DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})

# ROS related installation
set (ROS_PROJECT_NAME HDERviz)
set (ROS_PACKAGE_FILE ros/package.xml)
set (ROS_LAUNCH_FILE ros/launch/HDERviz.launch)
set (ROS_RVIZ_FILE ros/rviz/HDERviz.rviz)

# Copy the package.xml, which is necessary to be treated as
# a ROS package
install(FILES ${ROS_PACKAGE_FILE}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${ROS_PROJECT_NAME}/)


# Install the .launch file and the config file
install(FILES ${ROS_LAUNCH_FILE}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${ROS_PROJECT_NAME}/launch/)

install(FILES ${ROS_RVIZ_FILE}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${ROS_PROJECT_NAME}/rviz/)

# Install urdf files
install(FILES ${URDF_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${ROS_PROJECT_NAME}/urdfs/)

install(FILES ${URDF_HUMAN_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${ROS_PROJECT_NAME}/urdfs/)
