# install urdf files
set(URDF_FILES urdfs/iCubGenova02.urdf
               urdfs/iCubGenova04.urdf)

set(URDF_HUMAN_FILES urdfs/Marta48DoF.urdf
                     urdfs/Marta66DoF.urdf
                     urdfs/Meri48DoF.urdf
                     urdfs/Meri66DoF.urdf
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

