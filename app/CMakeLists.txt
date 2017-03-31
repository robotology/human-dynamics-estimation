# install urdf files
set(URDF_FILES urdfs/iCubGenova02.urdf
               urdfs/iCubGenova04.urdf)

set(URDF_HUMAN_FILES urdfs/Marta48DoF.urdf
                     urdfs/Marta66DoF.urdf
                     urdfs/Meri48DoF.urdf
                     urdfs/Meri66DoF.urdf)

yarp_install(FILES ${URDF_FILES}
             DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/urdfs/human-dynamic-estimation)

yarp_install(FILES ${URDF_HUMAN_FILES}
             DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/urdfs/human-dynamic-estimation)


# application files
# For now we don't install it as it must be configured on a cluster-basis
# and then we have duplicates in the Application manager
# set(APP_FILES conf/HumanDynamicsEstimation.xml)
# yarp_install(FILES ${APP_FILES}
#              DESTINATION ${YARP_APPLICATIONS_INSTALL_DIR})

