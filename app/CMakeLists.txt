# HDE xml files installation
set (WEARABLES_PROJECT_NAME Wearables)
set (WEARABLES_XML_FILES xml/FTShoeLeftWearableDevice.xml
                   xml/FTShoeRightWearableDevice.xml
                   xml/FTShoesWearableDevice.xml
                   xml/ICubWearableDevice.xml
                   xml/SkinInsolesWearableDevice.xml
                   xml/XsensSuitWearableDevice.xml

                   xml/applications/WearableDevices.xml)

install(FILES ${WEARABLES_XML_FILES}
             DESTINATION ${CMAKE_INSTALL_DATADIR}/${WEARABLES_PROJECT_NAME})
