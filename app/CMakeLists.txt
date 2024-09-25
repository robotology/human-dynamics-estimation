# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# HDE xml files installation

set (WEARABLES_XML_FILES xml/FTShoeLeftWearableDevice.xml
                   xml/FTShoeRightWearableDevice.xml
                   xml/FTShoesWearableDevice.xml
                   xml/ICubWearableDevice.xml
                   xml/SkinInsolesWearableDevice.xml
                   xml/XsensSuitWearableDevice.xml
                   xml/applications/WearableDevices-Dumper.xml
                   xml/applications/HumanDynamicsEstimation-WearableDumper.xml
                   xml/applications/WearableDevices.xml)

install(FILES ${WEARABLES_XML_FILES}
             DESTINATION ${CMAKE_INSTALL_DATADIR}/HumanDynamicsEstimation)
