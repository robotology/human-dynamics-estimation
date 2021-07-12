/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <iDynTree/Visualizer.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include <Wearable/IWear/IWear.h>

using namespace yarp::os;

std::atomic<bool> isClosing{false};

void my_handler(int)
{
    isClosing = true;
}

#ifdef WIN32

#include <windows.h>

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType) {
            // Handle the CTRL-C signal.
        case CTRL_C_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            my_handler(0);
            return TRUE;

        // Handle all other events
        default:
            return FALSE;
    }
}
#endif

void handleSigInt()
{
#ifdef WIN32
    SetConsoleCtrlHandler(CtrlHandler, TRUE);
#else
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = &my_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGABRT, &action, NULL);
#endif
}

struct FrameViewer
{
    std::shared_ptr<FrameViewer> parent{nullptr};
    std::string name;
    size_t vizIndex;
    iDynTree::Transform transform;
};

int main(int argc, char* argv[])
{
    std::string portNameIn = "WearableData:I";
    std::string portName = "WearableData";
    std::string namePrefix = "/WearableViewer";
    std::string logPrefix = "WearableDataVisualizer";

    // prepare and configure the resource finder
    yarp::os::ResourceFinder& config = yarp::os::ResourceFinder::getResourceFinderSingleton();

    config.setDefaultConfigFile("HapticGloveFramesVisualizationConfig.ini");

    config.configure(argc, argv);

    // =========================
    // Start the visualizer
    // =========================
    iDynTree::Visualizer visualizer;

    bool ok = visualizer.init();

    visualizer.camera().animator()->enableMouseControl();

    if (!ok) {
        std::cerr << "Failed to initialize the visualizer." << std::endl;
        return EXIT_FAILURE;
    }
    // =========================
    // Open the Drivers
    // =========================
    yarp::dev::PolyDriver tfDriver, iWearDriver;
    wearable::IWear* iWear;

    yarp::os::Property wearOptions, tfClientCfg;

    // Iwear Remapper
    yarp::os::Value* wearableDataPort;
    if (!config.check("wearable_data_ports", wearableDataPort)) {
        yError() << logPrefix << "Unable to find wearable_data_ports into config file.";
        return EXIT_FAILURE;
    }

    wearOptions.put("device", "iwear_remapper");
    wearOptions.put("wearableDataPorts", wearableDataPort);

    if (!iWearDriver.open(wearOptions)) {
        yError() << "Unable to open polydriver with the following options:"
                 << wearOptions.toString();
        return EXIT_FAILURE;
    }

    if (!iWearDriver.view(iWear) || iWear == nullptr) {
        yError() << "Unable to view Wearable Remapper interface.";
        return EXIT_FAILURE;
    }

    // =========================
    // Check the wearable driver
    // =========================
    while (iWear->getStatus() == wearable::WearStatus::WaitingForFirstRead) {
        yInfo() << logPrefix << "IWear interface waiting for first data. Waiting...";
        yarp::os::Time::delay(0.1);
    }

    if (iWear->getStatus() != wearable::WearStatus::Ok) {
        yError() << logPrefix << "The status of the attached IWear interface is not ok ("
                 << static_cast<int>(iWear->getStatus()) << ")";
        return EXIT_FAILURE;
    }

    // =========================
    // create the map of link names and wearble sensors
    // =========================
    std::vector<std::string> linkNameList;
    yarp::os::Bottle* linkListYarp;
    if (!(config.check("link_names_wearables") && config.find("link_names_wearables").isList())) {
        yError() << logPrefix << "Unable to find link_names_wearables in the config file.";
        return EXIT_FAILURE;
    }
    linkListYarp = config.find("link_names_wearables").asList();

    for (size_t i = 0; i < linkListYarp->size(); i++) {
        linkNameList.push_back(linkListYarp->get(i).asString());
    }

    std::map<std::string, wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>>
        linkSensorsMap;

    for (auto linkName : linkNameList) {
        auto sensor = iWear->getVirtualLinkKinSensor(linkName);
        if (!sensor) {
            yError() << logPrefix << "Failed to find sensor associated to link" << linkName
                     << "from the IWear interface";
            return EXIT_FAILURE;
        }
        linkSensorsMap[linkName] = sensor;
        yInfo() << logPrefix
                << "sensor name to add the sensors vector: " << sensor->getSensorName();
    }
    yInfo() << logPrefix << "linkSensorsMap.size(): " << linkSensorsMap.size();

    // =========================
    // Get the fixed frames
    // =========================
    std::vector<std::string> newFramesList;
    yarp::os::Bottle* newFramesListYarp;
    if (!(config.check("new_fixed_frames") && config.find("new_fixed_frames").isList())) {
        yError() << logPrefix << "Unable to find new_fixed_frames in the config file.";
        return EXIT_FAILURE;
    }
    newFramesListYarp = config.find("new_fixed_frames").asList();

    for (size_t i = 0; i < linkListYarp->size(); i++) {
        newFramesList.push_back(newFramesListYarp->get(i).asString());
    }

    // =========================
    // Get the frames map
    // =========================
    std::vector<std::string> FramesMapList;
    yarp::os::Bottle* FramesMapYarp;
    if (!(config.check("frames_map") && config.find("frames_map").isList())) {
        yError() << logPrefix << "Unable to find frames_map in the config file.";
        return EXIT_FAILURE;
    }
    FramesMapYarp = config.find("frames_map").asList();

    // =========================
    // Make Frame Viewers
    // =========================
    std::vector<std::shared_ptr<FrameViewer>> frames;

    // create all the frame viwers
    for (size_t i = 0; i < FramesMapYarp->size(); i++) {
        std::shared_ptr<FrameViewer> frame = std::make_shared<FrameViewer>();
        yarp::os::Bottle* frameMapYarp = FramesMapYarp->get(i).asList();
        if (frameMapYarp->size() != 5) {
            yError() << logPrefix << "the map does not have expected size. frameMap: ("
                     << frameMapYarp->toString() << ") , expected size: " << 5
                     << " , actual size: " << frameMapYarp->size();
            return EXIT_FAILURE;
        }
        frame->name = frameMapYarp->get(0).asString();
        frame->vizIndex = visualizer.frames().addFrame(iDynTree::Transform::Identity(),
                                                       frameMapYarp->get(2).asDouble());
        //      visualizer.frames()
        //          .getFrameLabel(frame->vizIndex)
        //          ->setText(frame->name); // to be merged
        frames.push_back(frame);
    }

    // check for the parent frame
    for (size_t i = 0; i < FramesMapYarp->size(); i++) {
        yarp::os::Bottle* frameMapYarp = FramesMapYarp->get(i).asList();
        if (frameMapYarp->size() != 5) {
            yError() << logPrefix << "the map does not have expected size. frameMap: ("
                     << frameMapYarp->toString() << ") , expected size: " << 5
                     << " , actual size: " << frameMapYarp->size();
            return EXIT_FAILURE;
        }

        std::string parentName = frameMapYarp->get(1).asString();
        std::string Name = frameMapYarp->get(0).asString();
        if (frames[i]->name != Name) {
            yError() << logPrefix
                     << "the frame viewer name is not equal to the one from the "
                        "frame map list. Frame Viewer Name: "
                     << frames[i]->name << " , yarp map name: " << Name;
            return EXIT_FAILURE;
        }

        for (auto& frame : frames) {
            if (frame->name == parentName) {
                frames[i]->parent = frame;
                break;
            }
        }

        // check for the fixed transformations

        iDynTree::Transform transformation;
        transformation.setPosition(iDynTree::Position::Zero());
        transformation.setRotation(iDynTree::Rotation::Identity()); // x is front (north), z is up.

        yarp::os::Bottle* positionList = frameMapYarp->get(3).asList();
        yarp::os::Bottle* quatList = frameMapYarp->get(4).asList();
        if (positionList->size() == 3) {
            iDynTree::Position position(positionList->get(0).asDouble(),
                                        positionList->get(1).asDouble(),
                                        positionList->get(2).asDouble());
            transformation.setPosition(position);
        }
        if (quatList->size() == 4) {
            iDynTree::Vector4 quat;
            quat[0] = quatList->get(0).asDouble();
            quat[1] = quatList->get(1).asDouble();
            quat[2] = quatList->get(2).asDouble();
            quat[3] = quatList->get(3).asDouble();

            // normalize the quaternion
            double norm = 0;
            for (size_t j = 0; j < 4; j++)
                norm += quat[j] * quat[j];

            norm = std::sqrt(norm);

            for (size_t j = 0; j < 4; j++)
                quat[j] = quat[j] / norm;

            iDynTree::Rotation rotation;
            rotation.fromQuaternion(quat); //(real: w, imaginary: x y z)
            transformation.setRotation(rotation);
        }
        frames[i]->transform = transformation;
    }

    // print frames info
    yInfo() << "Frames information:";
    for (auto& frame : frames) {
        yInfo() << "name: " << frame->name;
        if (frame->parent)
            yInfo() << "parent: " << frame->parent->name;

        yInfo() << "initial transformation from the parent: \n" << frame->transform.toString();
    }
    yInfo() << "***********";

    // =========================
    // Visualization loop
    // =========================

    iDynTree::Transform frameTransform;
    iDynTree::Rotation frameRotation;
    iDynTree::Vector4 quat;

    while (visualizer.run() && !isClosing) {

        for (auto& frame : frames) {
            if (frame->parent) { // if not it is inertial frame

                // link
                auto sensor = linkSensorsMap[frame->name];
                if (sensor) {
                    wearable::Quaternion orientation;
                    wearable::Vector3 position;
                    if (sensor->getSensorStatus() != wearable::WearStatus::Ok) {
                        yWarning()
                            << "senor status is not OK, sensor name:" << sensor->getSensorName();
                        break;
                    }
                    sensor->getLinkPose(position, orientation);
                    iDynTree::Position framePosition(position[0], position[1], position[2]);
                    quat[0] = orientation[0];
                    quat[1] = orientation[1];
                    quat[2] = orientation[2];
                    quat[3] = orientation[3];
                    frameRotation.fromQuaternion(quat); //(real: w, imaginary: x y z)
                    frameTransform.setPosition(framePosition);
                    frameTransform.setRotation(frameRotation);
                    frame->transform = frame->parent->transform * frameTransform;
                }
                visualizer.frames().updateFrame(frame->vizIndex, frame->transform);
            }
        }
        visualizer.draw();
    }

    return EXIT_SUCCESS;
}
