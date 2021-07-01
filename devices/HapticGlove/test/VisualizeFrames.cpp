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

void my_handler(int) { isClosing = true; }

#ifdef WIN32

#include <windows.h>

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType) {
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

void handleSigInt() {
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

struct FrameViewier {
  std::shared_ptr<FrameViewier> parent{nullptr};
  std::string name;
  size_t vizIndex;
  iDynTree::Transform transform;
};

int main(int argc, char *argv[]) {
  std::string portNameIn = "WearableData:I";
  std::string portName = "WearableData";
  std::string namePrefix = "/WearableViewer";
  std::string logPrefix = "WearableDataVisualizer";

  // prepare and configure the resource finder
  yarp::os::ResourceFinder &config =
      yarp::os::ResourceFinder::getResourceFinderSingleton();

  config.setDefaultConfigFile("config.ini");

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
  wearable::IWear *iWear;

  yarp::os::Property wearOptions, tfClientCfg;

  // Iwear Remapper
  yarp::os::Value *wearableDataPort;
  if (!config.check("wearable_data_ports", wearableDataPort)) {
    yError() << logPrefix
             << "Unable to find wearable_data_ports into config file.";
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
    yInfo() << logPrefix
            << "IWear interface waiting for first data. Waiting...";
    yarp::os::Time::delay(0.1);
  }

  if (iWear->getStatus() != wearable::WearStatus::Ok) {
    yError() << logPrefix
             << "The status of the attached IWear interface is not ok ("
             << static_cast<int>(iWear->getStatus()) << ")";
    return EXIT_FAILURE;
  }

  // =========================
  // create the vector of link sensors
  // =========================
  std::vector<std::string> linkNameList;
  yarp::os::Bottle *linkListYarp;
  if (!(config.check("linkNames") && config.find("linkNames").isList())) {
    yError() << logPrefix << "Unable to find linkNames in the config file.";
    return EXIT_FAILURE;
  }
  linkListYarp = config.find("linkNames").asList();

  for (size_t i = 0; i < linkListYarp->size(); i++) {
    linkNameList.push_back(linkListYarp->get(i).asString());
  }

  std::map<std::string,
           wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>>
      linkSensorsMap;

  for (auto linkName : linkNameList) {
    auto sensor = iWear->getVirtualLinkKinSensor(linkName);
    if (!sensor) {
      yError() << logPrefix << "Failed to find sensor associated to link"
               << linkName << "from the IWear interface";
      return EXIT_FAILURE;
    }
    linkSensorsMap[linkName] = sensor;
    yInfo() << logPrefix << "sensor name to add the sensors vector: "
            << sensor->getSensorName();
  }

  yInfo() << logPrefix << "linkSensorsMap.size(): " << linkSensorsMap.size();

  // =========================
  // Frames
  // =========================
  std::vector<std::shared_ptr<FrameViewier>> frames;

  // inertial and root frame
  iDynTree::Transform inertialFrame;
  inertialFrame.setPosition(iDynTree::Position::Zero());
  inertialFrame.setRotation(
      iDynTree::Rotation::Identity()); // x is front (north), z is up.

  std::shared_ptr<FrameViewier> rootFrame = std::make_shared<FrameViewier>();
  rootFrame->name = "inertialFrame";
  rootFrame->transform = inertialFrame;
  rootFrame->vizIndex =
      visualizer.frames().addFrame(iDynTree::Transform::Identity());
  //      visualizer.frames()
  //          .getFrameLabel(rootFrame->vizIndex)
  //          ->setText(rootFrame->name); // to be merged
  frames.push_back(rootFrame);

  // left hand
  iDynTree::Transform leftHandRootFrame;
  iDynTree::Rotation rootLeftHandRotation;
  rootLeftHandRotation.zero(); // x is toward left, z is upward
  rootLeftHandRotation(0, 1) = 1.0;
  rootLeftHandRotation(1, 0) = -1.0;
  rootLeftHandRotation(2, 2) = 1.0;
  iDynTree::Position rootLeftHandPosition(0.0, 0.2, 0.0);

  leftHandRootFrame.setPosition(rootLeftHandPosition);
  leftHandRootFrame.setRotation(rootLeftHandRotation);

  std::shared_ptr<FrameViewier> leftHandRootFrameViewer =
      std::make_shared<FrameViewier>();
  leftHandRootFrameViewer->name = "leftHandRootFrame";
  leftHandRootFrameViewer->transform = leftHandRootFrame;
  leftHandRootFrameViewer->vizIndex =
      visualizer.frames().addFrame(iDynTree::Transform::Identity());
  //      visualizer.frames()
  //          .getFrameLabel(rootFrame->vizIndex)
  //          ->setText(rootFrame->name); // to be merged
  frames.push_back(leftHandRootFrameViewer);

  // other frames
  for (auto const &[name, sensor] : linkSensorsMap) {
    std::shared_ptr<FrameViewier> frame = std::make_shared<FrameViewier>();
    frame->name = sensor->getSensorName();
    if (name.find("l_") != std::string::npos) {
      yInfo() << logPrefix << name << "is related to left hand.";
      frame->parent = leftHandRootFrameViewer;
      frame->transform = leftHandRootFrame;
    }
    frame->vizIndex =
        visualizer.frames().addFrame(iDynTree::Transform::Identity());
    //      visualizer.frames()
    //          .getFrameLabel(rootFrame->vizIndex)
    //          ->setText(rootFrame->name); // to be merged
    frames.push_back(frame);
  }

  iDynTree::Transform frameTransform;
  iDynTree::Rotation frameRotation;
  iDynTree::Vector4 quat;

  // =========================
  // Visualization loop
  // =========================
  while (visualizer.run() && !isClosing) {
    for (auto &frame : frames) {
      if (frame->parent) {
        // link
        auto sensor = linkSensorsMap[frame->name];
        if (sensor) {
          wearable::Quaternion orientation;
          wearable::Vector3 position;
          sensor->getLinkPose(position, orientation);
          iDynTree::Position framePosition(position[0], position[1],
                                           position[2]);

          quat[0] = orientation[0];
          quat[1] = orientation[1];
          quat[2] = orientation[2];
          quat[3] = orientation[3];
          frameRotation.fromQuaternion(quat); //(real: w, imaginary: x y z)

          frameTransform.setPosition(framePosition);
          frameTransform.setRotation(frameRotation);
          frame->transform = frame->parent->transform * frameTransform;
          visualizer.frames().updateFrame(frame->vizIndex, frame->transform);
        }
      }
    }

    visualizer.draw();
  }

  return EXIT_SUCCESS;
}
