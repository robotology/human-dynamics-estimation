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
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/LogStream.h>


#include <iostream>
#include <cstdlib>
#include <memory>
#include <csignal>

#include <Wearable/IWear/IWear.h>

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


struct FrameViewier
{
    std::shared_ptr<FrameViewier> parent{nullptr};
    std::string name;
    size_t vizIndex;
    iDynTree::Transform transform;
};

int main(int /*argc*/, char** /*argv*/)
{
    std::string portNameIn="WearableData:I";
    std::string portName="WearableData";

    wearable::IWearableDevice m_wearable;

    yarp::os::Property prop;
    prop.put("device", "iwear_remapper");
    prop.put("local",  "/HapticGlove/WearableDataRightHand");
    prop.put("remote", "/transformServer");

    yarp::dev::PolyDriver driver;
    yarp::dev::IFrameTransform* tfReader;

    if (!driver.open(prop))
    {
        yError() << "Unable to open polydriver with the following options:" << tfClientCfg.toString();
        return EXIT_FAILURE;
    }

    if (!driver.view(m_wearable) || m_wearable == nullptr)
    {
        yError() << "Unable to view IFrameTransform interface.";
        return EXIT_FAILURE;
    }


//    yarp::os::BufferedPort<wearable::IWear> m_port;

//    if (!m_port.open(portName))
//      {
//          yError() << portName << " port is already open.";
//          return false;
//      }
//    yarp::os::Network::connect(portNameIn, portName);

//    wearable::IWear* imuVal = m_port.read(false);
//    wearable::sensor::SensorName sensorName="palmIMU";
//    wearable::sensor linkSensor= imuVal->getVirtualLinkKinSensor(sensorName);
//    wearable::Quaternion orienation;
//    linkSensor= linkSensor.getLinkOrientation(orienation);

//    yInfo()<<"imuData:: " <<imuVal->getWearableName();


    /*

    std::string namePrefix = "/SenseGloveIMUViewer";
    // Visualize the model
    iDynTree::Visualizer visualizer;

    bool ok = visualizer.init();

    visualizer.camera().animator()->enableMouseControl();

    if( !ok )
    {
        std::cerr << "Failed to initialize the visualizer." << std::endl;
        return EXIT_FAILURE;
    }

    yarp::os::Property tfClientCfg;
    tfClientCfg.put("device", "transformClient");
    tfClientCfg.put("local",  namePrefix +"/tf");
    tfClientCfg.put("remote", "/transformServer");

    yarp::dev::PolyDriver driver;
    yarp::dev::IFrameTransform* tfReader;

    if (!driver.open(tfClientCfg))
    {
        yError() << "Unable to open polydriver with the following options:" << tfClientCfg.toString();
        return EXIT_FAILURE;
    }

    if (!driver.view(tfReader) || tfReader == nullptr)
    {
        yError() << "Unable to view IFrameTransform interface.";
        return EXIT_FAILURE;
    }

    iDynTree::Rotation senseGloveInertialRotation;
    senseGloveInertialRotation.zero();
    senseGloveInertialRotation(0,2) = -1.0; //-z is forward
    senseGloveInertialRotation(1,0) = -1.0; //-x is left
    senseGloveInertialRotation(2,1) =  1.0; // +y is up
    iDynTree::Transform SenseGloveInertial;
    SenseGloveInertial.setPosition(iDynTree::Position::Zero());
    SenseGloveInertial.setRotation(senseGloveInertialRotation);

    std::vector<std::shared_ptr<FrameViewier>> frames;

    std::shared_ptr<FrameViewier> rootFrame = std::make_shared<FrameViewier>();
    rootFrame->name = "senseglove_origin";
    rootFrame->transform = SenseGloveInertial;
    rootFrame->vizIndex = visualizer.frames().addFrame(iDynTree::Transform::Identity());
//    visualizer.frames().getFrameLabel(rootFrame->vizIndex)->setText(rootFrame->name);
    frames.push_back(rootFrame);

    std::shared_ptr<FrameViewier> leftHandFrame = std::make_shared<FrameViewier>();
    leftHandFrame->name = "senseglove_left_hand";
    leftHandFrame->parent = rootFrame;
    leftHandFrame->vizIndex = visualizer.frames().addFrame(iDynTree::Transform::Identity());
//    visualizer.frames().getFrameLabel(leftHandFrame->vizIndex)->setText(leftHandFrame->name);
    frames.push_back(leftHandFrame);

    std::shared_ptr<FrameViewier> rightHandFrame = std::make_shared<FrameViewier>();
    rightHandFrame->name = "senseglove_right_hand";
    rightHandFrame->parent = rootFrame;
    rightHandFrame->vizIndex = visualizer.frames().addFrame(iDynTree::Transform::Identity());
//    visualizer.frames().getFrameLabel(rightHandFrame->vizIndex)->setText(rightHandFrame->name);
    frames.push_back(rightHandFrame);

    iDynTree::Transform transformBuffer;
    yarp::sig::Matrix matrixBuffer;
    matrixBuffer.resize(4, 4);

    // Visualization loop
    while( visualizer.run() && !isClosing )
    {
        for (auto& frame: frames)
        {
            if (frame->parent)
            {
                if (tfReader->getTransform(frame->name, frame->parent->name, matrixBuffer))
                {
                    iDynTree::toiDynTree(matrixBuffer, transformBuffer);
                    frame->transform = frame->parent->transform * transformBuffer;
                    visualizer.frames().updateFrame(frame->vizIndex, frame->transform);
                }
            }
        }

        visualizer.draw();
    }

    */
    return EXIT_SUCCESS;
}
