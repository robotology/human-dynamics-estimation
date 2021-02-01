/*
     * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
     *
     * Licensed under either the GNU Lesser General Public License v3.0 :
     * https://www.gnu.org/licenses/lgpl-3.0.html
     * or the GNU Lesser General Public License v2.1 :
     * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
     * at your option.
     */

#include "IHumanState.h"

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Visualizer.h>

#include <iostream>
#include <chrono>
#include <cmath>
#include <thread>
#include <csignal>

const std::string ModuleName = "HumanStateVisualizer";
const std::string LogPrefix = ModuleName + " :";

std::atomic<bool> isClosing{false};

void my_handler(int signal)
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

int main(int argc, char* argv[])
{

    // Listen to signals for closing in a clean way the application
    handleSigInt();

    // parse the configuraiton options
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("HumanStateVisualizer.ini");
    rf.configure(argc, argv);
    
    if (rf.isNull())
    {
        yError() << LogPrefix << "Empty configuration file.";
        return EXIT_FAILURE;
    }

    if( !(rf.check("modelURDFName") && rf.find("modelURDFName").isString()) ) 
    {
        yError() << LogPrefix << "'modelURDFName' option not found or not valid.";
    }
    std::string urdfFile = rf.find("modelURDFName").asString();

    if( !(rf.check("ignoreMissingLinks") && rf.find("ignoreMissingLinks").isBool()) ) 
    {
        yError() << LogPrefix << "'ignoreMissingLinks' option not found or not valid.";
    }
    bool ignoreMissingLinks = rf.find("ignoreMissingLinks").asBool();

    iDynTree::Position cameraDeltaPosition;
    if( !(rf.check("cameraDeltaPosition") && rf.find("cameraDeltaPosition").isList() && rf.find("cameraDeltaPosition").asList()->size() == 3) ) 
    {
        yError() << LogPrefix << "'cameraDeltaPosition' option not found or not valid.";
    }
    for (size_t idx = 0; idx < 3; idx++)
    {
        if ( !(rf.find("cameraDeltaPosition").asList()->get(idx).isDouble()) )
        {
            yError() << LogPrefix << "'cameraDeltaPosition' entry [ " << idx << " ] is not valid.";
        }
        cameraDeltaPosition.setVal(idx, rf.find("cameraDeltaPosition").asList()->get(idx).asDouble());
    }

    if( !(rf.check("useFixedCamera") && rf.find("useFixedCamera").isBool()) ) 
    {
        yError() << LogPrefix << "'useFixedCamera' option not found or not valid.";
    }
    bool useFixedCamera = rf.find("useFixedCamera").asBool();

    iDynTree::Position fixedCameraTarget;
    if (useFixedCamera) 
    {
        if( !(rf.check("fixedCameraTarget") && rf.find("fixedCameraTarget").isList() && rf.find("fixedCameraTarget").asList()->size() == 3) ) 
        {
            yError() << LogPrefix << "'fixedCameraTarget' option not found or not valid.";
        }
        for (size_t idx = 0; idx < 3; idx++)
        {
            if ( !(rf.find("fixedCameraTarget").asList()->get(idx).isDouble()) )
            {
                yError() << LogPrefix << "'fixedCameraTarget' entry [ " << idx << " ] is not valid.";
            }
            fixedCameraTarget.setVal(idx, rf.find("fixedCameraTarget").asList()->get(idx).asDouble());
        }
    }

    if( !(rf.check("maxVisualizationFPS") && rf.find("maxVisualizationFPS").isInt() && rf.find("maxVisualizationFPS").asInt() > 0) ) 
    {
        yError() << LogPrefix << "'maxVisualizationFPS' option not found or not valid.";
    }
    unsigned int maxVisualizationFPS = rf.find("maxVisualizationFPS").asInt();

    if( !(rf.check("humanStateDataPortName") && rf.find("humanStateDataPortName").isString()) ) 
    {
        yError() << LogPrefix << "'humanStateDataPortName' option not found or not valid.";
    }
    std::string humanStateDataPortName = rf.find("humanStateDataPortName").asString();


    // load model
    std::string urdfFilePath = rf.findFile(urdfFile);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << urdfFile;
        return EXIT_FAILURE;
    }

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << urdfFilePath;
        return EXIT_FAILURE;
    }

    iDynTree::Model model = modelLoader.model();

    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<< LogPrefix <<"[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }


    // initialize iHumanState interface from remapper
    yarp::dev::PolyDriver remapperDevice;
    hde::interfaces::IHumanState* iHumanState{nullptr};

    yarp::os::Property remapperOptions;
    remapperOptions.put("device", "human_state_remapper");
    remapperOptions.put("humanStateDataPort", humanStateDataPortName);

    if(!remapperDevice.open(remapperOptions))
    {
        yError() << LogPrefix << "Failed to connect remapper device";
        return EXIT_FAILURE;
    }
    if(!remapperDevice.view(iHumanState) || !iHumanState )
    {
        yError() << LogPrefix << "Failed to view iHumanState interface";
        return EXIT_FAILURE;
    }

    // wait for the iHumanState to be initialized
    while (iHumanState->getBaseName().empty())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        yInfo() << LogPrefix << "Waiting for data from HumanStateRemapper";
    }
     
    // compare the iHumanState base and joint names with the visualization model
    yInfo() << LogPrefix << "Human State Interface providing data for the base link [ " << iHumanState->getBaseName() << " ]";
    if ( iHumanState->getBaseName() != model.getLinkName(model.getDefaultBaseLink()))
    {
        model.setDefaultBaseLink(model.getLinkIndex(iHumanState->getBaseName()));
        yInfo() << LogPrefix << "Default base link of the visualized model is changed to " << iHumanState->getBaseName();
    }
    yInfo() << LogPrefix << "Human State Interface providing data from [ " << iHumanState->getJointNames().size() << " ] joints";
    
    for (auto jointName : iHumanState->getJointNames())
    {
        if (model.getJointIndex(jointName) == iDynTree::JOINT_INVALID_INDEX)
        {
            if (!ignoreMissingLinks)
            {
                yError() << LogPrefix << "joint [ " << jointName << " ] not found in the visualized model." 
                         << " Set ignoreMissingLinks true if you want to ignore the missing joints.";
                return EXIT_FAILURE;
            }
            yWarning() << LogPrefix << "joint [ " << jointName << " ] not found in the visualized model, "
                       << "the joint will be ignored.";
        }
    }

    // initialize state variables for visualization
    std::array<double, 3> basePositionInterface;
    std::array<double, 4> baseOrientationInterface;
    std::vector<double> jointPositionsInterface;
    std::vector<std::string> jointNames;
    iDynTree::Vector4 baseOrientationQuaternion;
    iDynTree::Position basePosition;
    iDynTree::Position basePositionOld = fixedCameraTarget;

    iDynTree::Transform wHb = iDynTree::Transform::Identity();
    iDynTree::VectorDynSize joints(model.getNrOfDOFs());
    joints.zero();

    // initialize visualization
    iDynTree::Visualizer viz;
    iDynTree::VisualizerOptions options;

    viz.init(options);

    viz.camera().setPosition(cameraDeltaPosition);
    viz.camera().setTarget(fixedCameraTarget);

    viz.camera().animator()->enableMouseControl(true);
    
    viz.addModel(model, "human");

    // start Visualization
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastViz = std::chrono::steady_clock::now();

    long minimumMicroSecViz = std::round(1e6 / (double) maxVisualizationFPS);

    while(viz.run() && !isClosing)
    {
        now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::microseconds>(now - lastViz).count() < minimumMicroSecViz)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // read values from iHumanState
        basePositionInterface = iHumanState->getBasePosition();
        baseOrientationInterface = iHumanState->getBaseOrientation();
        jointPositionsInterface = iHumanState->getJointPositions();
        jointNames = iHumanState->getJointNames();

        baseOrientationQuaternion.setVal(0, baseOrientationInterface.at(0));
        baseOrientationQuaternion.setVal(1, baseOrientationInterface.at(1));
        baseOrientationQuaternion.setVal(2, baseOrientationInterface.at(2));
        baseOrientationQuaternion.setVal(3, baseOrientationInterface.at(3));

        basePosition.setVal(0, basePositionInterface.at(0));
        basePosition.setVal(1, basePositionInterface.at(1));
        basePosition.setVal(2, basePositionInterface.at(2));

        wHb.setRotation(iDynTree::Rotation::RotationFromQuaternion(baseOrientationQuaternion));
        wHb.setPosition(basePosition);

        for (size_t iHumanInterfaceIdx = 0; iHumanInterfaceIdx < jointNames.size(); iHumanInterfaceIdx++)
        {
            std::string jointName = jointNames.at(iHumanInterfaceIdx);
            double jointVal = jointPositionsInterface.at(iHumanInterfaceIdx);

            iDynTree::JointIndex jointIndex = viz.modelViz("human").model().getJointIndex(jointName);
            if (jointIndex != iDynTree::JOINT_INVALID_INDEX)
            {
                joints.setVal(jointIndex, jointVal);
            }
        }

        // follow the desired link with the camera
        if ( !useFixedCamera )
        {
            cameraDeltaPosition = viz.camera().getPosition() - basePositionOld;
            viz.camera().setPosition(basePosition + cameraDeltaPosition);
            viz.camera().setTarget(basePosition);

            basePositionOld = basePosition;
        }
        
        

        // Update the visulizer
        viz.modelViz("human").setPositions(wHb, joints);
        viz.draw();
        lastViz = std::chrono::steady_clock::now();
    }

    viz.close();
    remapperDevice.close();

    return 0;
}
