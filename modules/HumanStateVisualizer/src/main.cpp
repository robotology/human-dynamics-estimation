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

const std::string ModuleName = "HumanStateVisualizer";
const std::string LogPrefix = ModuleName + " :";

int main()
{

    // load model
    const std::string urdfFileName = "humanSubject01_66dof.urdf";

    auto& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string urdfFilePath = rf.findFile(urdfFileName);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << urdfFileName;
        return EXIT_FAILURE;
    }

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << urdfFilePath;
        return EXIT_FAILURE;
    }

    // initialize visualization
    iDynTree::Visualizer viz;
    iDynTree::VisualizerOptions options;

    viz.init(options);

    viz.camera().setPosition(iDynTree::Position(2, 0, 0.5));
    viz.camera().setTarget(iDynTree::Position(0, 0, 0));

    viz.addModel(modelLoader.model(), "human");

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
    remapperOptions.put("humanStateDataPort", "/HDE/HumanStateWrapper/state:o");

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

    yInfo() << LogPrefix << "Human State Interface providing data from the following [ " << iHumanState->getJointNames().size() << " ] joints:";
    for (auto jointName : iHumanState->getJointNames())
    {
        yInfo() << LogPrefix << jointName;
    }

    // initialize state variables for visualization
    iDynTree::Transform wHb = iDynTree::Transform::Identity();
    iDynTree::VectorDynSize joints(viz.modelViz("human").model().getNrOfDOFs());
    joints.zero();

    // start Visualization
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastViz = std::chrono::steady_clock::now();

    unsigned int maxVizFPS = 65;
    long minimumMicroSecViz = std::round(1e6 / (double) maxVizFPS);

    while(viz.run())
    {
        now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::microseconds>(now - lastViz).count() < minimumMicroSecViz)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // update the joints present in iHumanState
        std::array<double, 3> basePositionInterface = iHumanState->getBasePosition();
        std::array<double, 4> baseOrientationInterface = iHumanState->getBaseOrientation();
        std::vector<double> jointPositionsInterface = iHumanState->getJointPositions();
        std::vector<std::string> jointNames = iHumanState->getJointNames();
        std::string baseName = iHumanState->getBaseName();

        iDynTree::Vector4 quaternion;
        quaternion.setVal(0, baseOrientationInterface.at(0));
        quaternion.setVal(1, baseOrientationInterface.at(1));
        quaternion.setVal(2, baseOrientationInterface.at(2));
        quaternion.setVal(3, baseOrientationInterface.at(3));

        iDynTree::Position position;
        position.setVal(0, basePositionInterface.at(0));
        position.setVal(1, basePositionInterface.at(1));
        position.setVal(2, basePositionInterface.at(2));

        wHb.setRotation(iDynTree::Rotation::RotationFromQuaternion(quaternion));
        wHb.setPosition(position);

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

        viz.camera().setPosition(position + iDynTree::Position(2, 0, 0.5));
        viz.camera().setTarget(position);
        

        // Update the visulizer
        viz.modelViz("human").setPositions(wHb, joints);
        viz.draw();
        lastViz = std::chrono::steady_clock::now();
    }

    remapperDevice.close();

    return 0;
}
