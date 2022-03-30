/*
     * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia
     *
     * Licensed under either the GNU Lesser General Public License v3.0 :
     * https://www.gnu.org/licenses/lgpl-3.0.html
     * or the GNU Lesser General Public License v2.1 :
     * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
     * at your option.
     */

#include <hde/interfaces/IHumanState.h>
#include <hde/interfaces/IWearableTargets.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Visualizer.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <iostream>
#include <chrono>
#include <cmath>
#include <thread>
#include <csignal>
#include <unordered_map>

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
        return EXIT_FAILURE;
    }
    std::string urdfFile = rf.find("modelURDFName").asString();

    bool ignoreMissingLinks;
    if( !(rf.check("ignoreMissingLinks") && rf.find("ignoreMissingLinks").isBool()) ) 
    {
        yWarning() << LogPrefix << "'ignoreMissingLinks' option not found or not valid. It is set to False.";
        ignoreMissingLinks = false;
    }
    else
    {
        ignoreMissingLinks = rf.find("ignoreMissingLinks").asBool();
    }

    bool visualizeWrenches;
    if( !(rf.check("visualizeWrenches") && rf.find("visualizeWrenches").isBool()) ) 
    {
        yWarning() << LogPrefix << "'visualizeWrenches' option not found or not valid. It is set to False.";
        visualizeWrenches = false;
    }
    else
    {
        visualizeWrenches = rf.find("visualizeWrenches").asBool();
    }

    bool visualizeFrames;
    if( !(rf.check("visualizeFrames") && rf.find("visualizeFrames").isBool()) ) 
    {
        yWarning() << LogPrefix << "'visualizeFrames' option not found or not valid. It is set to False.";
        visualizeFrames = false;
    }
    visualizeFrames = rf.find("visualizeFrames").asBool();

    bool visualizeTargets;
    if( !(rf.check("visualizeTargets") && rf.find("visualizeTargets").isBool()) ) 
    {
        yWarning() << LogPrefix << "'visualizeTargets' option not found or not valid. It is set to False.";
        visualizeTargets = false;
    }
    visualizeTargets = rf.find("visualizeTargets").asBool();

    iDynTree::Position cameraDeltaPosition;
    if( !(rf.check("cameraDeltaPosition") && rf.find("cameraDeltaPosition").isList() && rf.find("cameraDeltaPosition").asList()->size() == 3) ) 
    {
        yError() << LogPrefix << "'cameraDeltaPosition' option not found or not valid.";
        return EXIT_FAILURE;
    }
    for (size_t idx = 0; idx < 3; idx++)
    {
        if ( !(rf.find("cameraDeltaPosition").asList()->get(idx).isFloat64()) )
        {
            yError() << LogPrefix << "'cameraDeltaPosition' entry [ " << idx << " ] is not valid.";
            return EXIT_FAILURE;
        }
        cameraDeltaPosition.setVal(idx, rf.find("cameraDeltaPosition").asList()->get(idx).asFloat64());
    }

    if( !(rf.check("useFixedCamera") && rf.find("useFixedCamera").isBool()) ) 
    {
        yError() << LogPrefix << "'useFixedCamera' option not found or not valid.";
        return EXIT_FAILURE;
    }
    bool useFixedCamera = rf.find("useFixedCamera").asBool();

    iDynTree::Position fixedCameraTarget = iDynTree::Position(0, 0, 0);
    if (useFixedCamera) 
    {
        if( !(rf.check("fixedCameraTarget") && rf.find("fixedCameraTarget").isList() && rf.find("fixedCameraTarget").asList()->size() == 3) ) 
        {
            yError() << LogPrefix << "'fixedCameraTarget' option not found or not valid.";
            return EXIT_FAILURE;
        }
        for (size_t idx = 0; idx < 3; idx++)
        {
            if ( !(rf.find("fixedCameraTarget").asList()->get(idx).isFloat64()) )
            {
                yError() << LogPrefix << "'fixedCameraTarget' entry [ " << idx << " ] is not valid.";
                return EXIT_FAILURE;
            }
            fixedCameraTarget.setVal(idx, rf.find("fixedCameraTarget").asList()->get(idx).asFloat64());
        }
    }

    if( !(rf.check("maxVisualizationFPS") && rf.find("maxVisualizationFPS").isInt32() && rf.find("maxVisualizationFPS").asInt32() > 0) ) 
    {
        yError() << LogPrefix << "'maxVisualizationFPS' option not found or not valid.";
        return EXIT_FAILURE;
    }
    unsigned int maxVisualizationFPS = rf.find("maxVisualizationFPS").asInt32();

    if( !(rf.check("humanStateDataPortName") && rf.find("humanStateDataPortName").isString()) ) 
    {
        yError() << LogPrefix << "'humanStateDataPortName' option not found or not valid.";
        return EXIT_FAILURE;
    }
    std::string humanStateDataPortName = rf.find("humanStateDataPortName").asString();

    // Visualize Wrenches Options
    std::string humanWrenchWrapperPortName;
    std::vector<std::string> wrenchSourceLinks;
    if (visualizeWrenches)
    {
        if( !(rf.check("humanWrenchWrapperPortName") && rf.find("humanWrenchWrapperPortName").isString()) )
        {
            yError() << LogPrefix << "'humanWrenchWrapperPortName' option not found or not valid. Wrench Visualization will be disabled";
            visualizeWrenches = false;
        }
        else
        {
            humanWrenchWrapperPortName = rf.find("humanWrenchWrapperPortName").asString();
        }
    }

    if (visualizeWrenches)
    {
        if( !(rf.check("wrenchSourceLinks") && rf.find("wrenchSourceLinks").isList()) )
        {
            yError() << LogPrefix << "'wrenchSourceLinks' option not found or not valid. Wrench Visualization will be disabled";
            visualizeWrenches = false;
        }
        else
        {
            auto wrenchSourceLinksList = rf.find("wrenchSourceLinks").asList();
            for (size_t it = 0; it < wrenchSourceLinksList->size(); it++)
            {
                if(!wrenchSourceLinksList->get(it).isString())
                {
                    yError() << LogPrefix << "in 'wrenchSourceLinks' there is a field that is not a string.";
                    return EXIT_FAILURE;
                }
                wrenchSourceLinks.push_back(wrenchSourceLinksList->get(it).asString());
            }
        }
    }
    int numberOfWrenchElements = 6 * wrenchSourceLinks.size();

    double forceScalingFactor;
    if (visualizeWrenches)
    {
        if ( !(rf.check("forceScalingFactor") && rf.find("forceScalingFactor").isFloat64()) )
        {
            yError() << LogPrefix << "'forceScalingFactor' option not found or not valid.";
            return EXIT_FAILURE;
        }
        forceScalingFactor = rf.find("forceScalingFactor").asFloat64();
    }

    // Visualize Frames Options
    std::vector<std::string> visualizedLinksFrame;
    if (visualizeFrames)
    {
        if( !(rf.check("visualizedLinksFrame") && rf.find("visualizedLinksFrame").isList()) )
        {
            yError() << LogPrefix << "'visualizedLinksFrame' option not found or not valid. Frames Visualization will be disabled";
            visualizeFrames = false;
        }
        else
        {
            auto visualizedLinksFrameList = rf.find("visualizedLinksFrame").asList();
            for (size_t it = 0; it < visualizedLinksFrameList->size(); it++)
            {
                if(!visualizedLinksFrameList->get(it).isString())
                {
                    yError() << LogPrefix << "in 'visualizedLinksFrame' there is a field that is not a string.";
                    return EXIT_FAILURE;
                }
                visualizedLinksFrame.push_back(visualizedLinksFrameList->get(it).asString());
            }
        }
    }

    double linksFrameScalingFactor = 1.0;
    if (visualizeFrames)
    {
        if ( !(rf.check("linksFrameScalingFactor") && rf.find("linksFrameScalingFactor").isDouble()) )
        {
            yWarning() << LogPrefix << "'linksFrameScalingFactor' option not found or not valid. Using default scale factor = 1.0";
        }
        else
        {
            linksFrameScalingFactor = rf.find("linksFrameScalingFactor").asDouble();
        }
    }

    // Visualize Targets Options
    std::vector<hde::TargetName> visualizedTargetsFrame;
    if (visualizeTargets)
    {
        if( !(rf.check("visualizedTargetsFrame") && rf.find("visualizedTargetsFrame").isList()) )
        {
            yError() << LogPrefix << "'visualizedTargetsFrame' option not found or not valid. Frames Visualization will be disabled";
            visualizeTargets = false;
        }
        else
        {
            auto visualizedTargetsFrameList = rf.find("visualizedTargetsFrame").asList();
            for (size_t it = 0; it < visualizedTargetsFrameList->size(); it++)
            {
                if(!visualizedTargetsFrameList->get(it).isString())
                {
                    yError() << LogPrefix << "in 'visualizedTargetsFrame' there is a field that is not a string.";
                    return EXIT_FAILURE;
                }
                visualizedTargetsFrame.push_back(visualizedTargetsFrameList->get(it).asString());
            }
        }
    }

    double targetsFrameScalingFactor = 1.0;
    if (visualizeTargets)
    {
        if ( !(rf.check("targetsFrameScalingFactor") && rf.find("targetsFrameScalingFactor").isDouble()) )
        {
            yWarning() << LogPrefix << "'targetsFrameScalingFactor' option not found or not valid. Using default scale factor = 1.0";
        }
        else
        {
            targetsFrameScalingFactor = rf.find("targetsFrameScalingFactor").asDouble();
        }
    }

    std::string wearableTargetsWrapperPortName;
    if (visualizeTargets)
    {
        if( !(rf.check("wearableTargetsWrapperPortName") && rf.find("wearableTargetsWrapperPortName").isString()) )
        {
            yError() << LogPrefix << "'wearableTargetsWrapperPortName' option not found or not valid. Wrench Visualization will be disabled";
            visualizeTargets = false;
        }
        else
        {
            wearableTargetsWrapperPortName = rf.find("wearableTargetsWrapperPortName").asString();
        }
    }


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

    // check if wrench sources are found in the model and save the link index
    std::vector<iDynTree::LinkIndex> wrenchSourceLinkIndices;
    if (visualizeWrenches)
    {
        for (auto wrenchSourceLink : wrenchSourceLinks)
        {
            auto frameIndex = model.getLinkIndex(wrenchSourceLink);
            if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
            {
                yError() << LogPrefix << "wrench source link [ " << wrenchSourceLink << " ] not found in the visualized model";
                return EXIT_FAILURE;
            }
            wrenchSourceLinkIndices.push_back(frameIndex);
        }
    }

    // check if links are found in the model and save the link index
    std::vector<iDynTree::LinkIndex> visualizedLinkIndices;
    if (visualizeFrames)
    {
        for (auto visualizedLink : visualizedLinksFrame)
        {
            auto frameIndex = model.getLinkIndex(visualizedLink);
            if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
            {
                yError() << LogPrefix << "visualized link [ " << visualizedLink << " ] not found in the visualized model";
                return EXIT_FAILURE;
            }
            visualizedLinkIndices.push_back(frameIndex);
        }
    }

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

    // initialize iWearableTargets interface from remapper
    std::unordered_map<hde::TargetName, std::shared_ptr<hde::WearableSensorTarget>> targets;
    yarp::dev::PolyDriver wearableTargetsRemapperDevice;
    if (visualizeTargets) {
        hde::interfaces::IWearableTargets* iWearableTargets{nullptr};

        yarp::os::Property wearableTargetsRemapperOptions;
        wearableTargetsRemapperOptions.put("device", "wearable_targets_remapper");
        wearableTargetsRemapperOptions.put("wearableTargetsDataPort", wearableTargetsWrapperPortName);
        std::cerr << "my wrapper port name is: "  << wearableTargetsWrapperPortName << std::endl;

        if(!wearableTargetsRemapperDevice.open(wearableTargetsRemapperOptions))
        {
            yError() << LogPrefix << "Failed to connect remapper device";
            return EXIT_FAILURE;
        }
        if(!wearableTargetsRemapperDevice.view(iWearableTargets) || !iWearableTargets )
        {
            yError() << LogPrefix << "Failed to view iWearableTargets interface";
            return EXIT_FAILURE;
        }

        // wait for the iWearableTargets to be initialized
        std::this_thread::sleep_for(std::chrono::seconds(1));
        while (iWearableTargets->getAllTargetsName().empty())
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            yInfo() << LogPrefix << "Waiting for data from WearableTargetsRemapper";
        }

        // Verify the selected targets exist, and that the associated link exists in the visualized model
        // and create a vector with the targets
        auto targetsName = iWearableTargets->getAllTargetsName();
        for (auto visualizedTargetFrame : visualizedTargetsFrame)
        {
            auto target = iWearableTargets->getTarget(visualizedTargetFrame);

            // check if target exists
            if (!target)
            {
                yError() << LogPrefix << "target [ " << visualizedTargetFrame << " ] not found in iWearableTargets." ;
                return EXIT_FAILURE;
            }

            // check if frame exists
            auto frameIndex = model.getLinkIndex(target.get()->modelLinkName);
            if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
            {
                yError() << LogPrefix << "target link [ " << target.get()->modelLinkName << " ] not found in the visualized model";
                return EXIT_FAILURE;
            }

            // add the target to the vector
            targets.emplace(visualizedTargetFrame, target);
        }
    }

    // initialize wrench port
    yarp::os::BufferedPort<yarp::sig::Vector> wrenchPort;
    yarp::sig::Vector* wrenchMeasuresVector;
    if (visualizeWrenches)
    {
        wrenchPort.open("/HumanStateVisualizer" + humanWrenchWrapperPortName);
        if (wrenchPort.isClosed())
        {
            yError() << LogPrefix << "failed to open the port /HumanStateVisualizer" << humanWrenchWrapperPortName;
            return EXIT_FAILURE;
        }
        if (!yarp.connect(humanWrenchWrapperPortName, wrenchPort.getName()))
        {
            yError() << LogPrefix << "failed to connect to the port"  << humanWrenchWrapperPortName;
            return EXIT_FAILURE;
        }
        wrenchMeasuresVector = wrenchPort.read(true);
        if (wrenchMeasuresVector == nullptr)
        {
            yError() << LogPrefix << "no data coming from the port " << humanWrenchWrapperPortName;
            return EXIT_FAILURE;
        }
        if(wrenchMeasuresVector->size() != (numberOfWrenchElements) )
        {
            yError() << LogPrefix << "expected " << numberOfWrenchElements << " elements in port " << humanWrenchWrapperPortName
                    << ", received " << wrenchMeasuresVector->size();
            return EXIT_FAILURE;
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
    iDynTree::Transform linkTransform;
    iDynTree::Vector3 gravityVector;
    iDynTree::Direction force;

    iDynTree::Transform wHb = iDynTree::Transform::Identity();
    iDynTree::VectorDynSize joints(model.getNrOfDOFs());
    joints.zero();

    // initialize visualization
    iDynTree::Visualizer viz;
    iDynTree::VisualizerOptions options;

    viz.init(options);

    viz.camera().setPosition(cameraDeltaPosition);
    viz.camera().setTarget(fixedCameraTarget);
    
    viz.setColorPalette("meshcat");

    viz.camera().animator()->enableMouseControl(true);
    
    viz.addModel(model, "human");

    if (visualizeWrenches)
    {
        for (size_t vectorIndex = 0; vectorIndex < wrenchSourceLinks.size(); vectorIndex++)
        {
            linkTransform = viz.modelViz("human").getWorldLinkTransform(wrenchSourceLinkIndices.at(vectorIndex));
            for (size_t i = 0; i < 3; i++)
            {
                force.setVal(i, forceScalingFactor * wrenchMeasuresVector->data()[6 * vectorIndex + i]);
            }
            force = linkTransform.getRotation() * force;
            viz.vectors().addVector(linkTransform.getPosition(), force);
        }
    }

    if (visualizeFrames)
    {
        for (size_t vectorIndex = 0; vectorIndex < visualizedLinksFrame.size(); vectorIndex++)
        {
            linkTransform = viz.modelViz("human").getWorldLinkTransform(visualizedLinksFrame.at(vectorIndex));
            viz.frames().addFrame(linkTransform, linksFrameScalingFactor);
        }
    }

    if (visualizeTargets)
    {
        for (auto targetEntity : targets)
        {
            if ( targetEntity.second.get()->targetType == hde::KinematicTargetType::pose ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::poseAndVelocity ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::position ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::positionAndVelocity ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::orientation ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::orientationAndVelocity ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::floorContact)
            {
                linkTransform = iDynTree::Transform(targetEntity.second.get()->getCalibratedRotation(), iDynTree::Position(targetEntity.second.get()->getCalibratedPosition()));
                viz.frames().addFrame(linkTransform, targetsFrameScalingFactor);
            }
            if ( targetEntity.second.get()->targetType == hde::KinematicTargetType::gravity )
            {
                linkTransform = iDynTree::Transform(targetEntity.second.get()->getCalibratedRotation(), iDynTree::Position(targetEntity.second.get()->getCalibratedPosition()));
                iDynTree::toEigen(gravityVector) = targetsFrameScalingFactor * iDynTree::toEigen(linkTransform.getRotation()).row(2);
                viz.vectors().addVector(linkTransform.getPosition(), gravityVector );
            }
            
        }
    }

    // start Visualization
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastViz = std::chrono::steady_clock::now();

    long minimumMicroSecViz = std::round(1e6 / (double) maxVisualizationFPS);

    std::cerr << "RUNNING VIZ!" << std::endl;

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

        viz.modelViz("human").setPositions(wHb, joints);

        size_t vectorsIterator = 0;
        if (visualizeWrenches)
        {
            wrenchMeasuresVector = wrenchPort.read(true);
            for (size_t vectorIndex = 0; vectorIndex < wrenchSourceLinks.size(); vectorIndex++)
            {
                linkTransform = viz.modelViz("human").getWorldLinkTransform(wrenchSourceLinkIndices.at(vectorIndex));
                for (size_t i = 0; i < 3; i++)
                {
                    force.setVal(i, forceScalingFactor * wrenchMeasuresVector->data()[6 * vectorIndex + i]);
                }
                force = linkTransform.getRotation() * force;
                viz.vectors().updateVector(vectorIndex, linkTransform.getPosition(), force);
                vectorsIterator++;
            }
        }

        size_t framesIterator = 0;
        if (visualizeFrames)
        {
            for (size_t vectorIndex = 0; vectorIndex < visualizedLinksFrame.size(); vectorIndex++)
            {
                linkTransform = viz.modelViz("human").getWorldLinkTransform(visualizedLinksFrame.at(vectorIndex));
                viz.frames().updateFrame(vectorIndex, linkTransform);
                framesIterator++;
            }
        }

        if (visualizeTargets)
        {
            for (auto targetEntity : targets)
            {
                switch (targetEntity.second.get()->targetType)
                {
                case hde::KinematicTargetType::pose: case hde::KinematicTargetType::poseAndVelocity: 
                    linkTransform = iDynTree::Transform(targetEntity.second.get()->getCalibratedRotation(), iDynTree::Position(targetEntity.second.get()->getCalibratedPosition()));
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                    break;
                case hde::KinematicTargetType::position: case hde::KinematicTargetType::positionAndVelocity:
                    linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                    linkTransform.setPosition(iDynTree::Position(targetEntity.second.get()->getCalibratedPosition()));
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                    break;
                case hde::KinematicTargetType::orientation: case hde::KinematicTargetType::orientationAndVelocity:
                    linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                    linkTransform.setRotation(targetEntity.second.get()->getCalibratedRotation());
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                    break;
                case hde::KinematicTargetType::gravity:
                    linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                    iDynTree::toEigen(gravityVector) = targetsFrameScalingFactor * (iDynTree::toEigen(linkTransform.getRotation()) * iDynTree::toEigen(targetEntity.second.get()->getCalibratedRotation()).row(2).transpose());
                    viz.vectors().updateVector(vectorsIterator, linkTransform.getPosition(), gravityVector);
                    vectorsIterator++;
                    break;
                case hde::KinematicTargetType::floorContact:
                    linkTransform = iDynTree::Transform(targetEntity.second.get()->getCalibratedRotation(), iDynTree::Position(targetEntity.second.get()->getCalibratedPosition()));
                    linkTransform.setPosition(iDynTree::Position(linkTransform.getPosition().getVal(0), linkTransform.getPosition().getVal(1), linkTransform.getPosition().getVal(1)));
        
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                    break;
                default:
                    linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                }
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
        
        // Update the visualizer
        viz.draw();
        lastViz = std::chrono::steady_clock::now();
    }

    viz.close();
    remapperDevice.close();
    wearableTargetsRemapperDevice.close();
    wrenchPort.close();

    return 0;
}
