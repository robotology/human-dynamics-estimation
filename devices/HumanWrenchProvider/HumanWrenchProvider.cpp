/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanWrenchProvider.h"
#include "WrenchFrameTransformers.h"

#include <hde/interfaces/IHumanState.h>

#include <Wearable/IWear/IWear.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Estimation/BerdySparseMAPSolver.h>
#include <iDynTree/Estimation/BerdyHelper.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcServer.h>

#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>

const std::string DeviceName = "HumanWrenchProvider";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;
using namespace hde::devices::impl;

struct AnalogSensorData
{
    size_t numberOfChannels = 0;
    std::vector<double> measurements;
};

enum class WrenchSourceType
{
    Fixed,
    Dummy,
    Robot, // TODO
};

enum rpcCommand
{
    empty,
    setWorldWrench,
};

/** 
 * This structure holds the parameters used for the Non-Collocated Wrench Estimation.
 */
struct MAPEstParams
{
    bool useMAPEst;
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> priorMeasurementsCovariance; // Sigma_y
    double priorDynamicsRegularizationExpected; // mu_d
    double priorDynamicsRegularizationCovarianceValue; // Sigma_d
    // measurements params
    double measurementDefaultCovariance;
    std::unordered_map<std::string, std::vector<double>> specificMeasurementsCovariance;
};

struct MAPEstHelper
{
    iDynTree::BerdyOptions berdyOptions;
    iDynTree::BerdyHelper berdyHelper;
    std::unique_ptr<iDynTree::BerdySparseMAPSolver> mapSolver;
};

struct WrenchSourceData
{
    std::string name;
    WrenchSourceType type;

    std::string outputFrame;
    IWrenchFrameTransformer frameTransformer;

    wearable::sensor::SensorName sensorName;
    wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor> ftWearableSensor;

    iDynTree::Wrench wrench;

    // Variables for Robot Human Transformation
    std::string humanLinkingFrame;
    std::string robotLinkingFrame;
    std::string robotSourceOriginFrame;
    std::string robotSourceOrientationFrame;

    iDynTree::Position humanFootPosition;
};

class HumanWrenchProvider::Impl
{
public:
    // constructor
    Impl();

    mutable std::mutex mutex;

    //pHRI scenario flag
    bool pHRIScenario;
    iDynTree::Transform robotHumanFeetFixedTransform;
    
    // Read human data
    bool readHumanData = false;

    //Gravity variable
    iDynTree::Vector3 world_gravity;

    //Attached interfaces
    wearable::IWear* iWear = nullptr;
    hde::interfaces::IHumanState* iHumanState = nullptr;

    AnalogSensorData analogSensorData;
    std::vector<WrenchSourceData> wrenchSources;

    // Human variables
    iDynTree::Model humanModel;
    // buffer variables (iDynTree)
    iDynTree::VectorDynSize humanJointPositionsVec;
    iDynTree::VectorDynSize humanJointVelocitiesVec;
    iDynTree::Transform basePose;
    iDynTree::Twist baseVelocity;

    // KinDynComputation
    iDynTree::KinDynComputations humanKinDynComp;

    // Robot variables
    iDynTree::Model robotModel;
    std::vector<std::string> robotJointsName;
    iDynTree::VectorDynSize robotJointPositionsVec;
    iDynTree::VectorDynSize robotJointVelocitiesVec;
    std::vector<std::string> robotJointNamesListFromConfig;
    wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualJointKinSensor> robotJointWearableSensors;

    // Rpc
    class CmdParser;
    std::unique_ptr<CmdParser> commandPro;
    yarp::os::RpcServer rpcPort;
    bool applyRpcCommand();
    
    // MAP estimator variables
    MAPEstParams mapEstParams;
    MAPEstHelper mapEstHelper;
    std::vector<iDynTree::Wrench> transformedWrenches;
};

HumanWrenchProvider::HumanWrenchProvider()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new Impl()}
{}

// Without this destructor here, the linker complains for
// undefined reference to vtable
HumanWrenchProvider::~HumanWrenchProvider() = default;

// ===============
// RPC PORT PARSER
// ===============

class HumanWrenchProvider::Impl::CmdParser : public yarp::os::PortReader
{
public:
    std::atomic<rpcCommand> cmdStatus{rpcCommand::empty};
    std::atomic<double> wrench_fx;
    std::atomic<double> wrench_fy;
    std::atomic<double> wrench_fz;
    std::atomic<double> wrench_tx;
    std::atomic<double> wrench_ty;
    std::atomic<double> wrench_tz;

    void resetInternalVariables()
    {
        cmdStatus = rpcCommand::empty;
        wrench_fx = wrench_fy = wrench_fz = 0;
        wrench_tx = wrench_ty = wrench_tz = 0;
    }

    bool read(yarp::os::ConnectionReader& connection) override
    {
        yarp::os::Bottle command, response;
        if (command.read(connection)) {
            if (command.get(0).asString() == "setWorldWrench") {
                this->cmdStatus = rpcCommand::setWorldWrench;
                this->wrench_fx = command.get(1).asFloat64();
                this->wrench_fy = command.get(2).asFloat64();
                this->wrench_fz = command.get(3).asFloat64();
                this->wrench_tx = command.get(4).asFloat64();
                this->wrench_ty = command.get(5).asFloat64();
                this->wrench_tz = command.get(6).asFloat64();
            }
            else {
                response.addString(
                    "Entered command is incorrect. Enter help to know available commands");
            }
        }
        else {
            resetInternalVariables();
            return false;
        }

        yarp::os::ConnectionWriter* reply = connection.getWriter();

        if (reply != NULL) {
            response.write(*reply);
        }
        else
            return false;

        return true;
    }

};

bool parseRotation(yarp::os::Bottle* list, iDynTree::Rotation& rotation)
{
    if (list->size() != 9) {
        yError() << LogPrefix << "The list with rotation data does not contain 9 elements";
        return false;
    }

    rotation = iDynTree::Rotation(list->get(0).asFloat64(),
                                  list->get(1).asFloat64(),
                                  list->get(2).asFloat64(),
                                  list->get(3).asFloat64(),
                                  list->get(4).asFloat64(),
                                  list->get(5).asFloat64(),
                                  list->get(6).asFloat64(),
                                  list->get(7).asFloat64(),
                                  list->get(8).asFloat64());
    return true;
}

bool parsePosition(yarp::os::Bottle* list, iDynTree::Position& position)
{
    if (list->size() != 3) {
        yError() << LogPrefix << "The list with position data does not contain 9 elements";
        return false;
    }

    position = iDynTree::Position(
        list->get(0).asFloat64(), list->get(1).asFloat64(), list->get(2).asFloat64());
    return true;
}

bool parseWrench(yarp::os::Bottle* list, iDynTree::Wrench& wrench)
{
    if (list->size() != 6) {
        yError() << LogPrefix << "The list with wrench data does not contain 6 elements";
        return false;
    }

    wrench = iDynTree::Wrench(iDynTree::Force(list->get(0).asFloat64(), list->get(1).asFloat64(), list->get(2).asFloat64()),
                                iDynTree::Torque(list->get(3).asFloat64(), list->get(4).asFloat64(), list->get(5).asFloat64()));

    return true;
}

bool parseMeasurementsCovariance(yarp::os::Searchable& config, MAPEstParams& mapEstParams)
{
    // find the group cov_measurements_NET_EXT_WRENCH_SENSOR
    yarp::os::Bottle& priorMeasurementsCovarianceBottle = config.findGroup("cov_measurements_NET_EXT_WRENCH_SENSOR");
    if(priorMeasurementsCovarianceBottle.isNull())
    {
        //TODO use default values?
        return false;
    }
    
    // find the default value
    if(!priorMeasurementsCovarianceBottle.check("value"))
    {
        //TODO
        return false;
    }
    mapEstParams.measurementDefaultCovariance = priorMeasurementsCovarianceBottle.find("value").asFloat64();  

    // find the elements with specific covariance
    yarp::os::Value& specificElements = priorMeasurementsCovarianceBottle.find("specificElements");
    if(specificElements.isNull())
    {
        yInfo()<<"No specific elements to specify the measurements covariance were found! Using"<<mapEstParams.measurementDefaultCovariance<<"for all";
    }
    if(!specificElements.isList())
    {
        yError()<<"Parameter specificElements must be a list!";
        return false;
    }

    yarp::os::Bottle *specificElementsList = specificElements.asList();
    if(!specificElements.isNull())
    {
        for(int i=0; i<specificElementsList->size(); i++)
        {
            std::string linkName = specificElementsList->get(i).asString();
            if(!priorMeasurementsCovarianceBottle.check(linkName))
            {
                yError()<<"The element"<<linkName<<"appears in the specificElements list, but its covariance is not specified!";
                return false;
            }

            // get the diagonal values of the covariance
            yarp::os::Bottle* covarianceList = priorMeasurementsCovarianceBottle.find(linkName).asList();
            if(covarianceList==nullptr)
            {
                yError()<<"The"<<linkName<<"parameter must be a list!";
                return false;
            }
            if(covarianceList->size()!=6)
            {
                yError()<<"The covariance values of"<<linkName<<"must have 6 parameters!";
                return false;
            }
            std::vector<double> covarianceListValues;
            for(int j=0; j<covarianceList->size(); j++)
            {
                covarianceListValues.push_back(covarianceList->get(j).asFloat64());
            }
            mapEstParams.specificMeasurementsCovariance.emplace(linkName, covarianceListValues);

        }

    }

    // get the RCM sensor covariance (if specified)
    yarp::os::Value& rcmPriorMeasurementsCovarianceValue  = config.find("cov_measurements_RCM_SENSOR");
    std::vector<double> rcmCovariance;
    if(rcmPriorMeasurementsCovarianceValue.isNull())
    {
        yInfo()<<LogPrefix<<"Using default value"<<mapEstParams.measurementDefaultCovariance<<"for the RCM sensor";
        for(int i=0; i<6; i++) rcmCovariance.push_back(mapEstParams.measurementDefaultCovariance);
    }
    else
    {
        if(!rcmPriorMeasurementsCovarianceValue.isList())
        {
            yError()<<LogPrefix<<"Param cov_measurements_RCM_SENSOR must be a list!";
            return false;
        }
        yarp::os::Bottle* rcmPriorMeasurementsCovarianceList = rcmPriorMeasurementsCovarianceValue.asList();
        if(rcmPriorMeasurementsCovarianceList->size()!=6)
        {
            yError()<<LogPrefix<<"Param cov_measurements_RCM_SENSOR must have 6 values!";
            return false;
        }

        for(int i=0; i<rcmPriorMeasurementsCovarianceList->size(); i++)
        {
            rcmCovariance.push_back(rcmPriorMeasurementsCovarianceList->get(i).asFloat64());
        }
    }

    mapEstParams.specificMeasurementsCovariance.emplace("RCM_SENSOR", rcmCovariance);

    return true;
}

bool parseMAPEstParams(yarp::os::Searchable& config, MAPEstParams& mapEstParams)
{
    // find the param group
    yarp::os::Bottle& mapEstParamsGroup = config.findGroup("MAPEstParams");
    if(mapEstParamsGroup.isNull())
    {
        mapEstParams.useMAPEst = false;
        return true;
    }

    // parse the useMAPEst param
    if(!mapEstParamsGroup.check("useMAPEst"))
    {
        yError() << "Missing useMAPEst parameter!";
        return false;
    }
    mapEstParams.useMAPEst = mapEstParamsGroup.find("useMAPEst").asBool();

    // parse the prior dynamic variable covariance
    if(mapEstParams.useMAPEst)
    {
        // parse the prior DynamicsVariableCovariance
        if(!mapEstParamsGroup.check("mu_dyn_variables") ||
            !mapEstParamsGroup.find("mu_dyn_variables").isFloat64())
        {
            yError() << "Missing valid floating point mu_dyn_variables parameter!";
            return false;
        }
        mapEstParams.priorDynamicsRegularizationExpected = mapEstParamsGroup.find("mu_dyn_variables").asFloat64();

        if(!mapEstParamsGroup.check("cov_dyn_variables") ||
            !mapEstParamsGroup.find("cov_dyn_variables").isFloat64())
        {
            yError() << "Missing valid floating point cov_dyn_variables parameter!";
            return false;
        }
        mapEstParams.priorDynamicsRegularizationCovarianceValue = mapEstParamsGroup.find("cov_dyn_variables").asFloat64();

        // parse the prior measurements covariance Sigma_y
        if(!parseMeasurementsCovariance(mapEstParamsGroup, mapEstParams))
        {
            return false;
        }
    }

    return true;
}

iDynTree::SpatialForceVector computeRCMInBaseUsingMeasurements(iDynTree::KinDynComputations& kinDynComputations,
                                                      iDynTree::LinkIndex baseIdx)
{
    //TODO
    iDynTree::SpatialForceVector rcm;
    rcm.zero();
    //return rcm;
    
    iDynTree::SpatialForceVector subjectWeight;
    subjectWeight.zero();
    subjectWeight.setVal(2, 9.81 * kinDynComputations.model().getTotalMass()); //TODO set weight from parameter
    yDebug()<<LogPrefix<<"Mass of the model:"<<kinDynComputations.model().getTotalMass();

    iDynTree::Transform centroidal_H_world;
    centroidal_H_world.setPosition(kinDynComputations.getCenterOfMassPosition());
    centroidal_H_world.setRotation(iDynTree::Rotation::Identity());

    iDynTree::SpatialForceVector subjectWeightInBase = kinDynComputations.getWorldBaseTransform().inverse() * centroidal_H_world.inverse() * subjectWeight;

    return subjectWeightInBase;

    const iDynTree::Model& model = kinDynComputations.model();

    iDynTree::LinkPositions linkPos(model);
    iDynTree::LinkVelArray linkVels(model);
    iDynTree::LinkAccArray linkProperAccs(model);
    iDynTree::LinkInertias linkInertias(model);

    // Get links info
    for(size_t linkIdx = 0; linkIdx < model.getNrOfLinks(); linkIdx++)
    {
        linkInertias(linkIdx) = model.getLink(linkIdx)->getInertia();
        linkPos(linkIdx) = kinDynComputations.getWorldTransform(linkIdx);
        linkVels(linkIdx) = kinDynComputations.getFrameVel(linkIdx);
        
        //TODO manage accelerations
        linkProperAccs(linkIdx).zero();
    }

    // Set centroidal to world transform
    iDynTree::Transform world_H_centroidal = iDynTree::Transform(iDynTree::Rotation::Identity(), kinDynComputations.getCenterOfMassPosition());

    // Iterate over measurements
    for(iDynTree::LinkIndex visitedLinkIndex = 0; visitedLinkIndex < model.getNrOfLinks(); visitedLinkIndex++)
    {
        // Get world_H_link transform
        const iDynTree::Transform world_H_link = linkPos(visitedLinkIndex);

        // Compute link to centroidal transform
        const iDynTree::Transform centroidal_H_link = world_H_centroidal.inverse() * world_H_link;

        // Get link velocity L_v_A,L
        const iDynTree::Twist linkVelocityExpressedInLink = linkVels(visitedLinkIndex);

        // Get link acceleration L_a_A,L
        const iDynTree::SpatialAcc linkAccelerationExpressedInLink = linkProperAccs(visitedLinkIndex);

        // velocity contribution
        const iDynTree::SpatialForceVector linkInertiaMultipliedVelocity = linkInertias(visitedLinkIndex).multiply(linkVelocityExpressedInLink);
        const iDynTree::SpatialForceVector rcmLinkVelContrib = centroidal_H_link * (linkVelocityExpressedInLink.cross(linkInertiaMultipliedVelocity));
        
        // acceleration contribution
        const iDynTree::SpatialForceVector rcmLinkAccContrib = centroidal_H_link * (linkInertias(visitedLinkIndex).multiply(linkAccelerationExpressedInLink));

        // update rcm
        rcm = rcm + rcmLinkVelContrib + rcmLinkAccContrib;
    }

    // base_H_centroidal transform
    const iDynTree::Transform base_H_centroidal = kinDynComputations.getWorldBaseTransform().inverse() * world_H_centroidal;

    return base_H_centroidal * rcm;
}

bool HumanWrenchProvider::open(yarp::os::Searchable& config)
{
    //Set gravity vector
    pImpl->world_gravity.setVal(0, 0);
    pImpl->world_gravity.setVal(1, 0);
    pImpl->world_gravity.setVal(2, -9.81);

    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("pHRIScenario") && config.find("pHRIScenario").isBool())) {
        yError() << LogPrefix << "Option 'pHRIScenario' not found or not a valid bool";
        return false;
    }

    // Get pHRI scenario flag
    pImpl->pHRIScenario = config.find("pHRIScenario").asBool();

    // ==========================
    // INITIALIZE THE HUMAN MODEL
    // ==========================

    if (!(config.check("human_urdf") && config.find("human_urdf").isString())) {
        yError() << LogPrefix << "Option 'human_urdf' not found or not a valid file name";
        return false;
    }

    const std::string humanURDFFileName = config.find("human_urdf").asString();
    auto& human_rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string humanURDFFilePath = human_rf.findFile(humanURDFFileName);
    if (humanURDFFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("human_urdf").asString();
        return false;
    }

    iDynTree::ModelLoader humanModelLoader;
    if (!humanModelLoader.loadModelFromFile(humanURDFFilePath) || !humanModelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << humanURDFFilePath;
        return false;
    }

    pImpl->humanModel = humanModelLoader.model();
    pImpl->humanKinDynComp.loadRobotModel(pImpl->humanModel);

    // ==========================
    // INITIALIZE THE ROBOT MODEL
    // ==========================

    if (pImpl->pHRIScenario) {
        if (!(config.check("robot_urdf") && config.find("robot_urdf").isString())) {
            yError() << LogPrefix << "Option 'robot_urdf' not found or not a valid file name";
            return false;
        }

        const std::string robotURDFFileName = config.find("robot_urdf").asString();
        auto& robot_rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
        std::string robotURDFFilePath = robot_rf.findFile(robotURDFFileName);
        if (robotURDFFilePath.empty()) {
            yError() << LogPrefix << "Failed to find file" << config.find("robot_urdf").asString();
            return false;
        }

        iDynTree::ModelLoader robotModelLoader;
        if (!robotModelLoader.loadModelFromFile(robotURDFFilePath) || !robotModelLoader.isValid()) {
            yError() << LogPrefix << "Failed to load model" << robotURDFFilePath;
            return false;
        }

        pImpl->robotModel = robotModelLoader.model();
    }

    // ==================================
    // INITIALIZE THE ROBOT JOINT SOURCES
    // ==================================

    if (pImpl->pHRIScenario) {

        if (!(config.check("FeetFixedTransformRotation") && config.find("FeetFixedTransformRotation").isList())) {
            yError() << LogPrefix << "Option"
                        << " FeetFixedTransformRotation not found or not a valid list";
            return false;
        }

        if (!(config.check("FeetFixedTransformPosition") && config.find("FeetFixedTransformPosition").isList())) {
            yError() << LogPrefix << "Option"
                        << " FeetFixedTransformPosition not found or not a valid list";
            return false;
        }

        iDynTree::Rotation rotation;
        iDynTree::Position position;

        if (!parseRotation(config.find("FeetFixedTransformRotation").asList(),rotation)
                        || !parsePosition(config.find("FeetFixedTransformPosition").asList(),position)) {
                    yError() << LogPrefix << "Failed to parse"
                             << " FeetFixedTransformRotation or FeetFixedTransformPosition";
                    return false;
                }
        pImpl->robotHumanFeetFixedTransform = {rotation, position};


                
        yarp::os::Bottle& robotJointsGroup = config.findGroup("RobotJoints");
        if (robotJointsGroup.isNull()) {
            yError() << LogPrefix << "Failed to find RobotJoints group in the configuration file";
            return false;
        }

        for (size_t i = 1; i < robotJointsGroup.size(); i++) {
            if (!(robotJointsGroup.get(i).isList() && robotJointsGroup.get(i).asList()->size() == 2)) {
                yError() << LogPrefix << "Failed to load the robot part list";
                return false;
            }

            yarp::os::Bottle* partList = robotJointsGroup.get(i).asList();
            yarp::os::Bottle* jointsNameList = partList->get(1).asList();

            for (int j = 0; j < jointsNameList->size(); j++) {
                if (!jointsNameList->get(j).isString()) {
                    yError() << LogPrefix << "Failed to read the robot joint name, expecting a string";
                    return false;
                }
                pImpl->robotJointNamesListFromConfig.push_back(jointsNameList->get(j).asString());
            }

        }

    }

    // =============================
    // INITIALIZE THE WRENCH SOURCES
    // =============================

    if (!(config.check("number_of_sources") && config.find("number_of_sources").asInt32())) {
        yError() << LogPrefix << "Option 'number_of_sources' not found or not a valid Integer";
        return false;
    }

    if (!(config.check("sources") && config.find("sources").isList())) {
        yError() << LogPrefix << "Option 'sources' not found or not a valid list";
        return false;
    }

    yarp::os::Bottle* listOfSources = config.find("sources").asList();
    std::vector<std::string> sourcesNames;

    // Check the number of sources are correct
    if ( config.find("number_of_sources").asInt32() != listOfSources->size()) {
        yError() << LogPrefix << " 'number_of_sources' and 'sources' list provided are not matching";
        return false;
    }
    else {
        for (unsigned i = 0; i < listOfSources->size(); ++i) {
            sourcesNames.push_back(listOfSources->get(i).asString());
            yDebug() << LogPrefix << "Found source" << sourcesNames.back();
        }
    }

    // Parse the groups using the sources list names
    for (const auto& sourceName : sourcesNames) {
        yarp::os::Bottle& sourceGroup = config.findGroup(sourceName);

        if (sourceGroup.isNull()) {
            yError() << LogPrefix << "Failed to find group" << sourceName;
            return false;
        }

        // Temporary object
        WrenchSourceData WrenchSourceData;
        WrenchSourceData.name = sourceName;

        if (!(sourceGroup.check("sensorName") && sourceGroup.find("sensorName").isString())) {
            yError() << LogPrefix << "Option" << sourceName
                     << ":: sensorName not found or not a valid string";
            return false;
        }

        WrenchSourceData.sensorName = sourceGroup.find("sensorName").asString();

        if (!(sourceGroup.check("outputFrame") && sourceGroup.find("outputFrame").isString())) {
            yError() << LogPrefix << "Option" << sourceName
                     << ":: outputFrame not found or not a valid string";
            return false;
        }

        WrenchSourceData.outputFrame = sourceGroup.find("outputFrame").asString();

        if (!(sourceGroup.check("type") && sourceGroup.find("type").isString())) {
            yError() << LogPrefix << "Option" << sourceName
                     << ":: type not found or not a valid string";
            return false;
        }

        std::string sourceType = sourceGroup.find("type").asString();
        if (sourceType == "fixed") {
            WrenchSourceData.type = WrenchSourceType::Fixed;
        }
        else if (sourceType == "robot") {
            WrenchSourceData.type = WrenchSourceType::Robot;
            pImpl->readHumanData = true;
        }
        else if (sourceType == "dummy") {
            WrenchSourceData.type = WrenchSourceType::Dummy;
            pImpl->readHumanData = true;
        }
        else {
            yError() << LogPrefix << "Option" << sourceName
                     << ":: type must be either 'fixed' or 'robot'";
            return false;
        }

        switch (WrenchSourceData.type) {
                // Process Fixed source type
                // =========================

            case WrenchSourceType::Fixed: {

                if (!(sourceGroup.check("rotation") && sourceGroup.find("rotation").isList())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: rotation not found or not a valid list";
                    return false;
                }

                if (!(sourceGroup.check("position") && sourceGroup.find("position").isList())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: position not found or not a valid list";
                    return false;
                }

                iDynTree::Rotation rotation;
                iDynTree::Position position;
                if (!parseRotation(sourceGroup.find("rotation").asList(), rotation)
                    || !parsePosition(sourceGroup.find("position").asList(), position)) {
                    yError() << LogPrefix << "Failed to parse" << sourceName
                             << ":: position or rotation";
                    return false;
                }

                // Store the transform 
                WrenchSourceData.frameTransformer.transform = {rotation, position};

                yDebug() << LogPrefix << "=============:";
                yDebug() << LogPrefix << "New source   :" << WrenchSourceData.name;
                yDebug() << LogPrefix << "Sensor name  :" << WrenchSourceData.sensorName;
                yDebug() << LogPrefix << "Type         :" << sourceType;
                yDebug() << LogPrefix << "Output frame :" << WrenchSourceData.outputFrame;
                yDebug() << LogPrefix << "Rotation     :" << sourceGroup.find("rotation").asList()->toString();
                yDebug() << LogPrefix << "Position     :" << sourceGroup.find("position").asList()->toString();
                yDebug() << LogPrefix << "=============:";

                break;
            }

            // Process Robot source type
            // =========================
            case WrenchSourceType::Robot: {

                if (!(sourceGroup.check("humanLinkingFrame") && sourceGroup.find("humanLinkingFrame").isString())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: humanLinkingFrame not found or not a valid string";
                    return false;
                }

                WrenchSourceData.humanLinkingFrame = sourceGroup.find("humanLinkingFrame").asString();

                if (!(sourceGroup.check("robotLinkingFrame") && sourceGroup.find("robotLinkingFrame").isString())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: robotLinkingFrame not found or not a valid string";
                    return false;
                }

                WrenchSourceData.robotLinkingFrame = sourceGroup.find("robotLinkingFrame").asString();

                if (!(sourceGroup.check("robotSourceOriginFrame") && sourceGroup.find("robotSourceOriginFrame").isString())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: robotSourceOriginFrame not found or not a valid string";
                    return false;
                }

                WrenchSourceData.robotSourceOriginFrame = sourceGroup.find("robotSourceOriginFrame").asString();

                if (!(sourceGroup.check("robotSourceOrientationFrame") && sourceGroup.find("robotSourceOrientationFrame").isString())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: robotSourceOrientationFrame not found or not a valid string";
                    return false;
                }

                WrenchSourceData.robotSourceOrientationFrame = sourceGroup.find("robotSourceOrientationFrame").asString();

                if (!(sourceGroup.check("humanFootPosition") && sourceGroup.find("humanFootPosition").isList())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: humanFootPosition not found or not a valid list";
                    return false;
                }

                if (!parsePosition(sourceGroup.find("humanFootPosition").asList(),WrenchSourceData.humanFootPosition)) {
                    yError() << LogPrefix << "Failed to parse" << sourceName
                             << ":: humanFootPosition";
                    return false;
                }

                yDebug() << LogPrefix << "================================:";
                yDebug() << LogPrefix << "New source                      :" << WrenchSourceData.name;
                yDebug() << LogPrefix << "Sensor name                     :" << WrenchSourceData.sensorName;
                yDebug() << LogPrefix << "Type                            :" << sourceType;
                yDebug() << LogPrefix << "Output frame                    :" << WrenchSourceData.outputFrame;
                yDebug() << LogPrefix << "Human foot position             :" << sourceGroup.find("humanFootPosition").asList()->toString();
                yDebug() << LogPrefix << "Human linking frame             :" << WrenchSourceData.humanLinkingFrame;
                yDebug() << LogPrefix << "Robot linking frame             :" << WrenchSourceData.robotLinkingFrame;
                yDebug() << LogPrefix << "Robot source origin frame       :" << WrenchSourceData.robotSourceOriginFrame;
                yDebug() << LogPrefix << "Robot source orientation frame  :" << WrenchSourceData.robotSourceOrientationFrame;
                yDebug() << LogPrefix << "================================:";

                break;
            }

            case WrenchSourceType::Dummy: {
                if (!(sourceGroup.check("value") && sourceGroup.find("value").isList())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: value not found or not a valid list";
                    return false;
                }

                iDynTree::Wrench wrench;
                if (!parseWrench(sourceGroup.find("value").asList(), wrench) ) {
                    yError() << LogPrefix << "Failed to parse" << sourceName
                             << ":: position or rotation";
                    return false;
                }
                WrenchSourceData.wrench = wrench;


                yDebug() << LogPrefix << "=============:";
                yDebug() << LogPrefix << "New source   :" << WrenchSourceData.name;
                yDebug() << LogPrefix << "Sensor name  :" << WrenchSourceData.sensorName;
                yDebug() << LogPrefix << "Type         :" << sourceType;
                yDebug() << LogPrefix << "Output frame :" << WrenchSourceData.outputFrame;
                yDebug() << LogPrefix << "=============:";

                break;
            }


        }

        // Store the wrench source data
        pImpl->wrenchSources.emplace_back(WrenchSourceData);
    }
    pImpl->transformedWrenches.resize(pImpl->wrenchSources.size());

    if(!parseMAPEstParams(config, pImpl->mapEstParams))
    {
        return false;
    }

    // Configure map estimator
    if(pImpl->mapEstParams.useMAPEst)
    {
        pImpl->readHumanData = true;

        pImpl->mapEstHelper.berdyOptions.berdyVariant = iDynTree::BerdyVariants::BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES;
        pImpl->mapEstHelper.berdyOptions.includeAllNetExternalWrenchesAsSensors = true;
        pImpl->mapEstHelper.berdyOptions.includeAllJointTorquesAsSensors = false;
        pImpl->mapEstHelper.berdyOptions.includeAllJointAccelerationsAsSensors = false;
        pImpl->mapEstHelper.berdyOptions.includeAllNetExternalWrenchesAsSensors = true;
        pImpl->mapEstHelper.berdyOptions.includeAllNetExternalWrenchesAsDynamicVariables = true;

        pImpl->mapEstHelper.berdyOptions.baseLink = "Pelvis"; //TODO use the ihumanstate one

        if(!pImpl->mapEstHelper.berdyOptions.checkConsistency())
        {
            yError() << LogPrefix << "BERDY options are not consistent";
            return false;
        }

        // Initialize the BerdyHelper
        if (!pImpl->mapEstHelper.berdyHelper.init(pImpl->humanModel, iDynTree::SensorsList(), pImpl->mapEstHelper.berdyOptions)) {
            yError() << LogPrefix << "Failed to initialize BERDY";
            return false;
        }

        pImpl->mapEstHelper.mapSolver = std::make_unique<iDynTree::BerdySparseMAPSolver>(pImpl->mapEstHelper.berdyHelper);
        pImpl->mapEstHelper.mapSolver->initialize();

        // sigma_y
        iDynTree::Triplets measurementsCovarianceMatrixTriplets;
        for (const iDynTree::BerdySensor& berdySensor : pImpl->mapEstHelper.berdyHelper.getSensorsOrdering()) {
            switch(berdySensor.type)
            {
                case iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR:
                {
                    // initialize with default covariance
                    iDynTree::Vector6 wrenchCovariance;
                    for(int i=0; i<6; i++) wrenchCovariance.setVal(i,pImpl->mapEstParams.measurementDefaultCovariance);
                    
                    // set specific covariance if configured
                    auto specificMeasurementsPtr = pImpl->mapEstParams.specificMeasurementsCovariance.find(berdySensor.id);
                    if(specificMeasurementsPtr!=pImpl->mapEstParams.specificMeasurementsCovariance.end())
                    {
                        for(int i=0; i<6; i++) wrenchCovariance.setVal(i, specificMeasurementsPtr->second[i]);
                    }
                    
                    for(std::size_t i=0; i<6; i++) measurementsCovarianceMatrixTriplets.setTriplet({berdySensor.range.offset+i,berdySensor.range.offset+i, wrenchCovariance[i]});
                }    
                    break;
                case iDynTree::BerdySensorTypes::RCM_SENSOR:
                {
                    auto specificMeasurementsPtr = pImpl->mapEstParams.specificMeasurementsCovariance.find("RCM_SENSOR");
                    for(std::size_t i=0; i<6; i++) 
                    {
                        measurementsCovarianceMatrixTriplets.setTriplet({berdySensor.range.offset+i,berdySensor.range.offset+i, specificMeasurementsPtr->second[i]});
                    }
                }
                    break;
                default:
                    break;
            }
        }
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> measurementsPriorCovarianceMatrix;
        std::size_t sigmaYSize = pImpl->mapEstHelper.berdyHelper.getNrOfSensorsMeasurements();
        measurementsPriorCovarianceMatrix.resize(sigmaYSize, sigmaYSize);
        measurementsPriorCovarianceMatrix.setFromTriplets(measurementsCovarianceMatrixTriplets);
        pImpl->mapEstHelper.mapSolver->setMeasurementsPriorCovariance(measurementsPriorCovarianceMatrix);

        // Set mu_d
        iDynTree::VectorDynSize dynamicsRegularizationExpectedValueVector;
        dynamicsRegularizationExpectedValueVector.resize(pImpl->mapEstHelper.berdyHelper.getNrOfDynamicVariables());
        for(std::size_t i=0; i<dynamicsRegularizationExpectedValueVector.size(); i++) dynamicsRegularizationExpectedValueVector.setVal(i, pImpl->mapEstParams.priorDynamicsRegularizationExpected);
        pImpl->mapEstHelper.mapSolver->setDynamicsRegularizationPriorExpectedValue(dynamicsRegularizationExpectedValueVector);

        // set Sigma_d
        iDynTree::Triplets priorDynamicsRegularizationCovarianceMatrixTriplets;
        std::size_t sigmaDSize = pImpl->mapEstHelper.berdyHelper.getNrOfDynamicVariables();
        for(std::size_t i=0; i<sigmaDSize; i++)
        {
            priorDynamicsRegularizationCovarianceMatrixTriplets.setTriplet({i,i,pImpl->mapEstParams.priorDynamicsRegularizationCovarianceValue});
        }
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> priorDynamicsRegularizationCovarianceMatrix;
        priorDynamicsRegularizationCovarianceMatrix.resize(sigmaDSize, sigmaDSize);
        priorDynamicsRegularizationCovarianceMatrix.setFromTriplets(priorDynamicsRegularizationCovarianceMatrixTriplets);
        pImpl->mapEstHelper.mapSolver->setDynamicsRegularizationPriorCovariance(priorDynamicsRegularizationCovarianceMatrix);

        if (!pImpl->mapEstHelper.mapSolver->isValid()) {
            yError() << LogPrefix << "Failed to initialize the Berdy MAP solver";
            return false;
        }

    }

    // ===================
    // INITIALIZE RPC PORT
    // ===================

    std::string rpcPortName;
    if (!(config.check("rpcPortPrefix") && config.find("rpcPortPrefix").isString())) {
        rpcPortName = "/" + DeviceName + "/rpc:i";
    }
    else {
        rpcPortName = "/" + config.find("rpcPortPrefix").asString() + "/" + DeviceName + "/rpc:i";
    }

    if (!pImpl->rpcPort.open(rpcPortName)) {
        yError() << LogPrefix << "Unable to open rpc port " << rpcPortName;
        return false;
    }

    // Set rpc port reader
    pImpl->rpcPort.setReader(*pImpl->commandPro);

    // Initialize buffer data
    pImpl->baseVelocity.zero();

    // Resize human joint quantities buffer
    pImpl->humanJointPositionsVec.resize(pImpl->humanModel.getNrOfDOFs());
    pImpl->humanJointVelocitiesVec.resize(pImpl->humanModel.getNrOfDOFs());

    pImpl->humanJointPositionsVec.zero();
    pImpl->humanJointVelocitiesVec.zero();

    return true;
}

bool HumanWrenchProvider::close()
{
    return true;
}

void HumanWrenchProvider::run()
{
    if (pImpl->readHumanData) {
        if(pImpl->iHumanState==nullptr)
        {
            yError()<<LogPrefix<<"iHumanState is not attached";
            askToStop();
            return;
        }
        auto basePositionInterface = pImpl->iHumanState->getBasePosition();
        auto baseVelocityInterface = pImpl->iHumanState->getBaseVelocity();
        auto baseOrientationInterface = pImpl->iHumanState->getBaseOrientation();
        auto jointPositionsInterface = pImpl->iHumanState->getJointPositions();
        auto jointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();
        auto jointNamesStateInterface = pImpl->iHumanState->getJointNames();

        pImpl->basePose.setPosition(iDynTree::Position(basePositionInterface.at(0),
                                                       basePositionInterface.at(1),
                                                       basePositionInterface.at(2)));
        iDynTree::Vector4 quaternion;
        quaternion.setVal(0, baseOrientationInterface.at(0));
        quaternion.setVal(1, baseOrientationInterface.at(1));
        quaternion.setVal(2, baseOrientationInterface.at(2));
        quaternion.setVal(3, baseOrientationInterface.at(3));
        pImpl->basePose.setRotation(iDynTree::Rotation::RotationFromQuaternion(quaternion));

        iDynTree::Vector3 linearVelocity;
        linearVelocity.setVal(0, baseVelocityInterface.at(0));
        linearVelocity.setVal(1, baseVelocityInterface.at(1));
        linearVelocity.setVal(2, baseVelocityInterface.at(2));
        iDynTree::Vector3 angularVelocity;
        angularVelocity.setVal(0, baseVelocityInterface.at(3));
        angularVelocity.setVal(1, baseVelocityInterface.at(4));
        angularVelocity.setVal(2, baseVelocityInterface.at(5));

        pImpl->baseVelocity.setLinearVec3(linearVelocity);
        pImpl->baseVelocity.setAngularVec3(angularVelocity);

        // Human joint quantities are in radians
        for (int j = 0; j < pImpl->humanModel.getNrOfDOFs(); j++) {
            for (int i = 0; i < jointNamesStateInterface.size(); i++) {
                if (pImpl->humanModel.getJointName(j) == jointNamesStateInterface.at(i)) {
                    pImpl->humanJointPositionsVec.setVal(j, jointPositionsInterface.at(i));
                    pImpl->humanJointVelocitiesVec.setVal(j, jointVelocitiesInterface.at(i));
                    break;
                }
            }
        }

        // Update kin-dyn object for human
        pImpl->humanKinDynComp.setRobotState(pImpl->basePose,
                                                 pImpl->humanJointPositionsVec,
                                                 pImpl->baseVelocity,
                                                 pImpl->humanJointVelocitiesVec,
                                                 pImpl->world_gravity);
    }
    if (pImpl->pHRIScenario) {
        // Get robot joint quantities from joint wearable sensors
        std::vector<double> robotJointsPosition;
        std::vector<double> robotJointsVeclocity;

        for (auto& jointSensor : pImpl->robotJointWearableSensors) {

            std::string sensorName = jointSensor->getSensorName();
            size_t found = sensorName.find_last_of(":");

            std::string jointName = sensorName.substr(found+1);

            std::vector<std::string>::iterator jointVecIterator
                                        = std::find(pImpl->robotJointNamesListFromConfig.begin(),
                                                    pImpl->robotJointNamesListFromConfig.end(),
                                                    jointName);

            if (jointVecIterator != pImpl->robotJointNamesListFromConfig.end()) {

                // Get joint position
                double jointPosition;
                jointSensor->getJointPosition(jointPosition);
                robotJointsPosition.push_back(jointPosition);

                // Get joint velocity
                double jointVelocity;
                jointSensor->getJointVelocity(jointVelocity);
                robotJointsVeclocity.push_back(jointVelocity);

            }
        }

        // Resize robot joint quantities buffer
        pImpl->robotJointPositionsVec.resize(pImpl->robotModel.getNrOfDOFs());
        pImpl->robotJointVelocitiesVec.resize(pImpl->robotModel.getNrOfDOFs());

        // Robot joint quantities are in degree and are converted to radians
        for (int j = 0; j < pImpl->robotModel.getNrOfDOFs(); j++) {
            for (int i = 0; i < pImpl->robotJointsName.size(); i++) {
                if (pImpl->robotModel.getJointName(j) == pImpl->robotJointsName.at(i)) {
                    pImpl->robotJointPositionsVec.setVal(j, robotJointsPosition.at(i)*(M_PI/180));
                    pImpl->robotJointVelocitiesVec.setVal(j, robotJointsVeclocity.at(i)*(M_PI/180));
                }
            }
        }

    }

    for (unsigned i = 0; i < pImpl->wrenchSources.size(); ++i) {
        auto& forceSource = pImpl->wrenchSources[i];

        // Get the measurement
        // ===================

        wearable::Vector3 forces;
        wearable::Vector3 torques;

        if (forceSource.type == WrenchSourceType::Fixed || forceSource.type == WrenchSourceType::Robot)
        {
            if (!forceSource.ftWearableSensor) {
                yError() << LogPrefix << "Failed to get wearable sensor for source" << forceSource.name;
                askToStop();
                return;
            }

            if (!forceSource.ftWearableSensor->getForceTorque6D(forces, torques)) {
                yError() << LogPrefix << "Failed to get measurement from sensor"
                        << forceSource.ftWearableSensor->getSensorName();
                askToStop();
                return;
            }

            forceSource.wrench = iDynTree::Wrench({forces[0], forces[1], forces[2]},
                                                  {torques[0], torques[1], torques[2]});
        }

        

        // Tranform it to the correct frame
        // ================================

        iDynTree::Wrench transformedWrench;

        if (forceSource.type == WrenchSourceType::Dummy) {
            std::lock_guard<std::mutex> lock(pImpl->mutex);

            // store the frame transform
            auto worldToFrameRotation = pImpl->humanKinDynComp.getWorldTransform(forceSource.outputFrame).getRotation();
            forceSource.frameTransformer.transform.setRotation(worldToFrameRotation.inverse());
        }

        if (forceSource.type == WrenchSourceType::Robot) {
            iDynTree::Transform humanFeetToHandsTransform = pImpl->humanKinDynComp.getRelativeTransform(forceSource.outputFrame,forceSource.humanLinkingFrame);

            iDynTree::Transform robotFeetToHandsTransform = iDynTree::Transform::Identity();
            iDynTree::KinDynComputations robotKinDynComp;

            robotKinDynComp.loadRobotModel(pImpl->robotModel);
            robotKinDynComp.setRobotState(iDynTree::Transform::Identity(),
                                          pImpl->robotJointPositionsVec,
                                          pImpl->baseVelocity,
                                          pImpl->robotJointVelocitiesVec,
                                          pImpl->world_gravity);
            robotFeetToHandsTransform = robotKinDynComp.getRelativeTransformExplicit(pImpl->robotModel.getFrameIndex(forceSource.robotLinkingFrame),
                                                                                     pImpl->robotModel.getFrameIndex(forceSource.robotLinkingFrame),
                                                                                     pImpl->robotModel.getFrameIndex(forceSource.robotSourceOriginFrame),
                                                                                     pImpl->robotModel.getFrameIndex(forceSource.robotSourceOrientationFrame));

            // TODO: Move this logic to WrenchFrameTransformers.cpp file

            // Access the tranforms through pointers
            std::lock_guard<std::mutex> lock(pImpl->mutex);

            // Compute the final robot to human transform
            iDynTree::Transform robotToHumanTransform;

            robotToHumanTransform = iDynTree::Transform::Identity() *
                                    humanFeetToHandsTransform * //HumanHand_H_HumanFoot
                                    pImpl->robotHumanFeetFixedTransform * //HumanFoot_H_RobotFoot
                                    robotFeetToHandsTransform; //RobotFoot_H_RobotHand

            // Update the stored transform
            forceSource.frameTransformer.transform = robotToHumanTransform;

            // Get reaction wrenches for the robot
            forceSource.wrench = forceSource.wrench * -1;
        }

        if (!forceSource.frameTransformer.transformWrenchFrame(forceSource.wrench, transformedWrench)) {
            askToStop();
            return;
        }

        pImpl->transformedWrenches[i] = transformedWrench;
    }

    if(pImpl->mapEstParams.useMAPEst)
    {
        // Set NET_EXT_WRENCH measurements
        iDynTree::VectorDynSize berdyMeasurementVector(pImpl->mapEstHelper.berdyHelper.getNrOfSensorsMeasurements());
        berdyMeasurementVector.zero();
        for (unsigned i = 0; i < pImpl->wrenchSources.size(); ++i) {
            auto& forceSource = pImpl->wrenchSources[i];
            std::string sensorName = forceSource.outputFrame;
            iDynTree::LinkIndex linkIndex = pImpl->mapEstHelper.berdyHelper.model().getLinkIndex(sensorName);
            if(linkIndex==iDynTree::LINK_INVALID_INDEX)
            {
                yError()<<LogPrefix<<"Cannot find link:"<<sensorName;
                askToStop();
                return;
            }
            iDynTree::IndexRange sensorRange = pImpl->mapEstHelper.berdyHelper.getRangeLinkSensorVariable(iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR, linkIndex);
            for(std::size_t j=0; j<sensorRange.size; j++)
            {
                berdyMeasurementVector.setVal(sensorRange.offset+j, pImpl->transformedWrenches[i].getVal(j));
            }
        }

        // Set RCM_SENSOR measurement
        iDynTree::Model &model = pImpl->mapEstHelper.berdyHelper.model();
        iDynTree::FrameIndex baseFrameIndex = model.getLinkIndex(pImpl->mapEstHelper.berdyOptions.baseLink);

        iDynTree::KinDynComputations kinDynComputations;
        kinDynComputations.loadRobotModel(pImpl->humanModel);
        kinDynComputations.setFrameVelocityRepresentation(iDynTree::FrameVelocityRepresentation::BODY_FIXED_REPRESENTATION);
        kinDynComputations.setRobotState(pImpl->basePose,
                                        pImpl->humanJointPositionsVec,
                                        pImpl->baseVelocity,
                                        pImpl->humanJointVelocitiesVec,
                                        pImpl->world_gravity);

        iDynTree::SpatialForceVector rcm = computeRCMInBaseUsingMeasurements(kinDynComputations, baseFrameIndex);

        iDynTree::IndexRange rcmSensorRange = pImpl->mapEstHelper.berdyHelper.getRangeRCMSensorVariable(iDynTree::BerdySensorTypes::RCM_SENSOR);
        for(std::size_t j=0; j<rcmSensorRange.size; j++)
        {
            berdyMeasurementVector.setVal(rcmSensorRange.offset+j, rcm.getVal(j));
        }

        iDynTree::Vector3 baseAngularVelocity = pImpl->baseVelocity.getAngularVec3();
        iDynTree::JointPosDoubleArray jointsPosition(model);
        iDynTree::JointDOFsDoubleArray jointsVelocity(model);
        iDynTree::JointDOFsDoubleArray jointsAcceleration(model);


        for(int i=0; i<pImpl->humanJointPositionsVec.size(); i++) jointsPosition.setVal(i, pImpl->humanJointPositionsVec.getVal(i));
        for(int i=0; i<pImpl->humanJointVelocitiesVec.size(); i++) jointsVelocity.setVal(i, pImpl->humanJointVelocitiesVec.getVal(i));


        // Set the kinematic information necessary for the dynamics estimation
        //TODO check if needed
        pImpl->mapEstHelper.berdyHelper.updateKinematicsFromFloatingBase(jointsPosition,
                                                             jointsVelocity,
                                                             baseFrameIndex,
                                                             baseAngularVelocity);


        pImpl->mapEstHelper.mapSolver->updateEstimateInformationFloatingBase(jointsPosition,
                                                                             jointsVelocity,
                                                                             baseFrameIndex,
                                                                             baseAngularVelocity,
                                                                             berdyMeasurementVector);


        if(!pImpl->mapEstHelper.mapSolver->doEstimate())
        {
            yError()<<LogPrefix<<"Estimate computation failed!";
            askToStop();
            return;
        }

        const iDynTree::VectorDynSize& estimatedWrenches = pImpl->mapEstHelper.mapSolver->getLastEstimate();

        iDynTree::LinkNetExternalWrenches linkExtWrenches(pImpl->humanModel);
        pImpl->mapEstHelper.berdyHelper.extractLinkNetExternalWrenchesFromDynamicVariables(estimatedWrenches,linkExtWrenches);

        // update transformed wrenches with estimate
        for (unsigned i = 0; i < pImpl->wrenchSources.size(); ++i)
        {
            iDynTree::LinkIndex linkIndex = pImpl->mapEstHelper.berdyHelper.model().getLinkIndex(pImpl->wrenchSources[i].outputFrame);
            for(int j=0; j<6; j++)
            {
                pImpl->transformedWrenches[i].setVal(j, linkExtWrenches(linkIndex).getVal(j));
            }
        
        }
    }

    // Expose the data as IAnalogSensor
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        for (unsigned i = 0; i < pImpl->wrenchSources.size(); ++i) {
            pImpl->analogSensorData.measurements[6 * i + 0] = pImpl->transformedWrenches[i].getLinearVec3()(0);
            pImpl->analogSensorData.measurements[6 * i + 1] = pImpl->transformedWrenches[i].getLinearVec3()(1);
            pImpl->analogSensorData.measurements[6 * i + 2] = pImpl->transformedWrenches[i].getLinearVec3()(2);
            pImpl->analogSensorData.measurements[6 * i + 3] = pImpl->transformedWrenches[i].getAngularVec3()(0);
            pImpl->analogSensorData.measurements[6 * i + 4] = pImpl->transformedWrenches[i].getAngularVec3()(1);
            pImpl->analogSensorData.measurements[6 * i + 5] = pImpl->transformedWrenches[i].getAngularVec3()(2);
        }
    }

    // Check for rpc command status and apply command
    if (pImpl->commandPro->cmdStatus != rpcCommand::empty) {
        // Apply rpc command
        if (!pImpl->applyRpcCommand()) {
            yWarning() << LogPrefix << "Failed to execute the rpc command";
        }

        // reset the rpc internal status
        {
            std::lock_guard<std::mutex> lock(pImpl->mutex);
            pImpl->commandPro->resetInternalVariables();
        }
    }
}

bool HumanWrenchProvider::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    // Get the device name from the driver
    const std::string deviceName = poly->getValue("device").asString();

    yInfo()<<LogPrefix<<"Attaching "<< deviceName;

    if (deviceName == "human_state_provider" || deviceName == "human_state_remapper" ) {

        // Attach IHumanState interface from HumanStateProvider
        if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
            yError() << LogPrefix << "Failed to view IHumanState interface from the polydriver";
            return false;
        }

        //TODO Check the interface
        int count = 1000000;
        while (pImpl->iHumanState->getNumberOfJoints() == 0
                || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
            yError() << LogPrefix<<"The IHumanState interface might not be ready";
            if(count--==0) return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
    }

    if (deviceName == "iwear_remapper") {

        if (pImpl->iWear || !poly->view(pImpl->iWear) || !pImpl->iWear) {
            yError() << LogPrefix << "Failed to view the IWear interface from the PolyDriver";
            return false;
        }

        while (pImpl->iWear->getStatus() == wearable::WearStatus::WaitingForFirstRead) {
            yInfo() << LogPrefix << "IWear interface waiting for first data. Waiting...";
            yarp::os::Time::delay(5);
        }

        if (pImpl->iWear->getStatus() != wearable::WearStatus::Ok) {
            yError() << LogPrefix << "The status of the attached IWear interface is not ok ("
                     << static_cast<int>(pImpl->iWear->getStatus()) << ")";
            return false;
        }

        // Get the ft wearable sensors containing the input measurements
        for (auto& ftSensorSourceData : pImpl->wrenchSources) {
            if (ftSensorSourceData.type == WrenchSourceType::Fixed)
            {
                auto sensor = pImpl->iWear->getForceTorque6DSensor(ftSensorSourceData.sensorName);

                if (!sensor) {
                    yError() << LogPrefix << "Failed to get sensor" << ftSensorSourceData.sensorName
                            << "from the attached IWear interface";
                    return false;
                }

                ftSensorSourceData.ftWearableSensor = sensor;
            }
        }

        // Initialize the number of channels of the equivalent IAnalogSensor
        const size_t numberOfFTSensors = pImpl->wrenchSources.size();
        {
            std::lock_guard<std::mutex> lock(pImpl->mutex);
            pImpl->analogSensorData.measurements.resize(6 * numberOfFTSensors, 0);
            pImpl->analogSensorData.numberOfChannels = 6 * numberOfFTSensors;
        }


        if (pImpl->pHRIScenario) {

            // Check the size is at least as much as the one required by the config joints list
            if (pImpl->iWear->getVirtualJointKinSensors().size() < pImpl->robotJointNamesListFromConfig.size()) {
                yError() << LogPrefix << "The number of joints from the IWear interface are less than the number of joints needed as defined in the configuration file";
                return false;
            }

            // Get the joing position sensors containing the joint data
            pImpl->robotJointWearableSensors = pImpl->iWear->getVirtualJointKinSensors();

            for (auto& robotJointWearableSensor : pImpl->robotJointWearableSensors) {
                if (!robotJointWearableSensor) {
                    yError() << LogPrefix << "Failed to get robot joint wearabke sensor pointer from the attached IWear interface";
                    return false;
                }

                // Get joint name from the sensor name
                std::string sensorName = robotJointWearableSensor->getSensorName();
                size_t found = sensorName.find_last_of(":");

                std::string jointName = sensorName.substr(found+1);

                std::vector<std::string>::iterator jointVecIterator
                                            = std::find(pImpl->robotJointNamesListFromConfig.begin(),
                                                        pImpl->robotJointNamesListFromConfig.end(),
                                                        jointName);

                if (jointVecIterator != pImpl->robotJointNamesListFromConfig.end()) {
                     pImpl->robotJointsName.push_back(jointName);
                }
                 else {
                    yWarning() << LogPrefix << "Ignoring sensor " << sensorName
                                            << " as it is not asked in the configuration file";
                }
            }
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
    }

    // ====
    // MISC
    // ====

    
    return true;
}

void HumanWrenchProvider::threadRelease()
{}

bool HumanWrenchProvider::detach()
{
    while(isRunning()) {
        stop();
    }

    pImpl->iHumanState = nullptr;
    pImpl->iWear = nullptr;
    return true;
}

bool HumanWrenchProvider::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    bool attachStatus = false;
    if (driverList.size() > 2) {
        yError() << LogPrefix << "This wrapper accepts only two attached PolyDriver";
        return false;
    }

    for (size_t i = 0; i < driverList.size(); i++) {
        const yarp::dev::PolyDriverDescriptor* driver = driverList[i];

        if (!driver) {
            yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
            return false;
        }

        attachStatus = attach(driver->poly);
    }

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    yInfo() << LogPrefix << "attach() successful";

    return attachStatus;
}

bool HumanWrenchProvider::detachAll()
{
    return detach();
}

// =============
// IAnalogSensor
// =============

int HumanWrenchProvider::read(yarp::sig::Vector& out)
{
    out.resize(pImpl->analogSensorData.measurements.size());

    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        std::copy(pImpl->analogSensorData.measurements.begin(),
                  pImpl->analogSensorData.measurements.end(),
                  out.data());
    }

    return IAnalogSensor::AS_OK; // TODO
}

int HumanWrenchProvider::getState(int ch)
{
    // Check if channel is in the right range
    if (ch < 0 || ch > pImpl->wrenchSources.size() - 1) {
        yError() << LogPrefix << "Failed to get status for channel" << ch;
        yError() << LogPrefix << "Channels must be in the range 0 -"
                 << pImpl->wrenchSources.size() - 1;
        return IAnalogSensor::AS_ERROR;
    }

    // Get the sensor associated with the channel
    const auto& sensorData = pImpl->wrenchSources[ch];

    if (!sensorData.ftWearableSensor) {
        yError() << "The wearable sensor for this channel was not allocated";
        return IAnalogSensor::AS_ERROR;
    }

    // Map the wearable sensor status to IAnalogSensor status
    switch (sensorData.ftWearableSensor->getSensorStatus()) {
        case wearable::WearStatus::Error:
            return IAnalogSensor::AS_ERROR;
        case wearable::WearStatus::Ok:
            return IAnalogSensor::AS_OK;
        case wearable::WearStatus::Calibrating:
            return IAnalogSensor::AS_TIMEOUT;
            ;
        case wearable::WearStatus::Overflow:
            return IAnalogSensor::AS_OVF;
        case wearable::WearStatus::Timeout:
            return IAnalogSensor::AS_TIMEOUT;

        case wearable::WearStatus::Unknown:
            return IAnalogSensor::AS_ERROR;

        case wearable::WearStatus::WaitingForFirstRead:
            return IAnalogSensor::AS_TIMEOUT;
    };

    // If there is no mapping between Wearable sensor 
    // status to IAnalogSensor, return error
    return IAnalogSensor::AS_ERROR;
}

int HumanWrenchProvider::getChannels()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->analogSensorData.numberOfChannels;
}

int HumanWrenchProvider::calibrateSensor()
{
    return IAnalogSensor::AS_ERROR;
}

int HumanWrenchProvider::calibrateSensor(const yarp::sig::Vector& /*value*/)
{
    return IAnalogSensor::AS_ERROR;
}

int HumanWrenchProvider::calibrateChannel(int /*ch*/)
{
    return IAnalogSensor::AS_ERROR;
}

int HumanWrenchProvider::calibrateChannel(int /*ch*/, double /*value*/)
{
    return IAnalogSensor::AS_ERROR;
}

// ============
// IHumanWrench
// ============

std::vector<std::string> HumanWrenchProvider::getWrenchSourceNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    std::vector<std::string> sourcesNames;
    for (size_t idx = 0; idx < pImpl->wrenchSources.size(); idx++) {
        sourcesNames.emplace_back(pImpl->wrenchSources.at(idx).name);
    }

    return sourcesNames;
}

size_t HumanWrenchProvider::getNumberOfWrenchSources() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->wrenchSources.size();
}

std::vector<double> HumanWrenchProvider::getWrenches() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    std::vector<double> wrenchValues;
    size_t vecSize = pImpl->analogSensorData.measurements.size();
    wrenchValues.resize(vecSize);
    for (size_t idx = 0; idx < vecSize; idx++) {
        wrenchValues.at(idx) = pImpl->analogSensorData.measurements.at(idx);
    }

    return wrenchValues;
}

bool HumanWrenchProvider::Impl::applyRpcCommand()
{
    switch(commandPro->cmdStatus) {
        case rpcCommand::setWorldWrench : {
            for (unsigned i = 0; i < wrenchSources.size(); ++i) {
                auto& forceSource = wrenchSources[i];
                if (forceSource.type == WrenchSourceType::Dummy)
                {
                    forceSource.wrench = iDynTree::Wrench(iDynTree::Force(commandPro->wrench_fx, commandPro->wrench_fy, commandPro->wrench_fz),
                                                            iDynTree::Torque(commandPro->wrench_tx, commandPro->wrench_ty, commandPro->wrench_tz) );
                }

            }
            break;
        }
        default : {
            yWarning() << LogPrefix << "Command not valid";
            return false;
        }
    }
    return true;
}

// ===========
// CONSTRUCTOR
// ===========

HumanWrenchProvider::Impl::Impl()
    : commandPro(new CmdParser())
{}
