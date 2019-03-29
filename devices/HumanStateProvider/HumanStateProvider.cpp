/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStateProvider.h"
#include "IKWorkerPool.h"
#include "InverseVelocityKinematics.hpp"

#include <Wearable/IWear/IWear.h>
#include <iDynTree/InverseKinematics.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <array>
#include <mutex>
#include <string>
#include <atomic>
#include <chrono>
#include <thread>
#include <stack>
#include <unordered_map>

#include <Utils.hpp>

/*!
 * @brief analyze model and list of segments to create all possible segment pairs
 *
 * @param[in] model the full model
 * @param[in] humanSegments list of segments on which look for the possible pairs
 * @param[out] framePairs resulting list of all possible pairs. First element is parent, second is child
 * @param[out] framePairIndeces indeces in the humanSegments list of the pairs in framePairs
 */
static void createEndEffectorsPairs(const iDynTree::Model& model,
                                    std::vector<SegmentInfo>& humanSegments,
                                    std::vector<std::pair<std::string, std::string>> &framePairs,
                                    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex> > &framePairIndeces);

static bool getReducedModel(const iDynTree::Model& modelInput,
                            const std::string& parentFrame,
                            const std::string& endEffectorFrame,
                            iDynTree::Model& modelOutput);

const std::string DeviceName = "HumanStateProvider";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;
using namespace wearable;

// ==============
// IMPL AND UTILS
// ==============

using ModelJointName = std::string;
using ModelLinkName = std::string;

using WearableLinkName = std::string;
using WearableJointName = std::string;

struct FloatingBaseName
{
    std::string model;
    std::string wearable;
};

struct WearableJointInfo
{
    WearableJointName name;
    size_t index;
};

// Struct that contains all the data exposed by the HumanState interface
struct SolutionIK
{
    std::vector<double> jointPositions;
    std::vector<double> jointVelocities;

    std::array<double, 3> basePosition;
    std::array<double, 4> baseOrientation;

    std::array<double, 6> baseVelocity;

    void clear()
    {
        jointPositions.clear();
        jointVelocities.clear();
    }
};

// Container of data coming from the wearable interface
struct WearableStorage
{
    // Sensor associated with the base
    SensorPtr<const sensor::IVirtualLinkKinSensor> baseLinkSensor;

    // Maps [model joint / link name] ==> [wearable virtual sensor name]
    //
    // E.g. [Pelvis] ==> [XsensSuit::vLink::Pelvis]. Read from the configuration.
    //
    std::unordered_map<ModelLinkName, WearableLinkName> modelToWearable_LinkName;
    std::unordered_map<ModelJointName, WearableJointInfo> modelToWearable_JointInfo;

    // Maps [wearable virtual sensor name] ==> [virtual sensor]
    std::unordered_map<WearableLinkName, SensorPtr<const sensor::IVirtualLinkKinSensor>>
    linkSensorsMap;
    std::unordered_map<WearableJointName, SensorPtr<const sensor::IVirtualSphericalJointKinSensor>>
    jointSensorsMap;
};

class HumanStateProvider::impl
{
public:
    // Attached interface
    wearable::IWear* iWear = nullptr;

    bool allowIKFailures;
    bool useXsensJointsAngles;

    bool firstRun = true;
    mutable std::mutex mutex;

    // Wearable variables
    WearableStorage wearableStorage;

    // Model variables
    iDynTree::Model humanModel;
    FloatingBaseName floatingBaseFrame;

    std::vector<SegmentInfo> segments;
    std::vector<LinkPairInfo> linkPairs;

    // Buffers
    iDynTree::VectorDynSize jointAngles;
    std::unordered_map<std::string, iDynTree::Rotation> linkRotationMatrices;
    std::unordered_map<std::string, iDynTree::Transform> linkTransformMatrices;
    std::unordered_map<std::string, iDynTree::Rotation> linkOrientationMatrices;
    std::unordered_map<std::string, iDynTree::Twist> linkVelocities;
    iDynTree::VectorDynSize jointConfigurationSolution;
    iDynTree::VectorDynSize jointVelocitiesSolution;
    iDynTree::VectorDynSize jointVelocitiesSolutionOld;
    iDynTree::Transform baseTransformSolution;
    iDynTree::Twist baseVelocitySolution;
    iDynTree::Twist baseVelocitySolutionOld;

    std::unordered_map<std::string, iDynTreeHelper::Rotation::rotationDistance> linkErrorOrientations;
    std::unordered_map<std::string, iDynTree::Vector3> linkErrorAngularVelocities;

    // IK stuff
    int ikPoolSize{1};
    int maxIterationsIK;
    double costTolerance;
    std::string solverName;
    yarp::os::Value ikPoolOption;
    std::unique_ptr<IKWorkerPool> ikPool;
    SolutionIK solution;

    double posTargetWeight;
    double rotTargetWeight;
    double costRegularization;

    double integrationBasedIKLinearCorrectionGain;
    double integrationBasedIKAngularCorrectionGain;

    bool useGlobalIK;
    bool usePairWisedIK;
    bool useIntegrationBasedIK;
    bool useBaseMeasurementDirectlyFromXsens;
    iDynTree::InverseKinematics globalIK;

    double lastTime{-1.0};
    double startTime{0};

    std::unique_ptr<iDynTree::KinDynComputations> kinDynComputations;

    bool getJointAnglesFromInputData(iDynTree::VectorDynSize& jointAngles);
    bool getLinkOrientationFromInputData(std::unordered_map<std::string, iDynTree::Rotation>& r);
    bool getLinkTransformFromInputData(std::unordered_map<std::string, iDynTree::Transform>& t);
    bool getLinkVelocityFromInputData(std::unordered_map<std::string, iDynTree::Twist>& t);

    bool getRealLinkJacobiansRelativeToBase(iDynTree::VectorDynSize jointConfigurations, std::unordered_map<std::string, iDynTree::MatrixDynSize>& J);
    bool computeJointVelocities(iDynTree::VectorDynSize jointConfigurations, std::unordered_map<std::string, iDynTree::Twist> linkVelocitiesMap, iDynTree::VectorDynSize& jointVelocities);

    bool getRealLinkJacobians(iDynTree::VectorDynSize jointConfigurations, iDynTree::Transform floatingBasePose, std::unordered_map<std::string, iDynTree::MatrixDynSize>& J);
    bool computeVelocities(iDynTree::VectorDynSize jointConfigurations, iDynTree::Transform floatingBasePose, std::unordered_map<std::string, iDynTree::Twist> linkVelocitiesMap, iDynTree::VectorDynSize& jointVelocities, iDynTree::Twist& baseVelocity);

    bool computeLinksOrientationErrors(std::unordered_map<std::string, iDynTree::Rotation> linkDesiredOrientations, iDynTree::VectorDynSize jointConfigurations, iDynTree::Transform floatingBasePose, std::unordered_map<std::string, iDynTreeHelper::Rotation::rotationDistance>& linkErrorOrientations);
    bool computeLinksAngularVelocityErrors(std::unordered_map<std::string, iDynTree::Twist> linkDesiredVelocities, iDynTree::VectorDynSize jointConfigurations, iDynTree::Transform floatingBasePose, iDynTree::VectorDynSize jointVelocities, iDynTree::Twist baseVelocity, std::unordered_map<std::string, iDynTree::Vector3>& linkAngularVelocityError);
};

// =========================
// HUMANSTATEPROVIDER DEVICE
// =========================

HumanStateProvider::HumanStateProvider()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanStateProvider::~HumanStateProvider()
{}

bool HumanStateProvider::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("urdf") && config.find("urdf").isString())) {
        yError() << LogPrefix << "urdf option not found or not valid";
        return false;
    }

    if (!(config.check("useGlobalIK") && config.find("useGlobalIK").isBool())) {
        yError() << LogPrefix << "useGlobalIK option not found or not valid";
        return false;
    }

    if (!(config.check("usePairWisedIK") && config.find("usePairWisedIK").isBool())) {
        yError() << LogPrefix << "usePairWisedIK option not found or not valid";
        return false;
    }

    if (!(config.check("useXsensJointsAngles") && config.find("useXsensJointsAngles").isBool())) {
        yError() << LogPrefix << "useXsensJointsAngles option not found or not valid";
        return false;
    }

    if (!(config.check("floatingBaseFrame") && config.find("floatingBaseFrame").isList()
          && config.find("floatingBaseFrame").asList()->size() == 2)) {
        yError() << LogPrefix << "floatingBaseFrame option not found or not valid";
        return false;
    }


    yarp::os::Bottle& linksGroup = config.findGroup("MODEL_TO_DATA_LINK_NAMES");
    if (linksGroup.isNull()) {
        yError() << LogPrefix << "Failed to find group MODEL_TO_DATA_LINK_NAMES";
        return false;
    }

    for (size_t i = 1; i < linksGroup.size(); ++i) {
        if (!(linksGroup.get(i).isList() && linksGroup.get(i).asList()->size() == 2)) {
            yError() << LogPrefix
                     << "Childs of MODEL_TO_DATA_LINK_NAMES must be lists of two elements";
            return false;
        }
        yarp::os::Bottle* list = linksGroup.get(i).asList();
        std::string key = list->get(0).asString();
        yarp::os::Bottle* listContent = list->get(1).asList();

        if (!((listContent->size() == 2) && (listContent->get(0).isString())
              && (listContent->get(1).isString()))) {
            yError() << LogPrefix << "Link list must have two strings";
            return false;
        }
    }

    // =======================================
    // PARSE THE GENERAL CONFIGURATION OPTIONS
    // =======================================

    yarp::os::Bottle* floatingBaseFrameList = config.find("floatingBaseFrame").asList();
    pImpl->useGlobalIK = config.find("useGlobalIK").asBool();
    pImpl->usePairWisedIK = config.find("usePairWisedIK").asBool();
    pImpl->useIntegrationBasedIK = config.find("useIntegrationBasedIK").asBool();
    pImpl->useXsensJointsAngles = config.find("useXsensJointsAngles").asBool();
    const std::string urdfFileName = config.find("urdf").asString();
    pImpl->floatingBaseFrame.model = floatingBaseFrameList->get(0).asString();
    pImpl->floatingBaseFrame.wearable = floatingBaseFrameList->get(1).asString();
    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();

    setPeriod(period);

    for (size_t i = 1; i < linksGroup.size(); ++i) {
        yarp::os::Bottle* listContent = linksGroup.get(i).asList()->get(1).asList();

        std::string modelLinkName = listContent->get(0).asString();
        std::string wearableLinkName = listContent->get(1).asString();

        yInfo() << LogPrefix << "Read link map:" << modelLinkName << "==>" << wearableLinkName;
        pImpl->wearableStorage.modelToWearable_LinkName[modelLinkName] = wearableLinkName;
    }

    // ==========================================
    // PARSE THE DEPENDENDT CONFIGURATION OPTIONS
    // ==========================================

    if (pImpl->useXsensJointsAngles)
    {
        yarp::os::Bottle& jointsGroup = config.findGroup("MODEL_TO_DATA_JOINT_NAMES");
        if (jointsGroup.isNull()) {
            yError() << LogPrefix << "Failed to find group MODEL_TO_DATA_JOINT_NAMES";
            return false;
        }

        for (size_t i = 1; i < jointsGroup.size(); ++i) {
            if (!(jointsGroup.get(i).isList() && jointsGroup.get(i).asList()->size() == 2)) {
                yError() << LogPrefix << "Childs of MODEL_TO_DATA_JOINT_NAMES must be lists";
                return false;
            }
            yarp::os::Bottle* list = jointsGroup.get(i).asList();
            std::string key = list->get(0).asString();
            yarp::os::Bottle* listContent = list->get(1).asList();

            if (!((listContent->size() == 3) && (listContent->get(0).isString())
                  && (listContent->get(1).isString()) && (listContent->get(2).isInt()))) {
                yError() << LogPrefix << "Joint list must have two strings and one integer";
                return false;
            }
        }

        for (size_t i = 1; i < jointsGroup.size(); ++i) {
            yarp::os::Bottle* listContent = jointsGroup.get(i).asList()->get(1).asList();

            std::string modelJointName = listContent->get(0).asString();
            std::string wearableJointName = listContent->get(1).asString();
            size_t wearableJointComponent = listContent->get(2).asInt();

            yInfo() << LogPrefix << "Read joint map:" << modelJointName << "==> (" << wearableJointName
                    << "," << wearableJointComponent << ")";
            pImpl->wearableStorage.modelToWearable_JointInfo[modelJointName] = {wearableJointName,
                                                                                wearableJointComponent};
        }
    }

    if (pImpl->usePairWisedIK || pImpl->useGlobalIK)
    {
        if (!(config.check("allowIKFailures") && config.find("allowIKFailures").isBool())) {
            yError() << LogPrefix << "allowFailures option not found or not valid";
            return false;
        }
        if (!(config.check("maxIterationsIK") && config.find("maxIterationsIK").isInt())) {
            yError() << LogPrefix << "maxIterationsIK option not found or not valid";
            return false;
        }

        if (!(config.check("costTolerance") && config.find("costTolerance").isFloat64())) {
            yError() << LogPrefix << "costTolerance option not found or not valid";
            return false;
        }
        if (!(config.check("ikLinearSolver") && config.find("ikLinearSolver").isString())) {
            yError() << LogPrefix << "ikLinearSolver option not found or not valid";
            return false;
        }
        if (!(config.check("posTargetWeight") && config.find("posTargetWeight").isFloat64())) {
            yError() << LogPrefix << "posTargetWeight option not found or not valid";
            return false;
        }

        if (!(config.check("rotTargetWeight") && config.find("rotTargetWeight").isFloat64())) {
            yError() << LogPrefix << "rotTargetWeight option not found or not valid";
            return false;
        }
        if (!(config.check("costRegularization") && config.find("costRegularization").isDouble())) {
            yError() << LogPrefix << "costRegularization option not found or not valid";
            return false;
        }

        pImpl->allowIKFailures = config.find("allowIKFailures").asBool();
        pImpl->maxIterationsIK = config.find("maxIterationsIK").asInt();
        pImpl->costTolerance = config.find("costTolerance").asFloat64();
        pImpl->solverName = config.find("ikLinearSolver").asString();
        pImpl->posTargetWeight = config.find("posTargetWeight").asFloat64();
        pImpl->rotTargetWeight = config.find("rotTargetWeight").asFloat64();
        pImpl->costRegularization = config.find("costRegularization").asDouble();
    }

    if (pImpl->useGlobalIK || pImpl->useIntegrationBasedIK)
    {
        if (!(config.check("useBaseMeasurementDirectlyFromXsens") && config.find("useBaseMeasurementDirectlyFromXsens").isBool())) {
            yError() << LogPrefix << "useBaseMeasurementDirectlyFromXsens option not found or not valid";
            return false;
        }

        pImpl->useBaseMeasurementDirectlyFromXsens = config.find("useBaseMeasurementDirectlyFromXsens").asBool();
    }

    if (pImpl->usePairWisedIK)
    {
        if (!(config.check("ikPoolSizeOption") && (config.find("ikPoolSizeOption").isString() || config.find("ikPoolSizeOption").isInt()))) {
            yError() << LogPrefix << "ikPoolOption option not found or not valid";
            return false;
        }

        // Get ikPoolSizeOption
        if (config.find("ikPoolSizeOption").isString() && config.find("ikPoolSizeOption").asString() == "auto" ) {
            yInfo() << LogPrefix << "Using " << std::thread::hardware_concurrency() << " available logical threads for ik pool";
            pImpl->ikPoolSize = static_cast<int>(std::thread::hardware_concurrency());
        }
        else if(config.find("ikPoolSizeOption").isInt()) {
            pImpl->ikPoolSize = config.find("ikPoolSizeOption").asInt();
        }
    }

    if (pImpl->useIntegrationBasedIK)
    {
        if (!(config.check("integrationBasedIKCorrectionGains") && config.find("integrationBasedIKCorrectionGains").isList()
              && config.find("integrationBasedIKCorrectionGains").asList()->size() == 2)) {
            yError() << LogPrefix << "integrationBasedIKCorrectionGains option not found or not valid";
            return false;
        }

        yarp::os::Bottle* integrationBasedIKCorrectionGains = config.find("integrationBasedIKCorrectionGains").asList();
        pImpl->integrationBasedIKLinearCorrectionGain = integrationBasedIKCorrectionGains->get(0).asFloat64();
        pImpl->integrationBasedIKAngularCorrectionGain = integrationBasedIKCorrectionGains->get(1).asFloat64();
    }

    // ===================================
    // PRINT CURRENT CONFIGURATION OPTIONS
    // ===================================

    yInfo() << LogPrefix << "*** =========================";
    yInfo() << LogPrefix << "*** Period                  :" << period;
    yInfo() << LogPrefix << "*** Urdf file name          :" << urdfFileName;
    yInfo() << LogPrefix << "*** Global IK               :" << pImpl->useGlobalIK;
    yInfo() << LogPrefix << "*** Pair-Wised IK           :" << pImpl->usePairWisedIK;
    yInfo() << LogPrefix << "*** Integration Based IK    :" << pImpl->useIntegrationBasedIK;
    yInfo() << LogPrefix << "*** Use Xsens joint angles  :" << pImpl->useXsensJointsAngles;
    if (pImpl->usePairWisedIK || pImpl->useGlobalIK)
    {
        yInfo() << LogPrefix << "*** Allow IK failures       :" << pImpl->allowIKFailures;
        yInfo() << LogPrefix << "*** Max IK iterations       :" << pImpl->maxIterationsIK;
        yInfo() << LogPrefix << "*** Cost Tolerance          :" << pImpl->costTolerance;
        yInfo() << LogPrefix << "*** IK Solver Name          :" << pImpl->solverName;
        yInfo() << LogPrefix << "*** Position target weight  :" << pImpl->posTargetWeight;
        yInfo() << LogPrefix << "*** Rotation target weight  :" << pImpl->rotTargetWeight;
        yInfo() << LogPrefix << "*** Cost regularization     :" << pImpl->costRegularization;
        yInfo() << LogPrefix << "*** Size of thread pool     :" << pImpl->ikPoolSize;
    }
    if (pImpl->useIntegrationBasedIK)
    {
        yInfo() << LogPrefix << "*** Linear correction gain  :" << pImpl->integrationBasedIKLinearCorrectionGain;
        yInfo() << LogPrefix << "*** Angular correction gain :" << pImpl->integrationBasedIKAngularCorrectionGain;
    }
    yInfo() << LogPrefix << "*** =========================";

    // ==========================
    // INITIALIZE THE HUMAN MODEL
    // ==========================

    auto& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string urdfFilePath = rf.findFile(urdfFileName);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("urdf").asString();
        return false;
    }

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << urdfFilePath;
        return false;
    }

    // Get the model from the loader
    pImpl->humanModel = modelLoader.model();

    // ================================
    // INITIALIZE LINK / JOINTS BUFFERS
    // ================================

    const size_t nrOfJoints = pImpl->humanModel.getNrOfJoints();
    // const size_t nrOfJoints = pImpl->humanModel.getNrOfLinks();

    pImpl->solution.jointPositions.resize(nrOfJoints);
    pImpl->solution.jointVelocities.resize(nrOfJoints);

    pImpl->jointAngles.resize(nrOfJoints);
    pImpl->jointAngles.zero();

    pImpl->jointConfigurationSolution.resize(nrOfJoints);
    pImpl->jointConfigurationSolution.zero();

    pImpl->jointVelocitiesSolution.resize(nrOfJoints);
    pImpl->jointVelocitiesSolution.zero();

    pImpl->jointVelocitiesSolutionOld.resize(nrOfJoints);
    pImpl->jointVelocitiesSolutionOld.zero();

    // Get the model link names according to the modelToWearable link sensor map
    const size_t nrOfSegments = pImpl->wearableStorage.modelToWearable_LinkName.size();
    pImpl->segments.resize(nrOfSegments);
    int segmentIndex = 0;

    for (size_t linkIndex = 0; linkIndex < pImpl->humanModel.getNrOfLinks(); ++linkIndex) {
        // Get the name of the link from the model and its prefix from iWear
        std::string modelLinkName = pImpl->humanModel.getLinkName(linkIndex);

        if (pImpl->wearableStorage.modelToWearable_LinkName.find(modelLinkName)
                == pImpl->wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        pImpl->segments[segmentIndex].velocities.zero();

        // Store the name of the link as segment name
        pImpl->segments[segmentIndex].segmentName =  modelLinkName;
        segmentIndex++;
    }

    // Initialize kinDyn computation
    pImpl->kinDynComputations = std::unique_ptr<iDynTree::KinDynComputations>(new iDynTree::KinDynComputations());
    pImpl->kinDynComputations->loadRobotModel(modelLoader.model());

    // ========================
    // INITIALIZE PAIR-WISED IK
    // ========================

    if (pImpl->usePairWisedIK) {

        // Get all the possible pairs composing the model
        std::vector<std::pair<std::string, std::string>> pairNames;
        std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex> > pairSegmentIndeces;

        // Get the link pair names
        createEndEffectorsPairs(pImpl->humanModel, pImpl->segments, pairNames, pairSegmentIndeces);
        pImpl->linkPairs.reserve(pairNames.size());

        for (unsigned index = 0; index < pairNames.size(); ++index) {
            LinkPairInfo pairInfo;

            pairInfo.parentFrameName = pairNames[index].first;
            pairInfo.parentFrameSegmentsIndex = pairSegmentIndeces[index].first;

            pairInfo.childFrameName = pairNames[index].second;
            pairInfo.childFrameSegmentsIndex = pairSegmentIndeces[index].second;

            // Get the reduced pair model
            if (!getReducedModel(pImpl->humanModel, pairInfo.parentFrameName, pairInfo.childFrameName, pairInfo.pairModel)) {

                yWarning() << LogPrefix << "failed to get reduced model for the segment pair " << pairInfo.parentFrameName.c_str()
                           << ", " << pairInfo.childFrameName.c_str();
                continue;
            }

            // Allocate the ik solver
            pairInfo.ikSolver = std::make_unique<iDynTree::InverseKinematics>();

            // Set ik parameters
            pairInfo.ikSolver->setVerbosity(1);
            pairInfo.ikSolver->setLinearSolverName(pImpl->solverName);
            pairInfo.ikSolver->setMaxIterations(pImpl->maxIterationsIK);
            pairInfo.ikSolver->setCostTolerance(pImpl->costTolerance);
            pairInfo.ikSolver->setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
            pairInfo.ikSolver->setRotationParametrization(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

            // Set ik model
            if (!pairInfo.ikSolver->setModel(pairInfo.pairModel)) {
                yWarning() << LogPrefix << "failed to configure IK solver for the segment pair" << pairInfo.parentFrameName.c_str()
                           << ", " << pairInfo.childFrameName.c_str() <<  " Skipping pair";
                continue;
            }

            // Add parent link as fixed base constraint with identity transform
            pairInfo.ikSolver->addFrameConstraint(pairInfo.parentFrameName, iDynTree::Transform::Identity());

            // Add child link as a target and set initial transform to be identity
            pairInfo.ikSolver->addTarget(pairInfo.childFrameName, iDynTree::Transform::Identity());

            // Add target position and rotation weights
            pairInfo.positionTargetWeight = pImpl->posTargetWeight;
            pairInfo.rotationTargetWeight = pImpl->rotTargetWeight;

            // Add cost regularization term
            pairInfo.costRegularization = pImpl->costRegularization;

            // Get floating base for the pair model
            pairInfo.floatingBaseIndex = pairInfo.pairModel.getFrameLink(pairInfo.pairModel.getFrameIndex(pairInfo.parentFrameName));

            // Set ik floating base
            if (!pairInfo.ikSolver->setFloatingBaseOnFrameNamed(pairInfo.pairModel.getLinkName(pairInfo.floatingBaseIndex))) {
                yError() << "Failed to set floating base frame for the segment pair" << pairInfo.parentFrameName.c_str()
                         << ", " << pairInfo.childFrameName.c_str() <<  " Skipping pair";
                return false;
            }

            // Set initial joint positions size
            pairInfo.sInitial.resize(pairInfo.pairModel.getNrOfJoints());

            // Obtain the joint location index in full model and the lenght of DoFs i.e joints map
            // This information will be used to put the IK solutions together for the full model
            std::vector<std::string> solverJoints;

            // Resize to number of joints in the pair model
            solverJoints.resize(pairInfo.pairModel.getNrOfJoints());

            for (int i=0; i < pairInfo.pairModel.getNrOfJoints(); i++) {
                solverJoints[i] = pairInfo.pairModel.getJointName(i);
            }

            pairInfo.consideredJointLocations.reserve(solverJoints.size());
            for (auto &jointName: solverJoints) {
                iDynTree::JointIndex jointIndex = pImpl->humanModel.getJointIndex(jointName);
                if (jointIndex == iDynTree::JOINT_INVALID_INDEX) {
                    yWarning() << LogPrefix << "IK considered joint " << jointName << " not found in the complete model";
                    continue;
                }
                iDynTree::IJointConstPtr joint = pImpl->humanModel.getJoint(jointIndex);

                // Save location index and length of each DoFs
                pairInfo.consideredJointLocations.push_back(std::pair<size_t, size_t>(joint->getDOFsOffset(), joint->getNrOfDOFs()));
            }

            // Set the joint configurations size and initialize to zero
            pairInfo.jointConfigurations.resize(solverJoints.size());
            pairInfo.jointConfigurations.zero();

            // Set the joint velocities size and initialize to zero
            pairInfo.jointVelocities.resize(solverJoints.size());
            pairInfo.jointVelocities.zero();

            // Now configure the kinDynComputation object
            modelLoader.loadReducedModelFromFullModel(pImpl->humanModel, solverJoints);
            const iDynTree::Model &reducedModel = modelLoader.model();

            // Save the indeces
            // TODO: check if link or frame
            pairInfo.parentFrameModelIndex = reducedModel.getFrameIndex(pairInfo.parentFrameName);
            pairInfo.childFrameModelIndex = reducedModel.getFrameIndex(pairInfo.childFrameName);

            pairInfo.kinDynComputations = std::unique_ptr<iDynTree::KinDynComputations>(new iDynTree::KinDynComputations());

            pairInfo.kinDynComputations->loadRobotModel(modelLoader.model());

            pairInfo.parentJacobian.resize(6, modelLoader.model().getNrOfDOFs());
            pairInfo.parentJacobian.zero();
            pairInfo.childJacobian = pairInfo.relativeJacobian = pairInfo.parentJacobian;

            // Overwriting the default contructed object
            pairInfo.jacobianDecomposition = Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(pairInfo.relativeJacobian.rows(), pairInfo.relativeJacobian.cols());

            // Move the link pair instance into the vector
            pImpl->linkPairs.push_back(std::move(pairInfo));
        }

        // =========================
        // INITIALIZE IK WORKER POOL
        // =========================

        pImpl->ikPool = std::unique_ptr<IKWorkerPool>(new IKWorkerPool(pImpl->ikPoolSize,
                                                                                 pImpl->linkPairs,
                                                                                 pImpl->segments));

        if (!pImpl->ikPool) {
            yError() << LogPrefix << "failed to create IK worker pool";
            return false;
        }

    }

    // ====================
    // INITIALIZE GLOBAL IK
    // ====================

    if (pImpl->useGlobalIK) {

        // Set global ik parameters
        pImpl->globalIK.setVerbosity(1);
        pImpl->globalIK.setLinearSolverName(pImpl->solverName);
        pImpl->globalIK.setMaxIterations(pImpl->maxIterationsIK);
        pImpl->globalIK.setCostTolerance(pImpl->costTolerance);
        pImpl->globalIK.setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
        pImpl->globalIK.setRotationParametrization(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

        if (!pImpl->globalIK.setModel(pImpl->humanModel)) {
            yError() << LogPrefix << "globalIK: failed to load the model";
            return false;
        }

        if (!pImpl->globalIK.setFloatingBaseOnFrameNamed(pImpl->floatingBaseFrame.model)) {
            yError() << LogPrefix << "Failed to set the globalIK floating base frame on link"
                     << pImpl->floatingBaseFrame.model;
            return false;
        }

        for (size_t linkIndex = 0; linkIndex < pImpl->humanModel.getNrOfLinks(); ++linkIndex) {
            std::string linkName = pImpl->humanModel.getLinkName(linkIndex);

            // Insert in the cost the rotation and position of the link used as base
            if (linkName == pImpl->floatingBaseFrame.model) {
                if (!pImpl->useBaseMeasurementDirectlyFromXsens && !pImpl->globalIK.addTarget(linkName, iDynTree::Transform::Identity(), 1.0, 1.0)) {
                    yError() << LogPrefix << "Failed to add target for floating base link" << linkName;
                    askToStop();
                    return false;
                }
                continue;
            }

            // Add ik targets and set to identity
            if (!pImpl->globalIK.addTarget(linkName, iDynTree::Transform::Identity(), pImpl->posTargetWeight, pImpl->rotTargetWeight)) {
                yError() << LogPrefix << "Failed to add target for link" << linkName;
                askToStop();
                return false;
            }
        }
    }

    return true;
}

bool HumanStateProvider::close()
{
    return true;
}

// This method returns the all link pair names from the full human model
static void createEndEffectorsPairs(const iDynTree::Model& model,
                                    std::vector<SegmentInfo>& humanSegments,
                                    std::vector<std::pair<std::string, std::string> > &framePairs,
                                    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex> > &framePairIndeces)
{
    //for each element in human segments
    //extract it from the vector (to avoid duplications)
    //Look for it in the model and get neighbours
    std::vector<SegmentInfo> segments(humanSegments);
    size_t segmentCount = segments.size();

    while (!segments.empty()) {
        SegmentInfo segment = segments.back();
        segments.pop_back();
        segmentCount--;

        iDynTree::LinkIndex linkIndex = model.getLinkIndex(segment.segmentName);
        if (linkIndex < 0 || static_cast<unsigned>(linkIndex) >= model.getNrOfLinks()) {
            yWarning("Segment %s not found in the URDF model", segment.segmentName.c_str());
            continue;
        }

        //this for loop should not be necessary, but this can help keeps the backtrace short
        //as we do not assume that we can go back further that this node
        for (unsigned neighbourIndex = 0; neighbourIndex < model.getNrOfNeighbors(linkIndex); ++neighbourIndex) {
            //remember the "biforcations"
            std::stack<iDynTree::LinkIndex> backtrace;
            //and the visited nodes
            std::vector<iDynTree::LinkIndex> visited;

            //I've already visited the starting node
            visited.push_back(linkIndex);
            iDynTree::Neighbor neighbour = model.getNeighbor(linkIndex, neighbourIndex);
            backtrace.push(neighbour.neighborLink);

            while (!backtrace.empty()) {
                iDynTree::LinkIndex currentLink = backtrace.top();
                backtrace.pop();
                //add the current link to the visited
                visited.push_back(currentLink);

                std::string linkName = model.getLinkName(currentLink);

                // check if this is a human segment
                std::vector<SegmentInfo>::iterator foundSegment = std::find_if(segments.begin(),
                                                                               segments.end(),
                                                                               [&](SegmentInfo& frame){ return frame.segmentName == linkName; });
                if (foundSegment != segments.end()) {
                    std::vector<SegmentInfo>::difference_type foundLinkIndex = std::distance(segments.begin(), foundSegment);
                    //Found! This is a segment
                    framePairs.push_back(std::pair<std::string, std::string>(segment.segmentName, linkName));
                    framePairIndeces.push_back(std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>(segmentCount, foundLinkIndex));
                    break;
                }
                //insert all non-visited neighbours
                for (unsigned i = 0; i < model.getNrOfNeighbors(currentLink); ++i) {
                    iDynTree::LinkIndex link = model.getNeighbor(currentLink, i).neighborLink;
                    //check if we already visited this segment
                    if (std::find(visited.begin(), visited.end(), link) != visited.end()) {
                        //Yes => skip
                        continue;
                    }
                    backtrace.push(link);
                }
            }

        }
    }

}

static bool getReducedModel(const iDynTree::Model& modelInput,
                            const std::string& parentFrame,
                            const std::string& endEffectorFrame,
                            iDynTree::Model& modelOutput)
{
    iDynTree::FrameIndex parentFrameIndex;
    iDynTree::FrameIndex endEffectorFrameIndex;
    std::vector<std::string> consideredJoints;
    iDynTree::Traversal traversal;
    iDynTree::LinkIndex parentLinkIdx;
    iDynTree::IJointConstPtr joint;
    iDynTree::ModelLoader loader;

    // Get frame indices
    parentFrameIndex = modelInput.getFrameIndex(parentFrame);
    endEffectorFrameIndex = modelInput.getFrameIndex(endEffectorFrame);

    if(parentFrameIndex == iDynTree::FRAME_INVALID_INDEX){
        yError() << LogPrefix << " Invalid parent frame: "<< parentFrame;
        return false;
    }
    else if(endEffectorFrameIndex == iDynTree::FRAME_INVALID_INDEX){
        yError() << LogPrefix << " Invalid End Effector Frame: "<< endEffectorFrame;
        return false;
    }

    // Select joint from traversal
    modelInput.computeFullTreeTraversal(traversal, modelInput.getFrameLink(parentFrameIndex));

    iDynTree::LinkIndex visitedLink = modelInput.getFrameLink(endEffectorFrameIndex);

    while (visitedLink != traversal.getBaseLink()->getIndex())
    {
        parentLinkIdx = traversal.getParentLinkFromLinkIndex(visitedLink)->getIndex();
        joint = traversal.getParentJointFromLinkIndex(visitedLink);

        // Check if the joint is supported
        if(modelInput.getJoint(joint->getIndex())->getNrOfDOFs() == 1)
        {
            consideredJoints.insert(consideredJoints.begin(), modelInput.getJointName(joint->getIndex()));
        }
        else {
            yWarning() << LogPrefix << "Joint " << modelInput.getJointName(joint->getIndex()) << " is ignored as it has (" << modelInput.getJoint(joint->getIndex())->getNrOfDOFs() << " DOFs)";
        }

        visitedLink = parentLinkIdx;
    }

    if (!loader.loadReducedModelFromFullModel(modelInput, consideredJoints)) {
        std::cerr << LogPrefix << " failed to select joints: " ;
        for (std::vector< std::string >::const_iterator i = consideredJoints.begin(); i != consideredJoints.end(); ++i){
            std::cerr << *i << ' ';
        }
        std::cerr << std::endl;
        return false;

    }

    modelOutput = loader.model();

    return true;
}

void HumanStateProvider::run()
{
    auto tick_PW = std::chrono::high_resolution_clock::now();

    // Get the link transformations from input data
    if (!pImpl->getLinkTransformFromInputData(pImpl->linkTransformMatrices)) {
        yError() << LogPrefix << "Failed to get link transforms from input data";
        askToStop();
        return;
    }

    // Get link orientation from input data
    if (!pImpl->getLinkOrientationFromInputData(pImpl->linkOrientationMatrices)) {
        yError() << LogPrefix << "Failed to get link transforms from input data";
        askToStop();
        return;
    }

    // Get the link velocity from input data
    if (!pImpl->getLinkVelocityFromInputData(pImpl->linkVelocities)) {
        yError() << LogPrefix << "Failed to get link velocity from input data";
        askToStop();
        return;
    }

    // Get base transform from the suit
    iDynTree::Transform measuredBaseTransform;
    measuredBaseTransform = pImpl->linkTransformMatrices.at(pImpl->floatingBaseFrame.model);

    // Get base velocity from the suit
    iDynTree::Twist measuredBaseVelocity;
    measuredBaseVelocity = pImpl->linkVelocities.at(pImpl->floatingBaseFrame.model);

    if(pImpl->usePairWisedIK && !(pImpl->useIntegrationBasedIK && !(pImpl->firstRun))) {

        {
            std::lock_guard<std::mutex> lock(pImpl->mutex);

            // Set the initial solution to zero
            for (auto& linkPair : pImpl->linkPairs) {
                linkPair.sInitial.zero();
            }

        }

        {
            std::lock_guard<std::mutex> lock(pImpl->mutex);

            // Set link segments transformation
            for (size_t segmentIndex = 0; segmentIndex < pImpl->segments.size(); segmentIndex++) {

                SegmentInfo& segmentInfo = pImpl->segments.at(segmentIndex);
                segmentInfo.poseWRTWorld = pImpl->linkTransformMatrices.at(segmentInfo.segmentName);

                segmentInfo.velocities = pImpl->linkVelocities.at(segmentInfo.segmentName);
            }
        }

        // Call IK worker pool to solve
        pImpl->ikPool->runAndWait();

        // Joint link pair ik solutions using joints map from link pairs initialization
        // to solution struct for exposing data through interface
        for (auto& linkPair : pImpl->linkPairs) {
            size_t jointIndex = 0;
            for (auto& pairJoint : linkPair.consideredJointLocations) {

                // Check if it is a valid 1 DoF joint
                if (pairJoint.second == 1) {
                    pImpl->jointConfigurationSolution.setVal(pairJoint.first, linkPair.jointConfigurations.getVal(jointIndex));
                    pImpl->jointVelocitiesSolution.setVal(pairJoint.first, linkPair.jointVelocities.getVal(jointIndex));

                    jointIndex++;
                }
                else {
                    yWarning() << LogPrefix << " Invalid DoFs for the joint, skipping the ik solution for this joint";
                    continue;
                }

            }
        }

        // Use measured pose and velocity for the base frame
        pImpl->baseTransformSolution = measuredBaseTransform;
        pImpl->baseVelocitySolution = measuredBaseVelocity;

        auto tock_PW = std::chrono::high_resolution_clock::now();
        yDebug() << LogPrefix << "Pair-Wised IK took"
                 << std::chrono::duration_cast<std::chrono::milliseconds>(tock_PW - tick_PW).count() << "ms";
    }

    if (pImpl->useGlobalIK && !(pImpl->useIntegrationBasedIK && !(pImpl->firstRun))) {

        auto tick_G = std::chrono::high_resolution_clock::now();

        // Set global IK initial condition
        if (!pImpl->globalIK.setFullJointsInitialCondition(&pImpl->baseTransformSolution, &pImpl->jointConfigurationSolution)) {
            yError() << LogPrefix << "Failed to set the joint configuration for initializing the global IK";
            askToStop();
            return;
        }

        iDynTree::Transform linkTransform;

        // Update ik targets based on wearable input data
        for (size_t linkIndex = 0; linkIndex < pImpl->humanModel.getNrOfLinks(); ++linkIndex) {
            std::string linkName = pImpl->humanModel.getLinkName(linkIndex);

            // For the link used as base insert both the rotation and position cost if not using direcly measurement from xsens
            if (linkName == pImpl->floatingBaseFrame.model) {
                if (!pImpl->useBaseMeasurementDirectlyFromXsens && !pImpl->globalIK.updateTarget(linkName, pImpl->linkTransformMatrices.at(linkName), 1.0, 1.0)) {
                    yError() << LogPrefix << "Failed to update target for floating base" << linkName;
                    askToStop();
                    return;
                }
                continue;
            }

            // Skip links with no associated measures (use only links from the configuration)
            if (pImpl->wearableStorage.modelToWearable_LinkName.find(linkName)
                    == pImpl->wearableStorage.modelToWearable_LinkName.end()) {
                continue;
            }

            if (pImpl->linkTransformMatrices.find(linkName) == pImpl->linkTransformMatrices.end()) {
                yError() << LogPrefix << "Failed to find transformation matrix for link" << linkName;
                askToStop();
                return;
            }

            linkTransform = pImpl->linkTransformMatrices.at(linkName);
            // if useBaseMeasurementDirectlyFromXsens, use the link transform relative to the base
            if (pImpl->useBaseMeasurementDirectlyFromXsens)
            {
                linkTransform = measuredBaseTransform.inverse() * linkTransform;
            }

            if (!pImpl->globalIK.updateTarget(linkName, linkTransform, pImpl->posTargetWeight, pImpl->rotTargetWeight)) {
                yError() << LogPrefix << "Failed to update target for link" << linkName;
                askToStop();
                return;
            }
        }

        // Use a postural task for regularization
        iDynTree::VectorDynSize posturalTaskJointAngles;
        posturalTaskJointAngles.resize(pImpl->jointAngles.size());
        posturalTaskJointAngles.zero();
        if (!pImpl->globalIK.setDesiredFullJointsConfiguration(posturalTaskJointAngles,pImpl->costRegularization)) {
             yError() << LogPrefix << "Failed to set the postural configuration of the IK";
             askToStop();
             return;
         }

        if (!pImpl->globalIK.solve()) {
                yError() << LogPrefix << "Failed to solve global IK";
        }

        // Get the global inverse kinematics solution
        pImpl->globalIK.getFullJointsSolution(pImpl->baseTransformSolution, pImpl->jointConfigurationSolution);

        // if useBaseMeasurementDirectlyFromXsens use directly the measurement of the base pose from the base link coming from Xsens and compute only the joint velocities solution
        if (pImpl->useBaseMeasurementDirectlyFromXsens)
        {
            pImpl->baseTransformSolution = measuredBaseTransform;
            pImpl->computeJointVelocities(pImpl->jointConfigurationSolution, pImpl->linkVelocities, pImpl->jointVelocitiesSolution);
            pImpl->baseVelocitySolution = measuredBaseVelocity;
        }
        else
        {
            pImpl->computeVelocities(pImpl->jointConfigurationSolution, pImpl->baseTransformSolution, pImpl->linkVelocities, pImpl->jointVelocitiesSolution, pImpl->baseVelocitySolution);
        }

        auto tock_G = std::chrono::high_resolution_clock::now();
        yDebug() << LogPrefix << "Global IK took"
                 << std::chrono::duration_cast<std::chrono::milliseconds>(tock_G - tick_G).count() << "ms";
    }

    if (pImpl->useIntegrationBasedIK && !(pImpl->firstRun))
    {

        auto tick_IB = std::chrono::high_resolution_clock::now();
        //compute timestep
        double dt;
            if (pImpl->lastTime < 0.0)
        {
            dt = this->getPeriod();
        }
        else
        {
            dt = yarp::os::Time::now()-pImpl->lastTime;
        };
        pImpl->lastTime = yarp::os::Time::now();

        // correction term for link velocities
        iDynTree::KinDynComputations *computations = pImpl->kinDynComputations.get();
        iDynTree::Vector3 worldGravity;
        worldGravity.zero();
        worldGravity(2) = -9.81;
        if (pImpl->useBaseMeasurementDirectlyFromXsens)
        {
            computations->setRobotState(pImpl->jointConfigurationSolution, pImpl->jointVelocitiesSolution, worldGravity);
        }
        else
        {
            computations->setRobotState(pImpl->baseTransformSolution, pImpl->jointConfigurationSolution, pImpl->baseVelocitySolution, pImpl->jointVelocitiesSolution, worldGravity);
        }

        for (size_t linkIndex = 0; linkIndex < pImpl->humanModel.getNrOfLinks(); ++linkIndex) {
            std::string linkName = pImpl->humanModel.getLinkName(linkIndex);

            // skip fake links
            if (pImpl->wearableStorage.modelToWearable_LinkName.find(linkName)
                    == pImpl->wearableStorage.modelToWearable_LinkName.end()) {
                continue;
            }

            iDynTree::Rotation rotationError = computations->getWorldTransform(pImpl->humanModel.getFrameIndex(linkName)).getRotation() * pImpl->linkTransformMatrices[linkName].getRotation().inverse();
            iDynTree::Vector3 angularVelocityCorrection;

            angularVelocityCorrection = iDynTreeHelper::Rotation::skewVee(rotationError);

            // for floating base link use error also on position if not useBaseMeasurementDirectlyFromXsens, otherwise skip the link
            if (linkName == pImpl->floatingBaseFrame.model) {
               if (pImpl->useBaseMeasurementDirectlyFromXsens)
               {
                   continue;
               }

               iDynTree::Vector3 linearVelocityCorrection;
               linearVelocityCorrection = computations->getWorldTransform(pImpl->humanModel.getFrameIndex(linkName)).getPosition() - pImpl->linkTransformMatrices[linkName].getPosition();
               for (int i=0; i<3; i++) {
                   pImpl->linkVelocities[linkName].setVal(i, pImpl->linkVelocities[linkName].getVal(i) - pImpl->integrationBasedIKLinearCorrectionGain * linearVelocityCorrection.getVal(i));
               }
            }

            // correct the links angular velocities
            for (int i=3; i<6; i++) {
                pImpl->linkVelocities[linkName].setVal(i, pImpl->linkVelocities[linkName].getVal(i) - pImpl->integrationBasedIKAngularCorrectionGain * angularVelocityCorrection.getVal(i-3));
            }
        }

        // if useBaseMeasurementDirectlyFromXsens use directly the measurement of the base pose from the base link coming from Xsens and compute only the joint velocities solution
        if (pImpl->useBaseMeasurementDirectlyFromXsens)
        {
            pImpl->computeJointVelocities(pImpl->jointConfigurationSolution, pImpl->linkVelocities, pImpl->jointVelocitiesSolution);
            pImpl->baseVelocitySolution = measuredBaseVelocity;
        }
        else
        {
            pImpl->computeVelocities(pImpl->jointConfigurationSolution, pImpl->baseTransformSolution, pImpl->linkVelocities, pImpl->jointVelocitiesSolution, pImpl->baseVelocitySolution);
        }

        // integrate joint velocities to obtain joint position using Tustin formula
        for (unsigned i = 0; i < pImpl->jointConfigurationSolution.size(); ++i) {
            pImpl->jointConfigurationSolution.setVal(i, pImpl->jointConfigurationSolution.getVal(i) + (pImpl->jointVelocitiesSolution.getVal(i) + pImpl->jointVelocitiesSolutionOld.getVal(i)) * dt / 2);
        }

        if (!pImpl->useBaseMeasurementDirectlyFromXsens)
        {
            // integrate base linear velocity to obtain base position using Tustin formula
            iDynTree::Position basePositionSolution;
            for (unsigned i = 0; i < 3; i++) {
               iDynTree::toEigen(basePositionSolution) = iDynTree::toEigen(pImpl->baseTransformSolution.getPosition()) + (iDynTree::toEigen(pImpl->baseVelocitySolution.getLinearVec3()) + iDynTree::toEigen(pImpl->baseVelocitySolutionOld.getLinearVec3())) * dt / 2;
            }
            pImpl->baseTransformSolution.setPosition(basePositionSolution);

            // integrato base angular velocity to obtain base rotation
            iDynTree::Rotation baseRotation = pImpl->baseTransformSolution.getRotation();

            iDynTree::Matrix3x3 dBaseRotation;
            iDynTree::toEigen(dBaseRotation) = iDynTree::skew(iDynTree::toEigen(pImpl->baseVelocitySolution.getAngularVec3())) * iDynTree::toEigen(baseRotation);

            iDynTree::Matrix3x3 dBaseRotationCorrection;
            // first order approximation correction
            iDynTree::toEigen(dBaseRotationCorrection) = (iDynTree::toEigen(iDynTree::Rotation::Identity()) - iDynTree::toEigen(baseRotation) * iDynTree::toEigen(baseRotation).transpose()) * iDynTree::toEigen(baseRotation) / (2 * dt);
            // second order approximation correction
            // iDynTree::toEigen(dBaseRotationCorrection) = (iDynTree::toEigen(baseRotation) + iDynTree::toEigen(dBaseRotation)) * (iDynTree::toEigen(iDynTree::Rotation::Identity()) - iDynTree::toEigen(baseRotation).transpose() * iDynTree::toEigen(baseRotation) - iDynTree::toEigen(baseRotation) * iDynTree::toEigen(dBaseRotation).transpose() * iDynTree::toEigen(dBaseRotation) * dt * dt) / (2 * dt);


            iDynTree::toEigen(dBaseRotation) = iDynTree::toEigen(dBaseRotation) + iDynTree::toEigen(dBaseRotationCorrection);
            iDynTree::toEigen(baseRotation) = iDynTree::toEigen(baseRotation) + iDynTree::toEigen(dBaseRotation) * dt;


            float baseRotationDeterminant = baseRotation.getVal(0,0) * (baseRotation.getVal(1,1) * baseRotation.getVal(2,2) - baseRotation.getVal(1,2) * baseRotation.getVal(2,1)) + baseRotation.getVal(0, 1) * (baseRotation.getVal(1,2) * baseRotation.getVal(2,0) - baseRotation.getVal(1,0) * baseRotation.getVal(2,2)) +
                     baseRotation.getVal(0, 2) * (baseRotation.getVal(1,0) * baseRotation.getVal(2,1) - baseRotation.getVal(1,1) * baseRotation.getVal(2,0));

            iDynTree::Vector4 quaternion = baseRotation.asQuaternion();

            baseRotation.fromQuaternion(quaternion);
            baseRotationDeterminant = baseRotation.getVal(0,0) * (baseRotation.getVal(1,1) * baseRotation.getVal(2,2) - baseRotation.getVal(1,2) * baseRotation.getVal(2,1)) + baseRotation.getVal(0, 1) * (baseRotation.getVal(1,2) * baseRotation.getVal(2,0) - baseRotation.getVal(1,0) * baseRotation.getVal(2,2)) +
                                 baseRotation.getVal(0, 2) * (baseRotation.getVal(1,0) * baseRotation.getVal(2,1) - baseRotation.getVal(1,1) * baseRotation.getVal(2,0));

            pImpl->baseTransformSolution.setRotation(baseRotation);
        }
        else
        {
            pImpl->baseTransformSolution = measuredBaseTransform;
        }

        // store velocities
        pImpl->baseVelocitySolutionOld = pImpl->baseVelocitySolution;
        pImpl->jointVelocitiesSolutionOld = pImpl->jointVelocitiesSolution;

        auto tock_IB = std::chrono::high_resolution_clock::now();
        yDebug() << LogPrefix << "Integral Based IK took"
                 << std::chrono::duration_cast<std::chrono::milliseconds>(tock_IB - tick_IB).count() << "ms";
    }

    // Expose IK solution for IHumanState
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);

        for (unsigned i = 0; i < pImpl->jointConfigurationSolution.size(); ++i) {
            pImpl->solution.jointPositions[i] = pImpl->jointConfigurationSolution.getVal(i);
            pImpl->solution.jointVelocities[i] = pImpl->jointVelocitiesSolution.getVal(i);
        }

        pImpl->solution.basePosition = {pImpl->baseTransformSolution.getPosition().getVal(0),
                                        pImpl->baseTransformSolution.getPosition().getVal(1),
                                        pImpl->baseTransformSolution.getPosition().getVal(2)};

        pImpl->solution.baseOrientation = {
            pImpl->baseTransformSolution.getRotation().asQuaternion().getVal(0),
            pImpl->baseTransformSolution.getRotation().asQuaternion().getVal(1),
            pImpl->baseTransformSolution.getRotation().asQuaternion().getVal(2),
            pImpl->baseTransformSolution.getRotation().asQuaternion().getVal(3),
        };

        // Use measured base frame velocity
        pImpl->solution.baseVelocity = {
            pImpl->baseVelocitySolution.getVal(0),
            pImpl->baseVelocitySolution.getVal(1),
            pImpl->baseVelocitySolution.getVal(2),
            pImpl->baseVelocitySolution.getVal(3),
            pImpl->baseVelocitySolution.getVal(4),
            pImpl->baseVelocitySolution.getVal(5)
        };
    }

    // compute the inverse kinematic errors
    pImpl->computeLinksOrientationErrors(pImpl->linkOrientationMatrices, pImpl->jointConfigurationSolution, pImpl->baseTransformSolution, pImpl->linkErrorOrientations);
    pImpl->computeLinksAngularVelocityErrors(pImpl->linkVelocities, pImpl->jointConfigurationSolution, pImpl->baseTransformSolution, pImpl->jointVelocitiesSolution, pImpl->baseVelocitySolution, pImpl->linkErrorAngularVelocities);

    // iDynTreeHelper::Rotation::rotationDistance distance;
    if (pImpl->firstRun)
    {
        // initialize integrator
        if (pImpl->useIntegrationBasedIK) {
            pImpl->jointVelocitiesSolutionOld = pImpl->jointVelocitiesSolution;
            pImpl->baseVelocitySolutionOld = pImpl->baseVelocitySolution;
        }
        pImpl->firstRun = false;
        pImpl->startTime = yarp::os::Time::now();
    }

    // reset integral after 2 seconds
//    if (pImpl->lastTime - pImpl->startTime > 2.0)
//    {
//        pImpl->firstRun = true;
//        pImpl->lastTime = -1.0;
//    }
}

bool HumanStateProvider::impl::getLinkTransformFromInputData(
        std::unordered_map<std::string, iDynTree::Transform>& transforms)
{
    for (const auto& linkMapEntry : wearableStorage.modelToWearable_LinkName) {
        const ModelLinkName& modelLinkName = linkMapEntry.first;
        const WearableLinkName& wearableLinkName = linkMapEntry.second;

        if (wearableStorage.linkSensorsMap.find(wearableLinkName)
                == wearableStorage.linkSensorsMap.end()
                || !wearableStorage.linkSensorsMap.at(wearableLinkName)) {
            yError() << LogPrefix << "Failed to get" << wearableLinkName
                     << "sensor from the device. Something happened after configuring it.";
            return false;
        }

        const wearable::SensorPtr<const sensor::IVirtualLinkKinSensor> sensor =
                wearableStorage.linkSensorsMap.at(wearableLinkName);

        if (!sensor) {
            yError() << LogPrefix << "Sensor" << wearableLinkName
                     << "has been added but not properly configured";
            return false;
        }

        if (sensor->getSensorStatus() != sensor::SensorStatus::Ok) {
            yError() << LogPrefix << "The sensor status of" << sensor->getSensorName()
                     << "is not ok (" << static_cast<double>(sensor->getSensorStatus()) << ")";
            return false;
        }

        wearable::Vector3 position;
        if (!sensor->getLinkPosition(position)) {
            yError() << LogPrefix << "Failed to read link position from virtual link sensor";
            return false;
        }

        iDynTree::Position pos(position.at(0),
                               position.at(1),
                               position.at(2));

        Quaternion orientation;
        if (!sensor->getLinkOrientation(orientation)) {
            yError() << LogPrefix << "Failed to read link orientation from virtual link sensor";
            return false;
        }

        iDynTree::Rotation rotation;
        rotation.fromQuaternion({orientation.data(), 4});

        iDynTree::Transform transform(rotation, pos);

        // Note that this map is used during the IK step for setting a target transform to a
        // link of the model. For this reason the map keys are model names.
        transforms[modelLinkName] = std::move(transform);
    }

    return true;
}

bool HumanStateProvider::impl::getLinkVelocityFromInputData(
        std::unordered_map<std::string, iDynTree::Twist>& velocities)
{
    for (const auto& linkMapEntry : wearableStorage.modelToWearable_LinkName) {
        const ModelLinkName& modelLinkName = linkMapEntry.first;
        const WearableLinkName& wearableLinkName = linkMapEntry.second;

        if (wearableStorage.linkSensorsMap.find(wearableLinkName)
                == wearableStorage.linkSensorsMap.end()
                || !wearableStorage.linkSensorsMap.at(wearableLinkName)) {
            yError() << LogPrefix << "Failed to get" << wearableLinkName
                     << "sensor from the device. Something happened after configuring it.";
            return false;
        }

        const wearable::SensorPtr<const sensor::IVirtualLinkKinSensor> sensor =
                wearableStorage.linkSensorsMap.at(wearableLinkName);

        if (!sensor) {
            yError() << LogPrefix << "Sensor" << wearableLinkName
                     << "has been added but not properly configured";
            return false;
        }

        if (sensor->getSensorStatus() != sensor::SensorStatus::Ok) {
            yError() << LogPrefix << "The sensor status of" << sensor->getSensorName()
                     << "is not ok (" << static_cast<double>(sensor->getSensorStatus()) << ")";
            return false;
        }

        wearable::Vector3 linearVelocity;
        if (!sensor->getLinkLinearVelocity(linearVelocity)) {
            yError() << LogPrefix << "Failed to read link linear velocity from virtual link sensor";
            return false;
        }

        wearable::Vector3 angularVelocity;
        if (!sensor->getLinkAngularVelocity(angularVelocity)) {
            yError() << LogPrefix << "Failed to read link angular velocity from virtual link sensor";
            return false;
        }

        velocities[modelLinkName].setVal(0, linearVelocity.at(0));
        velocities[modelLinkName].setVal(1, linearVelocity.at(1));
        velocities[modelLinkName].setVal(2, linearVelocity.at(2));
        velocities[modelLinkName].setVal(3, angularVelocity.at(0));
        velocities[modelLinkName].setVal(4, angularVelocity.at(1));
        velocities[modelLinkName].setVal(5, angularVelocity.at(2));
    }

    return true;
}

bool HumanStateProvider::impl::getLinkOrientationFromInputData(
        std::unordered_map<std::string, iDynTree::Rotation>& transforms)
{
    for (const auto& linkMapEntry : wearableStorage.modelToWearable_LinkName) {
        const ModelLinkName& modelLinkName = linkMapEntry.first;
        const WearableLinkName& wearableLinkName = linkMapEntry.second;

        if (wearableStorage.linkSensorsMap.find(wearableLinkName)
                == wearableStorage.linkSensorsMap.end()
                || !wearableStorage.linkSensorsMap.at(wearableLinkName)) {
            yError() << LogPrefix << "Failed to get" << wearableLinkName
                     << "sensor from the device. Something happened after configuring it.";
            return false;
        }

        const wearable::SensorPtr<const sensor::IVirtualLinkKinSensor> sensor =
                wearableStorage.linkSensorsMap.at(wearableLinkName);

        if (!sensor) {
            yError() << LogPrefix << "Sensor" << wearableLinkName
                     << "has been added but not properly configured";
            return false;
        }

        if (sensor->getSensorStatus() != sensor::SensorStatus::Ok) {
            yError() << LogPrefix << "The sensor status of" << sensor->getSensorName()
                     << "is not ok (" << static_cast<double>(sensor->getSensorStatus()) << ")";
            return false;
        }

        Quaternion orientation;
        if (!sensor->getLinkOrientation(orientation)) {
            yError() << LogPrefix << "Failed to read link orientation from virtual link sensor";
            return false;
        }

        iDynTree::Rotation rotation;
        rotation.fromQuaternion({orientation.data(), 4});

        // Note that this map is used during the IK step for setting a target orientation to a
        // link of the model. For this reason the map keys are model names.
        transforms[modelLinkName] = std::move(rotation);
    }

    return true;
}

bool HumanStateProvider::impl::getJointAnglesFromInputData(iDynTree::VectorDynSize& jointAngles)
{
    for (const auto& jointMapEntry : wearableStorage.modelToWearable_JointInfo) {
        const ModelJointName& modelJointName = jointMapEntry.first;
        const WearableJointInfo& wearableJointInfo = jointMapEntry.second;

        if (wearableStorage.jointSensorsMap.find(wearableJointInfo.name)
                == wearableStorage.jointSensorsMap.end()
                || !wearableStorage.jointSensorsMap.at(wearableJointInfo.name)) {
            yError() << LogPrefix << "Failed to get" << wearableJointInfo.name
                     << "sensor from the device. Something happened after configuring it.";
            return false;
        }

        const wearable::SensorPtr<const sensor::IVirtualSphericalJointKinSensor> sensor =
                wearableStorage.jointSensorsMap.at(wearableJointInfo.name);

        if (!sensor) {
            yError() << LogPrefix << "Sensor" << wearableJointInfo.name
                     << "has been added but not properly configured";
            return false;
        }

        if (sensor->getSensorStatus() != sensor::SensorStatus::Ok) {
            yError() << LogPrefix << "The sensor status of " << sensor->getSensorName()
                     << " is not ok (" << static_cast<double>(sensor->getSensorStatus()) << ")";
            return false;
        }

        Vector3 anglesXYZ;
        if (!sensor->getJointAnglesAsRPY(anglesXYZ)) {
            yError() << LogPrefix << "Failed to read joint angles from virtual joint sensor";
            return false;
        }

        // Since anglesXYZ describes a spherical joint, take the right component
        // (specified in the configuration file)
        // TODO: we still need to validate the Xsens convention. Particularly, the zeros of
        //       the joint angles might be different.
        jointAngles.setVal(humanModel.getJointIndex(modelJointName),
                           anglesXYZ[wearableJointInfo.index]);
    }

    return true;
}

bool HumanStateProvider::impl::getRealLinkJacobiansRelativeToBase(iDynTree::VectorDynSize jointConfigurations, std::unordered_map<std::string, iDynTree::MatrixDynSize>& jacobians)
{
    iDynTree::Vector3 worldGravity;
    worldGravity.zero();
    worldGravity(2) = -9.81;

    iDynTree::VectorDynSize zeroJointVelocities = jointConfigurations;
    zeroJointVelocities.zero();

    iDynTree::KinDynComputations *computations = kinDynComputations.get();
    computations->setRobotState(jointConfigurations, zeroJointVelocities, worldGravity);

    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        std::string linkName = humanModel.getLinkName(linkIndex);

        // skip fake links
        if (wearableStorage.modelToWearable_LinkName.find(linkName)
                == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        // skip floating base link
        if (linkName == floatingBaseFrame.model) {
            continue;
        }

        iDynTree::MatrixDynSize relativeJacobian(6, humanModel.getNrOfLinks());
        computations->getRelativeJacobian(humanModel.getFrameIndex(floatingBaseFrame.model), linkIndex, relativeJacobian);

        jacobians[linkName] = std::move(relativeJacobian);
    }

    return true;
}

bool HumanStateProvider::impl::computeJointVelocities(iDynTree::VectorDynSize jointConfigurations, std::unordered_map<std::string, iDynTree::Twist> linkVelocitiesMap, iDynTree::VectorDynSize& jointVelocities)
{
    iDynTree::VectorDynSize fullLinkVelocitiesRelativeToBase(3 * linkVelocitiesMap.size());
    fullLinkVelocitiesRelativeToBase.zero();

    iDynTree::MatrixDynSize fullLinkJacobianRelativeToBase(3 * linkVelocitiesMap.size(), jointConfigurations.size());
    fullLinkJacobianRelativeToBase.zero();

    std::unordered_map<std::string, iDynTree::MatrixDynSize> linkJacobiansRelativeToBase;

    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > jacobianDecomposition;
    jacobianDecomposition = Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >(fullLinkJacobianRelativeToBase.rows(), fullLinkJacobianRelativeToBase.cols());

    if(!getRealLinkJacobiansRelativeToBase(jointConfigurations, linkJacobiansRelativeToBase))
    {
        yError() << LogPrefix << "Failed to compute the link Jacobians";
        return false;
    }

    int linkCount = 0;
    for (const auto& linkMapEntry : linkJacobiansRelativeToBase) {
        const ModelLinkName& linkName = linkMapEntry.first;
        for (int i=3; i<6; i++)
        {
            for (int j=0; j<humanModel.getNrOfJoints(); j++)
            {
                fullLinkJacobianRelativeToBase.setVal(3 * linkCount + i, j, linkJacobiansRelativeToBase[linkName].getVal(i, j));
            }
            fullLinkVelocitiesRelativeToBase.setVal(3 * linkCount + i, linkVelocitiesMap[linkName].getVal(i) - linkVelocitiesMap[floatingBaseFrame.model].getVal(i));
        }
        linkCount = linkCount + 1;
    }

    //Pseudo-invert the Full Jacobian
    //Compute the QR decomposition
    jacobianDecomposition.compute(iDynTree::toEigen(fullLinkJacobianRelativeToBase));
    // the solve method on the decomposition directly solves the associated least-squares problem
    iDynTree::toEigen(jointVelocities) = jacobianDecomposition.solve(iDynTree::toEigen(fullLinkVelocitiesRelativeToBase));

    return true;
}

bool HumanStateProvider::impl::getRealLinkJacobians(iDynTree::VectorDynSize jointConfigurations, iDynTree::Transform floatingBasePose, std::unordered_map<std::string, iDynTree::MatrixDynSize>& jacobians)
{
    iDynTree::Vector3 worldGravity;
    worldGravity.zero();
    worldGravity(2) = -9.81;

    iDynTree::VectorDynSize zeroJointVelocities = jointConfigurations;
    zeroJointVelocities.zero();

    iDynTree::Twist zeroBaseVelocity;
    zeroBaseVelocity.zero();

    iDynTree::KinDynComputations *computations = kinDynComputations.get();
    computations->setRobotState(floatingBasePose, jointConfigurations, zeroBaseVelocity, zeroJointVelocities, worldGravity);

    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        std::string linkName = humanModel.getLinkName(linkIndex);

        // skip fake links
        if (wearableStorage.modelToWearable_LinkName.find(linkName)
                == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        iDynTree::MatrixDynSize jacobian(6, humanModel.getNrOfLinks() + 6);
        computations->getFrameFreeFloatingJacobian(linkName, jacobian);

        jacobians[linkName] = std::move(jacobian);
    }

    return true;
}

bool HumanStateProvider::impl::computeVelocities(iDynTree::VectorDynSize jointConfigurations, iDynTree::Transform floatingBasePose, std::unordered_map<std::string, iDynTree::Twist> linkVelocitiesMap, iDynTree::VectorDynSize& jointVelocities, iDynTree::Twist& baseVelocity)
{
    iDynTree::VectorDynSize fullLinkVelocities(3 + 3 * linkVelocitiesMap.size());
    fullLinkVelocities.zero();

    iDynTree::MatrixDynSize fullLinkJacobian(3 + 3 * linkVelocitiesMap.size(), jointConfigurations.size() + 6);
    fullLinkJacobian.zero();

    std::unordered_map<std::string, iDynTree::MatrixDynSize> linkJacobians;

    Eigen::FullPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > jacobianDecomposition;
    jacobianDecomposition = Eigen::FullPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >(fullLinkJacobian.rows(), fullLinkJacobian.cols());

    // Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> jacobianDecomposition;
    // jacobianDecomposition = Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(fullLinkJacobian.rows(), fullLinkJacobian.cols());

    if(!getRealLinkJacobians(jointConfigurations, floatingBasePose, linkJacobians))
    {
        yError() << LogPrefix << "Failed to compute the link Jacobians";
        return false;
    }

    int linkCount = 0;
    for (const auto& linkMapEntry : linkJacobians) {
        const ModelLinkName& linkName = linkMapEntry.first;

        if (linkName == floatingBaseFrame.model)
        {
            for (int i=0; i<6; i++)
            {
                for (int j=0; j<(humanModel.getNrOfJoints() + 6); j++)
                {
                    if ( i==j )
                    {
                        fullLinkJacobian.setVal(i, j, 1);
                    }
                    else
                    {
                        fullLinkJacobian.setVal(i, j, 0);
                    }
                }
                fullLinkVelocities.setVal(i, linkVelocitiesMap[linkName].getVal(i));
            }
        }
        else
        {
            for (int i=0; i<3; i++)
            {
                for (int j=0; j<(humanModel.getNrOfJoints() + 6); j++)
                {

                    fullLinkJacobian.setVal(6 + 3 * linkCount + i, j, linkJacobians[linkName].getVal(i + 3, j));
                }
                fullLinkVelocities.setVal(6 + 3 * linkCount + i, linkVelocitiesMap[linkName].getVal(i + 3));


            }
            linkCount = linkCount + 1;
        }
    }

    //Pseudo-invert the Full Jacobian
    //Compute the QR decomposition
    jacobianDecomposition.compute(iDynTree::toEigen(fullLinkJacobian));
    // the solve method on the decomposition directly solves the associated least-squares problem
    iDynTree::VectorDynSize nu(jointConfigurations.size() + 6);
    iDynTree::toEigen(nu) = jacobianDecomposition.solve(iDynTree::toEigen(fullLinkVelocities));

    baseVelocity.setVal(0, nu.getVal(0));
    baseVelocity.setVal(1, nu.getVal(1));
    baseVelocity.setVal(2, nu.getVal(2));
    baseVelocity.setVal(3, nu.getVal(3));
    baseVelocity.setVal(4, nu.getVal(4));
    baseVelocity.setVal(5, nu.getVal(5));

    for (int k=6; k< jointConfigurations.size() + 6; k++)
    {
        jointVelocities.setVal(k-6, nu.getVal(k));
    }

    return true;
}

bool HumanStateProvider::impl::computeLinksOrientationErrors(std::unordered_map<std::string, iDynTree::Rotation> linkDesiredOrientations, iDynTree::VectorDynSize jointConfigurations, iDynTree::Transform floatingBasePose, std::unordered_map<std::string, iDynTreeHelper::Rotation::rotationDistance>& linkErrorOrientations)
{
    iDynTree::Vector3 worldGravity;
    worldGravity.zero();
    worldGravity(2) = -9.81;

    iDynTree::VectorDynSize zeroJointVelocities = jointConfigurations;
    zeroJointVelocities.zero();

    iDynTree::Twist zeroBaseVelocity;
    zeroBaseVelocity.zero();

    iDynTree::KinDynComputations *computations = kinDynComputations.get();
    computations->setRobotState(floatingBasePose, jointConfigurations, zeroBaseVelocity, zeroJointVelocities, worldGravity);

    for (const auto& linkMapEntry : linkDesiredOrientations) {
        const ModelLinkName& linkName = linkMapEntry.first;
        linkErrorOrientations[linkName] = iDynTreeHelper::Rotation::rotationDistance(computations->getWorldTransform(linkName).getRotation(), linkDesiredOrientations[linkName]); // computations->getWorldTransform(linkName).getRotation() * linkDesiredOrientations[linkName].inverse();
    }

    return true;
}

bool HumanStateProvider::impl::computeLinksAngularVelocityErrors(std::unordered_map<std::string, iDynTree::Twist> linkDesiredVelocities, iDynTree::VectorDynSize jointConfigurations, iDynTree::Transform floatingBasePose, iDynTree::VectorDynSize jointVelocities, iDynTree::Twist baseVelocity, std::unordered_map<std::string, iDynTree::Vector3>& linkAngularVelocityError)
{
    iDynTree::Vector3 worldGravity;
    worldGravity.zero();
    worldGravity(2) = -9.81;

    iDynTree::KinDynComputations *computations = kinDynComputations.get();
    computations->setRobotState(floatingBasePose, jointConfigurations, baseVelocity, jointVelocities, worldGravity);

    for (const auto& linkMapEntry : linkDesiredVelocities) {
        const ModelLinkName& linkName = linkMapEntry.first;
        iDynTree::toEigen(linkAngularVelocityError[linkName]) = iDynTree::toEigen(linkDesiredVelocities[linkName].getLinearVec3()) - iDynTree::toEigen(computations->getFrameVel(linkName).getLinearVec3());
    }

    return true;
}

bool HumanStateProvider::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->iWear || !poly->view(pImpl->iWear) || !pImpl->iWear) {
        yError() << LogPrefix << "Failed to view the IWear interface from the PolyDriver";
        return false;
    }

    while (pImpl->iWear->getStatus() == WearStatus::WaitingForFirstRead) {
        yInfo() << LogPrefix << "IWear interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
    }

    if (pImpl->iWear->getStatus() != WearStatus::Ok) {
        yError() << LogPrefix << "The status of the attached IWear interface is not ok ("
                 << static_cast<int>(pImpl->iWear->getStatus()) << ")";
        return false;
    }

    // ===========
    // CHECK LINKS
    // ===========

    // Check that the attached IWear interface contains all the model links
    for (size_t linkIndex = 0; linkIndex < pImpl->humanModel.getNrOfLinks(); ++linkIndex) {
        // Get the name of the link from the model and its prefix from iWear
        std::string modelLinkName = pImpl->humanModel.getLinkName(linkIndex);

        if (pImpl->wearableStorage.modelToWearable_LinkName.find(modelLinkName)
                == pImpl->wearableStorage.modelToWearable_LinkName.end()) {
            //yWarning() << LogPrefix << "Failed to find" << modelLinkName
            //           << "entry in the configuration map. Skipping this link.";
            continue;
        }

        // Get the name of the sensor associated to the link
        std::string wearableLinkName =
                pImpl->wearableStorage.modelToWearable_LinkName.at(modelLinkName);

        // Try to get the sensor
        auto sensor = pImpl->iWear->getVirtualLinkKinSensor(wearableLinkName);
        if (!sensor) {
            //yError() << LogPrefix << "Failed to find sensor associated to link" << wearableLinkName
            //<< "from the IWear interface";
            return false;
        }

        // Create a sensor map entry using the wearable sensor name as key
        pImpl->wearableStorage.linkSensorsMap[wearableLinkName] =
                pImpl->iWear->getVirtualLinkKinSensor(wearableLinkName);
    }

    // Store the sensor associated as base
    std::string baseLinkSensorName = pImpl->floatingBaseFrame.wearable;
    if (pImpl->wearableStorage.linkSensorsMap.find(baseLinkSensorName)
            == pImpl->wearableStorage.linkSensorsMap.end()) {
        yError() << LogPrefix
                 << "Failed to find sensor associated with the base passed in the configuration"
                 << baseLinkSensorName;
        return false;
    }

    pImpl->wearableStorage.baseLinkSensor =
            pImpl->wearableStorage.linkSensorsMap.at(baseLinkSensorName);

    // ============
    // CHECK JOINTS
    // ============

    if (pImpl->useXsensJointsAngles) {
        yDebug() << "Checking joints";

        for (size_t jointIndex = 0; jointIndex < pImpl->humanModel.getNrOfJoints(); ++jointIndex) {
            // Get the name of the joint from the model and its prefix from iWear
            std::string modelJointName = pImpl->humanModel.getJointName(jointIndex);

            // Urdfs don't have support of spherical joints, IWear instead does.
            // We use the configuration for addressing this mismatch.
            if (pImpl->wearableStorage.modelToWearable_JointInfo.find(modelJointName)
                    == pImpl->wearableStorage.modelToWearable_JointInfo.end()) {
                yWarning() << LogPrefix << "Failed to find" << modelJointName
                           << "entry in the configuration map. Skipping this joint.";
                continue;
            }

            // Get the name of the sensor associate to the joint
            std::string wearableJointName =
                    pImpl->wearableStorage.modelToWearable_JointInfo.at(modelJointName).name;

            // Try to get the sensor
            auto sensor = pImpl->iWear->getVirtualSphericalJointKinSensor(wearableJointName);
            if (!sensor) {
                yError() << LogPrefix << "Failed to find sensor associated with joint"
                         << wearableJointName << "from the IWear interface";
                return false;
            }

            // Create a sensor map entry using the wearable sensor name as key
            pImpl->wearableStorage.jointSensorsMap[wearableJointName] = sensor;
        }
    }

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    yInfo() << LogPrefix << "attach() successful";
    return true;
}

void HumanStateProvider::threadRelease()
{
    if(!pImpl->ikPool->closeIKWorkerPool()) {
        yError() << LogPrefix << "Failed to close the IKWorker pool";
    }
}

bool HumanStateProvider::detach()
{
    while (isRunning()) {
        stop();
    }

    {
        std::lock_guard<std::mutex>(pImpl->mutex);
        pImpl->solution.clear();
    }

    pImpl->iWear = nullptr;
    return true;
}

bool HumanStateProvider::detachAll()
{
    return detach();
}

bool HumanStateProvider::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

std::vector<std::string> HumanStateProvider::getJointNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    std::vector<std::string> jointNames;

    for (size_t jointIndex = 0; jointIndex < pImpl->humanModel.getNrOfJoints(); ++jointIndex) {
        jointNames.emplace_back(pImpl->humanModel.getJointName(jointIndex));
    }

    return jointNames;
}

size_t HumanStateProvider::getNumberOfJoints() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->humanModel.getNrOfJoints();
}

std::string HumanStateProvider::getBaseName() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->floatingBaseFrame.model;
}

std::vector<double> HumanStateProvider::getJointPositions() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.jointPositions;
}

std::vector<double> HumanStateProvider::getJointVelocities() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.jointVelocities;
}

std::array<double, 6> HumanStateProvider::getBaseVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.baseVelocity;
}

std::array<double, 4> HumanStateProvider::getBaseOrientation() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.baseOrientation;
}

std::array<double, 3> HumanStateProvider::getBasePosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.basePosition;
}
