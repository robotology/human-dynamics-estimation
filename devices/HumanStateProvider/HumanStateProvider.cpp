/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStateProvider.h"
#include "HumanIKWorkerPool.h"

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
    std::unordered_map<std::string, iDynTree::Vector6> linkVelocities;
    iDynTree::VectorDynSize linkPairsJointConfigurationSolution;
    iDynTree::VectorDynSize globalJointConfigurationSolution;
    iDynTree::VectorDynSize jointVelocitiesSolution;

    // IK stuff
    std::string solverName;
    yarp::os::Value ikPoolOption;
    std::unique_ptr<HumanIKWorkerPool> ikPool;
    SolutionIK solution;

    int posTargetWeight;
    int rotTargetWeight;
    double costRegularization;

    bool useGlobalIK;
    iDynTree::InverseKinematics globalIK;

    bool getJointAnglesFromInputData(iDynTree::VectorDynSize& jointAngles);
    bool getLinkOrientationFromInputData(std::unordered_map<std::string, iDynTree::Rotation>& r);
    bool getLinkTransformFromInputData(std::unordered_map<std::string, iDynTree::Transform>& t);
    bool getLinkVelocityFromInputData(std::unordered_map<std::string, iDynTree::Vector6>& t);
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

    if (!(config.check("ikPoolSizeOption") && (config.find("ikPoolSizeOption").isString() || config.find("ikPoolSizeOption").isInt()))) {
        yError() << LogPrefix << "ikPoolOption option not found or not valid";
        return false;
    }

    if (!(config.check("posTargetWeight") && config.find("posTargetWeight").isInt())) {
        yError() << LogPrefix << "posTargetWeight option not found or not valid";
        return false;
    }

    if (!(config.check("rotTargetWeight") && config.find("rotTargetWeight").isInt())) {
        yError() << LogPrefix << "rotTargetWeight option not found or not valid";
        return false;
    }

    if (!(config.check("costRegularization") && config.find("costRegularization").isDouble())) {
        yError() << LogPrefix << "costRegularization option not found or not valid";
        return false;
    }

    if (!(config.check("useGlobalIK") && config.find("useGlobalIK").isBool())) {
        yError() << LogPrefix << "useGlobalIK option not found or not valid";
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

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    yarp::os::Bottle* floatingBaseFrameList = config.find("floatingBaseFrame").asList();

    pImpl->allowIKFailures = config.find("allowIKFailures").asBool();
    int maxIterationsIK = config.find("maxIterationsIK").asInt();
    double costTolerance = config.find("costTolerance").asFloat64();
    pImpl->solverName = config.find("ikLinearSolver").asString();
    pImpl->posTargetWeight = config.find("posTargetWeight").asInt();
    pImpl->rotTargetWeight = config.find("rotTargetWeight").asInt();
    pImpl->costRegularization = config.find("costRegularization").asDouble();
    pImpl->useGlobalIK = config.find("useGlobalIK").asBool();
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

    yInfo() << LogPrefix << "*** ========================";
    yInfo() << LogPrefix << "*** Period                 :" << period;
    yInfo() << LogPrefix << "*** Urdf file name         :" << urdfFileName;
    yInfo() << LogPrefix << "*** Allow IK failures      :" << pImpl->allowIKFailures;
    yInfo() << LogPrefix << "*** Max IK iterations      :" << maxIterationsIK;
    yInfo() << LogPrefix << "*** Cost Tolerance         :" << costTolerance;
    yInfo() << LogPrefix << "*** IK Solver Name         :" << pImpl->solverName;
    yInfo() << LogPrefix << "*** Position target weight :" << pImpl->posTargetWeight;
    yInfo() << LogPrefix << "*** Rotation target weight :" << pImpl->rotTargetWeight;
    yInfo() << LogPrefix << "*** Cost regularization     :" << pImpl->costRegularization;
    yInfo() << LogPrefix << "*** Global IK status       :" << pImpl->useGlobalIK;
    yInfo() << LogPrefix << "*** Use Xsens joint angles :" << pImpl->useXsensJointsAngles;
    yInfo() << LogPrefix << "*** ========================";

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

    pImpl->solution.jointPositions.resize(nrOfJoints);
    pImpl->solution.jointVelocities.resize(nrOfJoints);

    pImpl->jointAngles.resize(nrOfJoints);
    pImpl->jointAngles.zero();

    pImpl->linkPairsJointConfigurationSolution.resize(nrOfJoints);
    pImpl->linkPairsJointConfigurationSolution.zero();

    pImpl->globalJointConfigurationSolution.resize(nrOfJoints);
    pImpl->globalJointConfigurationSolution.zero();

    pImpl->jointVelocitiesSolution.resize(nrOfJoints);
    pImpl->jointVelocitiesSolution.zero();

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

        pImpl->segments[segmentIndex].velocities.resize(6);
        pImpl->segments[segmentIndex].velocities.zero();

        // Store the name of the link as segment name
        pImpl->segments[segmentIndex].segmentName =  modelLinkName;
        segmentIndex++;
    }

    // =====================
    // INITIALIZE LINK PAIRS
    // =====================

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
        pairInfo.ikSolver->setMaxIterations(maxIterationsIK);
        pairInfo.ikSolver->setCostTolerance(costTolerance);
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

        // TODO: Check if KinDyn is needed
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

    // Set default ik pool size
    int ikPoolSize = 1;

    // Get ikPoolSizeOption
    if (config.find("ikPoolSizeOption").isString() || config.find("ikPoolSizeOption").asString() == "auto" ) {
        yInfo() << LogPrefix << "Using " << std::thread::hardware_concurrency() << " available logical threads for ik pool";
        ikPoolSize = static_cast<int>(std::thread::hardware_concurrency());
    }
    else if(config.find("ikPoolSizeOption").isInt()) {
        ikPoolSize = config.find("ikPoolSizeOption").asInt();
    }

    pImpl->ikPool = std::unique_ptr<HumanIKWorkerPool>(new HumanIKWorkerPool(ikPoolSize,
                                                                             pImpl->linkPairs,
                                                                             pImpl->segments));

    if (!pImpl->ikPool) {
        yError() << LogPrefix << "failed to create IK worker pool";
        return false;
    }

    // ====================
    // INITIALIZE GLOBAL IK
    // ====================

    if (pImpl->useGlobalIK) {

        // Set global ik parameters
        pImpl->globalIK.setVerbosity(1);
        pImpl->globalIK.setLinearSolverName(pImpl->solverName);
        pImpl->globalIK.setMaxIterations(maxIterationsIK);
        pImpl->globalIK.setCostTolerance(1E-10);
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

            // Do not insert in the cost the rotation of the link used as base
            if (linkName == pImpl->floatingBaseFrame.model) {
                continue;
            }

            // Add ik targets and set to identity
            if (!pImpl->globalIK.addTarget(linkName, iDynTree::Transform::Identity())) {
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
    // Get the link transformations from input data
    if (!pImpl->getLinkTransformFromInputData(pImpl->linkTransformMatrices)) {
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

    auto tick = std::chrono::high_resolution_clock::now();

    // Call IK worker pool to solve
    pImpl->ikPool->runAndWait();

    auto tock = std::chrono::high_resolution_clock::now();

    yDebug() << "IK took"
             << std::chrono::duration_cast<std::chrono::milliseconds>(tock - tick).count() << "ms";


    // =================================================
    // JOIN IK SOLUTIONS AND EXPOSE DATA FOR IHUMANSTATE
    // =================================================

    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);

        // Joint link pair ik solutions using joints map from link pairs initialization
        // to solution struct for exposing data through interface
        for (auto& linkPair : pImpl->linkPairs) {
            size_t jointIndex = 0;
            for (auto& pairJoint : linkPair.consideredJointLocations) {

                // Check if it is a valid 1 DoF joint
                if (pairJoint.second == 1) {

                    // If global ik is false, use link pair joint solutions
                    if (!pImpl->useGlobalIK) {
                        pImpl->solution.jointPositions[pairJoint.first] = linkPair.jointConfigurations.getVal(jointIndex);

                        //TODO: Set the correct velocities values
                        pImpl->solution.jointVelocities[pairJoint.first] = linkPair.jointVelocities.getVal(jointIndex);
                    }
                    else {
                        pImpl->linkPairsJointConfigurationSolution.setVal(pairJoint.first, linkPair.jointConfigurations.getVal(jointIndex));
                    }

                    jointIndex++;
                }
                else {
                    yWarning() << LogPrefix << " Invalid DoFs for the joint, skipping the ik solution for this joint";
                    continue;
                }

            }
        }

        // Get base transform from the suit
        iDynTree::Transform measuredBaseTransform;
        measuredBaseTransform = pImpl->linkTransformMatrices.at(pImpl->floatingBaseFrame.model);

        // Get base velocity from the suit
        iDynTree::Vector6 measuredBaseVelocity;
        measuredBaseVelocity = pImpl->linkVelocities.at(pImpl->floatingBaseFrame.model);

        // If global ik is false, use measured pose for base frame
        if (!pImpl->useGlobalIK) {
            pImpl->solution.basePosition = {measuredBaseTransform.getPosition().getVal(0),
                                            measuredBaseTransform.getPosition().getVal(1),
                                            measuredBaseTransform.getPosition().getVal(2)};

            pImpl->solution.baseOrientation = {
                measuredBaseTransform.getRotation().asQuaternion().getVal(0),
                measuredBaseTransform.getRotation().asQuaternion().getVal(1),
                measuredBaseTransform.getRotation().asQuaternion().getVal(2),
                measuredBaseTransform.getRotation().asQuaternion().getVal(3),
            };

            pImpl->solution.baseVelocity = {
                measuredBaseVelocity.getVal(0),
                measuredBaseVelocity.getVal(1),
                measuredBaseVelocity.getVal(2),
                measuredBaseVelocity.getVal(3),
                measuredBaseVelocity.getVal(4),
                measuredBaseVelocity.getVal(5)
            };
        }
        else {

            // Set global IK initial condition
            if (!pImpl->globalIK.setFullJointsInitialCondition(&measuredBaseTransform, &pImpl->linkPairsJointConfigurationSolution)) {
                yError() << LogPrefix << "Failed to set the joint configuration for initializing the global IK";
                askToStop();
                return;
            }

            // Update ik targets based on wearable input data
            for (size_t linkIndex = 0; linkIndex < pImpl->humanModel.getNrOfLinks(); ++linkIndex) {
                std::string linkName = pImpl->humanModel.getLinkName(linkIndex);

                // Do not insert in the cost the rotation of the link used as base
                if (linkName == pImpl->floatingBaseFrame.model) {
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

                if (!pImpl->globalIK.updateTarget(linkName, pImpl->linkTransformMatrices.at(linkName))) {
                    yError() << LogPrefix << "Failed to update target for link" << linkName;
                    askToStop();
                    return;
                }
            }

            // Set link pairs ik solution as desired joint configuration for global ik
            pImpl->globalIK.setDesiredFullJointsConfiguration(pImpl->linkPairsJointConfigurationSolution, 1000);

            auto tick = std::chrono::high_resolution_clock::now();

            // TODO: Double check the logic if this fails
            if (!pImpl->globalIK.solve()) {
                    yError() << LogPrefix << "Failed to solve global IK";
            }

            auto tock = std::chrono::high_resolution_clock::now();
            yDebug() << LogPrefix << "Global IK took"
                     << std::chrono::duration_cast<std::chrono::milliseconds>(tock - tick).count() << "ms";

            // Get the optimized base from global ik solution
            iDynTree::Transform optimizedBaseTransform;

            pImpl->globalIK.getFullJointsSolution(optimizedBaseTransform, pImpl->globalJointConfigurationSolution);

            for (unsigned i = 0; i < pImpl->globalJointConfigurationSolution.size(); ++i) {
                pImpl->solution.jointPositions[i] = pImpl->globalJointConfigurationSolution.getVal(i);
            }

            pImpl->solution.basePosition = {optimizedBaseTransform.getPosition().getVal(0),
                                            optimizedBaseTransform.getPosition().getVal(1),
                                            optimizedBaseTransform.getPosition().getVal(2)};

            pImpl->solution.baseOrientation = {
                optimizedBaseTransform.getRotation().asQuaternion().getVal(0),
                optimizedBaseTransform.getRotation().asQuaternion().getVal(1),
                optimizedBaseTransform.getRotation().asQuaternion().getVal(2),
                optimizedBaseTransform.getRotation().asQuaternion().getVal(3),
            };

            // TODO: base velocity
            // TODO: joint velocities
            pImpl->solution.jointVelocities.resize(pImpl->solution.jointPositions.size(), 0);

        }
    }
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
        std::unordered_map<std::string, iDynTree::Vector6>& velocities)
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
