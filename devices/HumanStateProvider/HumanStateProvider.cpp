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
        jointPositions = {};
        jointVelocities = {};
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
    iDynTree::VectorDynSize jointConfigurationSolution;
    iDynTree::VectorDynSize jointVelocitiesSolution;

    // IK stuff
    yarp::os::Value ikPoolOption;
    std::unique_ptr<HumanIKWorkerPool> ikPool;
    SolutionIK solution;
    iDynTree::InverseKinematics ik;
    std::string solverName;

    bool getJointAnglesFromInputData(iDynTree::VectorDynSize& jointAngles);
    bool getLinkOrientationFromInputData(std::unordered_map<std::string, iDynTree::Rotation>& t);
};

// =========================
// HUMANSTATEPROVIDER DEVICE
// =========================

HumanStateProvider::HumanStateProvider()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanStateProvider::~HumanStateProvider()
{
    detachAll();
}

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

<<<<<<< refs/remotes/yeshifork/yeshi-release/HDEv2
    if (!(config.check("costTolerance") && config.find("costTolerance").isFloat64())) {
        yError() << LogPrefix << "costTolerance option not found or not valid";
=======
    if (!(config.check("ikLinearSolver") && config.find("ikLinearSolver").isString())) {
        yError() << LogPrefix << "ikLinearSolver option not found or not valid";
        return false;
    }

    if (!(config.check("ikPoolOption") && (config.find("ikPoolOption").isString() || config.find("ikPoolOption").isInt()))) {
        yError() << LogPrefix << "ikPoolOption option not found or not valid";
>>>>>>> Update config with ik worker pool
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
<<<<<<< refs/remotes/yeshifork/yeshi-release/HDEv2
    double costTolerance = config.find("costTolerance").asFloat64();
=======
    pImpl->solverName = config.find("ikLinearSolver").asString();
>>>>>>> Update config with ik worker pool
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
<<<<<<< refs/remotes/yeshifork/yeshi-release/HDEv2
    yInfo() << LogPrefix << "*** Cost Tolerance         :" << costTolerance;
=======
    yInfo() << LogPrefix << "*** IK Solver Name         :" << pImpl->solverName;
>>>>>>> Update config with ik worker pool
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

    pImpl->jointConfigurationSolution.resize(nrOfJoints);
    pImpl->jointConfigurationSolution.zero();

    pImpl->jointVelocitiesSolution.resize(nrOfJoints);
    pImpl->jointVelocitiesSolution.zero();

    // TODO: Verify this
    // Get the model link names from the attached IWear interface
    const size_t nrOfSegments = pImpl->humanModel.getNrOfLinks();
    pImpl->segments.resize(nrOfSegments);
    for (size_t linkIndex = 0; linkIndex < nrOfSegments; ++linkIndex) {
        pImpl->segments[linkIndex].velocities.resize(6);
        pImpl->segments[linkIndex].velocities.zero();

        // Get the name of the link from the model and its prefix from iWear
        pImpl->segments[linkIndex].segmentName =  pImpl->humanModel.getLinkName(linkIndex);
    }

    // Get all the possible pairs composing the model
    std::vector<std::pair<std::string, std::string>> pairNames;
    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex> > pairSegmentIndeces;

    // Get the pairNames
    createEndEffectorsPairs(pImpl->humanModel, pImpl->segments, pairNames, pairSegmentIndeces);

    pImpl->linkPairs.reserve(pairNames.size());

    for (unsigned index = 0; index < pairNames.size(); ++index) {
        LinkPairInfo pairInfo;

        pairInfo.parentFrameName = pairNames[index].first;
        pairInfo.parentFrameSegmentsIndex = pairSegmentIndeces[index].first;

        pairInfo.childFrameName = pairNames[index].second;
        pairInfo.childFrameSegmentsIndex = pairSegmentIndeces[index].second;

        // Allocate the solver
        pairInfo.ikSolver = std::unique_ptr<iDynTree::InverseKinematics>(new iDynTree::InverseKinematics());

        pairInfo.ikSolver->setVerbosity(1);

        // Get the reduced model
        iDynTree::Model pairModel;
        if (!getReducedModel(pImpl->humanModel, pairInfo.parentFrameName, pairInfo.childFrameName, pairModel)) {

            yWarning() << LogPrefix << "failed to get reduced model for the segment pair " << pairInfo.parentFrameName.c_str()
                                    << ", " << pairInfo.childFrameName.c_str();
            continue;
        }

        if (!pairInfo.ikSolver->setModel(pairModel)) {
            yWarning() << LogPrefix << "failed to configure IK solver for the segment pair" << pairInfo.parentFrameName.c_str()
                                  << ", " << pairInfo.childFrameName.c_str() <<  " Skipping pair";
            continue;
        }

        //now we have to obtain the information needed to map the IK solution
        //back to the total Dofs vector we need to output
        std::vector<std::string> solverJoints;

        solverJoints.resize(pairModel.getNrOfJoints());

        for (int i=0; i < pairModel.getNrOfJoints(); i++) {
            solverJoints[i] = pairModel.getJointName(i);
        }

        pairInfo.consideredJointLocations.reserve(solverJoints.size());
        for (std::vector<std::string>::const_iterator jointName(solverJoints.begin());
             jointName != solverJoints.end(); ++jointName) {
            iDynTree::JointIndex jointIndex = pairModel.getJointIndex(*jointName);
            if (jointIndex == iDynTree::JOINT_INVALID_INDEX) {
                yWarning("IK considered joint %s not found in the complete model", jointName->c_str());
                continue;
            }
            iDynTree::IJointConstPtr joint = pairModel.getJoint(jointIndex);
            //Save location and length of each DoFs
            pairInfo.consideredJointLocations.push_back(std::pair<size_t, size_t>(joint->getDOFsOffset(), joint->getNrOfDOFs()));
        }

        pairInfo.jointConfigurations.resize(solverJoints.size());
        pairInfo.jointConfigurations.zero();

        //same size and initialization
        pairInfo.jointVelocities = pairInfo.jointConfigurations;


        //Now configure the kinDynComputation objects
        modelLoader.loadReducedModelFromFullModel(pairModel, solverJoints);
        const iDynTree::Model &reducedModel = modelLoader.model();
        //save the indeces
        //TODO: check if link or frame
        pairInfo.parentFrameModelIndex = reducedModel.getFrameIndex(pairInfo.parentFrameName);
        pairInfo.childFrameModelIndex = reducedModel.getFrameIndex(pairInfo.childFrameName);

        pairInfo.kinDynComputations = std::unique_ptr<iDynTree::KinDynComputations>(new iDynTree::KinDynComputations());

        pairInfo.kinDynComputations->loadRobotModel(modelLoader.model());
        pairInfo.parentJacobian.resize(6, modelLoader.model().getNrOfDOFs());
        pairInfo.parentJacobian.zero();
        pairInfo.childJacobian = pairInfo.relativeJacobian = pairInfo.parentJacobian;

        //overwriting the default contructed object
        pairInfo.jacobianDecomposition = Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(pairInfo.relativeJacobian.rows(), pairInfo.relativeJacobian.cols());

        pImpl->linkPairs.push_back(std::move(pairInfo));
    }


    // =========================
    // INITIALIZE IK WORKER POOL
    // =========================

    yInfo() << LogPrefix << "Available logical threads are %u : " << std::thread::hardware_concurrency();
    int ikPoolSize = -1;

    // TODO: Verify this option
    // Get ikPoolOption
    pImpl->ikPoolOption = rf.check("ikPoolOption", yarp::os::Value(1), "Checking size of pool");

    if (pImpl->ikPoolOption.isString()) {
        if (pImpl->ikPoolOption.asString() != "auto") {
            yWarning() << LogPrefix << "IK pool option not recognized. Not using IK worker pool";
        } else {
            ikPoolSize = static_cast<int>(std::thread::hardware_concurrency());
        }

    } else {
        ikPoolSize = pImpl->ikPoolOption.asInt();
    }

    pImpl->ikPool = std::unique_ptr<HumanIKWorkerPool>(new HumanIKWorkerPool(ikPoolSize,
                                                                             pImpl->linkPairs,
                                                                             pImpl->segments));

    if (!pImpl->ikPool) {
        yError() << LogPrefix << "failed to create IK worker pool";
        return false;
    }

    // =============================
    // INITIALIZE INVERSE KINEMATICS
    // =============================

    {
        using namespace iDynTree;

        pImpl->ik.setVerbosity(1); // TODO
        pImpl->ik.setMaxIterations(maxIterationsIK);
        //        pImpl->ik.setRotationParametrization(InverseKinematicsRotationParametrizationQuaternion);
        pImpl->ik.setRotationParametrization(InverseKinematicsRotationParametrizationRollPitchYaw);
        pImpl->ik.setCostTolerance(costTolerance);

        if (!pImpl->ik.setModel(pImpl->humanModel)) {
            yError() << LogPrefix << "IK: failed to load the model";
            return false;
        }

        if (!pImpl->ik.setFloatingBaseOnFrameNamed(pImpl->floatingBaseFrame.model)) {
            yError() << LogPrefix << "Failed to set the IK floating base frame on link"
                     << pImpl->floatingBaseFrame.model;
            return false;
        }

        // Use targets only as targets (in the cost)
        pImpl->ik.setDefaultTargetResolutionMode(InverseKinematicsTreatTargetAsConstraintNone);
        // pImpl->ik.setDefaultTargetResolutionMode(
        //     InverseKinematicsTreatTargetAsConstraintRotationOnly);
    }

    return true;
}


// TODO: Verify this method
static void createEndEffectorsPairs(const iDynTree::Model& model,
                                    std::vector<SegmentInfo>& humanSegments,
                                    std::vector<std::pair<std::string, std::string> > &framePairs,
                                    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex> > &framePairIndeces)
{
    //for each element in human segments
    //extract it from the vector (to avoid duplications)
    //Look for it in the model and get neighbours.

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
    std::vector< std::string> consideredJoints;
    iDynTree::Traversal traversal;
    iDynTree::LinkIndex parentLinkIdx;
    iDynTree::IJointConstPtr joint;
    iDynTree::ModelLoader loader;

    // TODO: Check if autoSelectJointsFromTraversal and removeUnsupportedJoints methods
    // are needed


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

        consideredJoints.insert(consideredJoints.begin(), modelInput.getJointName(joint->getIndex()));

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

    // Remove unsupported joints
    // TODO: Check if this is needed
    for(int i=0; i < loader.model().getNrOfJoints(); ++i){
        if(loader.model().getJoint(i)->getNrOfDOFs() == 1){
            consideredJoints.push_back(loader.model().getJointName(i));
        }
        else std::cerr << "Joint " << loader.model().getJointName(i) << " ignored (" << loader.model().getJoint(i)->getNrOfDOFs() << " DOF)" << std::endl;
    }

    loader.loadReducedModelFromFullModel(loader.model(), consideredJoints);

    modelOutput = loader.model();

    return true;
}



bool HumanStateProvider::close()
{
    return true;
}

void HumanStateProvider::run()
{
    if (pImpl->firstRun) {
        pImpl->firstRun = false;
        // Set the link orientations as IK targets
        for (size_t linkIndex = 0; linkIndex < pImpl->humanModel.getNrOfLinks(); ++linkIndex) {
            std::string linkName = pImpl->humanModel.getLinkName(linkIndex);

            // Do not insert in the cost the rotation of the link used as base
            if (linkName == pImpl->floatingBaseFrame.model) {
                continue;
            }

            iDynTree::Rotation dummyRotation;
            if (!pImpl->ik.addRotationTarget(linkName, dummyRotation)) {
                yError() << LogPrefix << "Failed to add rotation target for link" << linkName;
                askToStop();
                return;
            }
        }
    }

    // Process incoming data
    for (unsigned linkIndex = 0; linkIndex < pImpl->humanModel.getNrOfLinks(); linkIndex++) {
        std::string linkName = pImpl->humanModel.getLinkName(linkIndex);

        Vector3 position;
        Quaternion orientation;

        if (!pImpl->wearableStorage.baseLinkSensor->getLinkPose(position, orientation)) {
            yError() << LogPrefix << "Failed to get position and orientation of the base"
                     << "from the input data";
            askToStop();
            return;
        }
        else {
            if (pImpl->wearableStorage.linkSensorsMap.find(linkName)
                == pImpl->wearableStorage.linkSensorsMap.end()) {
                yError() << LogPrefix << "Failed to find sensor for link" << linkName;
                askToStop();
                return;
            }

            auto& linkSensor = pImpl->wearableStorage.linkSensorsMap.at(linkName);

            // Get the position and orientation of the link
            if (!linkSensor->getLinkPose(position, orientation)) {
                yError() << LogPrefix << "Failed to get position and orientation of link "
                         << linkName << "from the input data";
                askToStop();
                return;
            }
        }

        iDynTree::Position pos(position.at(0),
                               position.at(1),
                               position.at(2));

        iDynTree::Vector4 quat;
        quat.setVal(0,orientation[0]);
        quat.setVal(1,orientation[1]);
        quat.setVal(2,orientation[2]);
        quat.setVal(3,orientation[3]);

        iDynTree::Rotation rot = iDynTree::Rotation::RotationFromQuaternion(quat);

        // Fill the link data into segements
        SegmentInfo& segmentInfo = pImpl->segments.at(linkIndex);
        segmentInfo.poseWRTWorld = iDynTree::Transform(rot, pos);

        // TODO: Change the velocites from zero to data from the suit
        for (unsigned i = 0; i < 6; ++i) {
            segmentInfo.velocities(i) = 0;
        }
    }

    // Call IK worker pool to solve
    pImpl->ikPool->runAndWait();

    // Join ik solutions saved in jointsConfiguration and jointsVelocitu of various pairInfo variables
    for (auto &pair : pImpl->linkPairs) {
        size_t jointIndex = 0;
        for (auto &pairJoint : pair.consideredJointLocations) {

            // Initialize joint quantities with zero values and size
            Eigen::Map<Eigen::VectorXd> jointPositions(pImpl->jointConfigurationSolution.data(), pImpl->jointConfigurationSolution.size());
            jointPositions.segment(pairJoint.first, pairJoint.second) = iDynTree::toEigen(pair.jointConfigurations).segment(jointIndex, pairJoint.second);

            Eigen::Map<Eigen::VectorXd> jointVelocities(pImpl->jointVelocitiesSolution.data(),pImpl->jointVelocitiesSolution.size());
            jointVelocities.segment(pairJoint.first, pairJoint.second) = iDynTree::toEigen(pair.jointVelocities).segment(jointIndex, pairJoint.second);

            jointIndex += pairJoint.second;
        }
    }


    // =================
    // INITIALIZE THE IK
    // =================
    //
    // The Xsens segment (link) measurements are used in the IK cost for calculating
    // the joint angles of our urdf model.
    //
    // The IK can be initialized either with the previous solution or with the joint
    // angles provided by Xsens.

    // Get the base transform
    // ======================

    Vector3 position;
    Quaternion orientation;

    // Get the position and orientation
    if (!pImpl->wearableStorage.baseLinkSensor->getLinkPose(position, orientation)) {
        yError() << LogPrefix << "Failed to get position and orientation of the base"
                 << "from the input data";
        askToStop();
        return;
    }

    // Convert them to iDynTree objects since we're going to use them for the IK
    iDynTree::Position positioniDynTree(position[0], position[1], position[2]);
    iDynTree::Rotation rotationiDynTree;
    rotationiDynTree.fromQuaternion({orientation.data(), 4});

    iDynTree::Transform baseTransform(std::move(rotationiDynTree), std::move(positioniDynTree));

    // Initialize the IK problem with joint angles
    // ===========================================

    if (pImpl->useXsensJointsAngles) {
        // Use joint angles from the input data
        if (!pImpl->getJointAnglesFromInputData(pImpl->jointAngles)) {
            yError() << LogPrefix << "Failed to get joint angles from input data";
            askToStop();
            return;
        }
    }
    else {
        // Use joint angles from the previous solution
        pImpl->jointAngles = pImpl->jointConfigurationSolution;
    }

    // Initialize the IK
    if (!pImpl->ik.setFullJointsInitialCondition(&baseTransform, &pImpl->jointAngles)) {
        yError() << LogPrefix << "Failed to set the joint configuration for initializing the IK";
        askToStop();
        return;
    }

    // TODO: postural. This can be moved after the IK finds a solution and it could
    //       substitute setting the initial condition.
    // if (!pImpl->ik.setDesiredFullJointsConfiguration(pImpl->jointAngles)) {
    //     yError() << LogPrefix << "Failed to set the postural configuration of the IK";
    //     askToStop();
    //     return;
    // }

    // ======================
    // PREPARE THE IK PROBLEM
    // ======================

    // Get the orientation of the links from the input data
    if (!pImpl->getLinkOrientationFromInputData(pImpl->linkRotationMatrices)) {
        yError() << LogPrefix << "Failed to get link orientations from input data";
        askToStop();
        return;
    }

    // Set the link orientations as IK targets
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

        if (pImpl->linkRotationMatrices.find(linkName) == pImpl->linkRotationMatrices.end()) {
            yError() << LogPrefix << "Failed to find rotation matrix for link" << linkName;
            askToStop();
            return;
        }

        if (!pImpl->ik.updateRotationTarget(linkName, pImpl->linkRotationMatrices.at(linkName))) {
            yError() << LogPrefix << "Failed to update rotation target for link" << linkName;
            askToStop();
            return;
        }
    }

    // ====================
    // SOLVE THE IK PROBLEM
    // ====================

    // TODO: add benchmarking capabilities. Using PeriodicThread utilities?
    auto tick = std::chrono::high_resolution_clock::now();

    if (!pImpl->ik.solve()) {
        if (pImpl->allowIKFailures) {
            yWarning() << LogPrefix << "IK failed, keeping the previous solution";
            return;
        }
        else {
            yError() << LogPrefix << "Failed to solve IK";
            askToStop();
            return;
        }
    }

    auto tock = std::chrono::high_resolution_clock::now();
    yDebug() << LogPrefix << "IK took"
             << std::chrono::duration_cast<std::chrono::milliseconds>(tock - tick).count() << "ms";

    // ===========================
    // EXPOSE DATA FOR IHUMANSTATE
    // ===========================

    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);

        // Move the solution to the struct used from exposing the data through the interface
        iDynTree::Transform baseTransformSolution;
        pImpl->ik.getFullJointsSolution(baseTransformSolution, pImpl->jointConfigurationSolution);

        for (unsigned i = 0; i < pImpl->jointConfigurationSolution.size(); ++i) {
            pImpl->solution.jointPositions[i] = pImpl->jointConfigurationSolution.getVal(i);
        }

        pImpl->solution.basePosition = {baseTransformSolution.getPosition().getVal(0),
                                        baseTransformSolution.getPosition().getVal(1),
                                        baseTransformSolution.getPosition().getVal(2)};

        pImpl->solution.baseOrientation = {
            baseTransformSolution.getRotation().asQuaternion().getVal(0),
            baseTransformSolution.getRotation().asQuaternion().getVal(1),
            baseTransformSolution.getRotation().asQuaternion().getVal(2),
            baseTransformSolution.getRotation().asQuaternion().getVal(3),
        };

        // TODO: base velocity
        // TODO: joint velocities
        pImpl->solution.jointVelocities.resize(pImpl->solution.jointPositions.size(), 0);
    }
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
            yWarning() << LogPrefix << "Failed to find" << modelLinkName
                       << "entry in the configuration map. Skipping this link.";
            continue;
        }

        // Get the name of the sensor associated to the link
        std::string wearableLinkName =
            pImpl->wearableStorage.modelToWearable_LinkName.at(modelLinkName);

        // Try to get the sensor
        auto sensor = pImpl->iWear->getVirtualLinkKinSensor(wearableLinkName);
        if (!sensor) {
            yError() << LogPrefix << "Failed to find sensor associated to link" << wearableLinkName
                     << "from the IWear interface";
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

bool HumanStateProvider::detach()
{
    askToStop();

    {
        std::lock_guard<std::mutex>(pImpl->mutex);
        pImpl->solution.clear();
    }

    pImpl->iWear = nullptr;
    return true;
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

bool HumanStateProvider::detachAll()
{
    return detach();
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
