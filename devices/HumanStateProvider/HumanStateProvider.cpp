/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStateProvider.h"
#include "IKWorkerPool.h"
#include "InverseVelocityKinematics/InverseVelocityKinematics.hpp"

#include <Wearable/IWear/IWear.h>
#include <iDynTree/InverseKinematics.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Traversal.h>


#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <stack>
#include <string>
#include <thread>
#include <unordered_map>

#include "Utils.hpp"

/*!
 * @brief analyze model and list of segments to create all possible segment pairs
 *
 * @param[in] model the full model
 * @param[in] humanSegments list of segments on which look for the possible pairs
 * @param[out] framePairs resulting list of all possible pairs. First element is parent, second is
 * child
 * @param[out] framePairIndeces indeces in the humanSegments list of the pairs in framePairs
 */
static void createEndEffectorsPairs(
    const iDynTree::Model& model,
    std::vector<SegmentInfo>& humanSegments,
    std::vector<std::pair<std::string, std::string>>& framePairs,
    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>>& framePairIndeces);

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

using InverseVelocityKinematicsSolverName = std::string;

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

    std::array<double, 3> CoMPosition;
    std::array<double, 3> CoMVelocity;
    std::array<double, 3> CoMBiasAcceleration;

    void clear()
    {
        jointPositions.clear();
        jointVelocities.clear();
    }
};

enum SolverIK
{
    global,
    pairwised,
    integrationbased
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

    // Accelerometer sensor wearable data variables
    std::unordered_map<ModelLinkName, WearableLinkName> modelToWearable_AccelerometerParentLinkName;
    std::unordered_map<WearableLinkName, SensorPtr<const sensor::IFreeBodyAccelerationSensor>> accelerometerSensorsMap;

    // Orientation sensor wearable data variables
    std::unordered_map<ModelLinkName, WearableLinkName> modelToWearable_SensorOrientationParentLinkName;
    std::unordered_map<WearableLinkName, SensorPtr<const sensor::IOrientationSensor>> orientationSensorsMap;
};

struct HumanSensorData
{
    // Accelerometers
    std::string accelerometerSensorMeasurementsOption;
    std::vector<std::string> accelerometerSensorNames;
    std::vector<std::array<double, 6>> accelerometerSensorMeasurements;
};

class HumanStateProvider::impl
{
public:
    // Attached interface
    wearable::IWear* iWear = nullptr;

    bool allowIKFailures;
    bool useXsensJointsAngles;

    float period;
    mutable std::mutex mutex;

    // Wearable variables
    WearableStorage wearableStorage;

    // Model variables
    iDynTree::Model humanModel;
    iDynTree::SensorsList humanSensors;
    FloatingBaseName floatingBaseFrame;

    std::vector<SegmentInfo> segments;
    std::vector<LinkPairInfo> linkPairs;

    // Link wearable data buffers
    std::unordered_map<std::string, iDynTree::Transform> linkTransformMatrices;
    std::unordered_map<std::string, iDynTree::Twist> linkVelocities;
    std::unordered_map<std::string, iDynTree::SpatialAcc> linkAccelerations;

    // Accelerometer fbAcc and Orientation wearable  data buffers
    std::unordered_map<std::string, iDynTree::AngAcceleration> fbAccelerationMatrices;
    std::unordered_map<std::string, iDynTree::Rotation> sensorOrientationMatrices;

    iDynTree::VectorDynSize jointConfigurationSolution;
    iDynTree::VectorDynSize jointVelocitiesSolution;
    iDynTree::Transform baseTransformSolution;
    iDynTree::Twist baseVelocitySolution;

    iDynTree::Vector3 integralOrientationError;
    iDynTree::Vector3 integralLinearVelocityError;

    std::unordered_map<std::string, iDynTreeHelper::Rotation::rotationDistance>
        linkErrorOrientations;
    std::unordered_map<std::string, iDynTree::Vector3> linkErrorAngularVelocities;

    // Sensor variables
    bool useFBAccelerationFromWearableData;
    HumanSensorData humanSensorData;

    std::array<double, 3> CoMProperAcceleration;

    // IK parameters
    int ikPoolSize{1};
    int maxIterationsIK;
    double costTolerance;
    std::string linearSolverName;
    yarp::os::Value ikPoolOption;
    std::unique_ptr<IKWorkerPool> ikPool;
    SolutionIK solution;
    InverseVelocityKinematicsSolverName inverseVelocityKinematicsSolver;

    double posTargetWeight;
    double rotTargetWeight;
    double linVelTargetWeight;
    double angVelTargetWeight;
    double costRegularization;

    double integrationBasedIKMeasuredLinearVelocityGain;
    double integrationBasedIKMeasuredAngularVelocityGain;
    double integrationBasedIKLinearCorrectionGain;
    double integrationBasedIKAngularCorrectionGain;
    double integrationBasedIKIntegralLinearCorrectionGain;
    double integrationBasedIKIntegralAngularCorrectionGain;
    double integrationBasedJointVelocityLimit;

    SolverIK ikSolver;

    bool useDirectBaseMeasurement;

    iDynTree::InverseKinematics globalIK;
    InverseVelocityKinematics inverseVelocityKinematics;
    iDynTreeHelper::State::integrator stateIntegrator;

    // clock
    double lastTime{-1.0};

    // kinDynComputation
    std::unique_ptr<iDynTree::KinDynComputations> kinDynComputations;
    iDynTree::Vector3 worldGravity;

    // get input data
    bool getJointAnglesFromInputData(iDynTree::VectorDynSize& jointAngles);
    bool getLinkQuantitiesFromInputData(std::unordered_map<std::string, iDynTree::Transform>& t,
                                        std::unordered_map<std::string, iDynTree::SpatialAcc>& linkAcc);
    bool getLinkVelocityFromInputData(std::unordered_map<std::string, iDynTree::Twist>& t); //TODO: This should be removed and link velocity should be retrieved inside getLinkQuantitiesFromInputData
    bool getfbAccelerationFromInputData(std::unordered_map<std::string, iDynTree::AngAcceleration>& acc);
    bool getOrientationFromInputData(std::unordered_map<std::string, iDynTree::Rotation>& ori);

    // solver initialization and update
    bool initializePairwisedInverseKinematicsSolver();
    bool initializeGlobalInverseKinematicsSolver();
    bool initializeIntegrationBasedInverseKinematicsSolver();
    bool solvePairwisedInverseKinematicsSolver();
    bool solveGlobalInverseKinematicsSolver();
    bool solveIntegrationBasedInverseKinematics();

    // optimization targets
    bool updateInverseKinematicTargets();
    bool addInverseKinematicTargets();

    bool updateInverseVelocityKinematicTargets();
    bool addInverseVelocityKinematicsTargets();

    bool computeLinksOrientationErrors(
        std::unordered_map<std::string, iDynTree::Transform> linkDesiredOrientations,
        iDynTree::VectorDynSize jointConfigurations,
        iDynTree::Transform floatingBasePose,
        std::unordered_map<std::string, iDynTreeHelper::Rotation::rotationDistance>&
            linkErrorOrientations);
    bool computeLinksAngularVelocityErrors(
        std::unordered_map<std::string, iDynTree::Twist> linkDesiredVelocities,
        iDynTree::VectorDynSize jointConfigurations,
        iDynTree::Transform floatingBasePose,
        iDynTree::VectorDynSize jointVelocities,
        iDynTree::Twist baseVelocity,
        std::unordered_map<std::string, iDynTree::Vector3>& linkAngularVelocityError);
};

// =========================
// HUMANSTATEPROVIDER DEVICE
// =========================

HumanStateProvider::HumanStateProvider()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanStateProvider::~HumanStateProvider() {}

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

    if (!(config.check("ikSolver") && config.find("ikSolver").isString())) {
        yError() << LogPrefix << "ikSolver option not found or not valid";
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

    // Parse MODEL_TO_DATA_LINK_NAMES
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

    // Set gravity
    pImpl->worldGravity.zero();
    pImpl->worldGravity(2) = -9.81;

    // Get accelerometer sensor measurements option parameters
    // Default option set to "none" for zero measurement values
    pImpl->humanSensorData.accelerometerSensorMeasurementsOption = config.check("accelerometerSensorMeasurementsOption",yarp::os::Value("none")).asString();


    if (pImpl->humanSensorData.accelerometerSensorMeasurementsOption == "proper") {
        yInfo() << LogPrefix << "Configuring accelerometers to user freebody accelerometer values from wearable input data";
        pImpl->useFBAccelerationFromWearableData = true;
    }
    else if ( pImpl->humanSensorData.accelerometerSensorMeasurementsOption == "gravity") {
        yInfo() << LogPrefix << "Configuring accelerometers to use compensate only for gravity acceleration";
        pImpl->useFBAccelerationFromWearableData = true;
    }
    else {

        yInfo() << LogPrefix << "Configuring accelerometers based on the given urdf model";
        yInfo() << LogPrefix << "Using default values of Zeros for the accelerometers";

        pImpl->useFBAccelerationFromWearableData = false;
    }

    // If proper acceleration from wearable data is needed, user has to pass MODEL_TO_DATA_ACCELEROMETER_PARENT_LINK_NAMES
    // TODO: This parsing can be improved by a single function for both links and accelerometers
    if (pImpl->useFBAccelerationFromWearableData) {

        // Parse MODEL_TO_DATA_ACCELEROMETER_PARENT_LINK_NAMES
        yarp::os::Bottle& accelerometersGroup = config.findGroup("MODEL_TO_DATA_ACCELEROMETER_PARENT_LINK_NAMES");
        if (accelerometersGroup.isNull()) {
            yError() << LogPrefix << "Failed to find group MODEL_TO_DATA_ACCELEROMETER_PARENT_LINK_NAMES";
            return false;
        }

        for (size_t a = 1; a < accelerometersGroup.size(); ++a) {

            if (!(accelerometersGroup.get(a).isList() && accelerometersGroup.get(a).asList()->size() == 2)) {
                yError() << LogPrefix
                         << "Childs of MODEL_TO_DATA_ACCELEROMETER_PARENT_LINK_NAMES must be lists of two elements";
                return false;
            }

            yarp::os::Bottle* list = accelerometersGroup.get(a).asList();
            std::string key = list->get(0).asString();
            yarp::os::Bottle* listContent = list->get(1).asList();

            if (!((listContent->size() == 2) && (listContent->get(0).isString())
                  && (listContent->get(1).isString()))) {
                yError() << LogPrefix << "Accelerometer parent link list must have two strings";
                return false;
            }

        }

        // Parse accelerometersGroup parameters
        for (size_t a = 1; a < accelerometersGroup.size(); ++a) {
            yarp::os::Bottle* listContent = accelerometersGroup.get(a).asList()->get(1).asList();

            std::string modelParentLinkName = listContent->get(0).asString();
            std::string wearableParentLinkName =listContent->get(1).asString();

            yInfo() << LogPrefix << "Read accelerometer parent link map: " << modelParentLinkName << "==>" << wearableParentLinkName;
            pImpl->wearableStorage.modelToWearable_AccelerometerParentLinkName[modelParentLinkName] = wearableParentLinkName;
        }

        // Parse MODEL_TO_DATA_ORIENTATION_PARENT_LINK_NAMES
        yarp::os::Bottle& orientationsGroup = config.findGroup("MODEL_TO_DATA_ORIENTATION_PARENT_LINK_NAMES");
        if (orientationsGroup.isNull()) {
            yError() << LogPrefix << "Failed to find group MODEL_TO_DATA_ORIENTATION_PARENT_LINK_NAMES";
            return false;
        }

        for (size_t o = 1; o < orientationsGroup.size(); ++o) {

            if (!(orientationsGroup.get(o).isList() && orientationsGroup.get(o).asList()->size() == 2)) {
                yError() << LogPrefix
                         << "Childs of MODEL_TO_DATA_ORIENTATION_PARENT_LINK_NAMES must be lists of two elements";
                return false;
            }

            yarp::os::Bottle* list = orientationsGroup.get(o).asList();
            std::string key = list->get(0).asString();
            yarp::os::Bottle* listContent = list->get(1).asList();

            if (!((listContent->size() == 2) && (listContent->get(0).isString())
                  && (listContent->get(1).isString()))) {
                yError() << LogPrefix << "Orientation parent link list must have two strings";
                return false;
            }

        }

        // Parse orientationsGroup parameters
        for (size_t o = 1; o < orientationsGroup.size(); ++o) {
            yarp::os::Bottle* listContent = orientationsGroup.get(o).asList()->get(1).asList();

            std::string modelParentLinkName = listContent->get(0).asString();
            std::string wearableParentLinkName =listContent->get(1).asString();

            yInfo() << LogPrefix << "Read orientation parent link map: " << modelParentLinkName << "==>" << wearableParentLinkName;
            pImpl->wearableStorage.modelToWearable_SensorOrientationParentLinkName[modelParentLinkName] = wearableParentLinkName;
        }

        // Check if the size of acclerometers and orientations match from the config file
        if (pImpl->wearableStorage.modelToWearable_AccelerometerParentLinkName.size() != pImpl->wearableStorage.modelToWearable_SensorOrientationParentLinkName.size()) {
            yError() << LogPrefix << "Mismatch in the number of the accelerometers and orientation sensors Model to Wearable parameters. They should be the same size.";
            return false;
        }

        // Check if the acclerometers and orientations parent links are similar from the config file
        for (auto& accelerometerParentLinkMapElement : pImpl->wearableStorage.modelToWearable_AccelerometerParentLinkName) {

            std::string accelerometerParentLinkName = accelerometerParentLinkMapElement.first;


            if (pImpl->wearableStorage.modelToWearable_SensorOrientationParentLinkName.find(accelerometerParentLinkName) ==
                pImpl->wearableStorage.modelToWearable_SensorOrientationParentLinkName.end()) {

                yError() << LogPrefix << "Mismatch in parent link names between MODEL_TO_DATA_ACCELEROMETER_PARENT_LINK_NAMES and MODEL_TO_DATA_ORIENTATION_PARENT_LINK_NAMES."
                                         "They should contain the same parent link names.";
                return false;

            }
        }

    }

    // =======================================
    // PARSE THE GENERAL CONFIGURATION OPTIONS
    // =======================================

    std::string solverName = config.find("ikSolver").asString();
    if (solverName == "global")
        pImpl->ikSolver = SolverIK::global;
    else if (solverName == "pairwised")
        pImpl->ikSolver = SolverIK::pairwised;
    else if (solverName == "integrationbased")
        pImpl->ikSolver = SolverIK::integrationbased;
    else {
        yError() << LogPrefix << "ikSolver " << solverName << " not found";
        return false;
    }

    yarp::os::Bottle* floatingBaseFrameList = config.find("floatingBaseFrame").asList();
    pImpl->useXsensJointsAngles = config.find("useXsensJointsAngles").asBool();
    const std::string urdfFileName = config.find("urdf").asString();
    pImpl->floatingBaseFrame.model = floatingBaseFrameList->get(0).asString();
    pImpl->floatingBaseFrame.wearable = floatingBaseFrameList->get(1).asString();
    pImpl->period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();

    setPeriod(pImpl->period);

    // Parse linksGroup parameters
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

    if (pImpl->useXsensJointsAngles) {
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

            yInfo() << LogPrefix << "Read joint map:" << modelJointName << "==> ("
                    << wearableJointName << "," << wearableJointComponent << ")";
            pImpl->wearableStorage.modelToWearable_JointInfo[modelJointName] = {
                wearableJointName, wearableJointComponent};
        }
    }

    if (pImpl->ikSolver == SolverIK::pairwised || pImpl->ikSolver == SolverIK::global) {
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
        pImpl->linearSolverName = config.find("ikLinearSolver").asString();
        pImpl->posTargetWeight = config.find("posTargetWeight").asFloat64();
        pImpl->rotTargetWeight = config.find("rotTargetWeight").asFloat64();
        pImpl->costRegularization = config.find("costRegularization").asDouble();
    }

    if (pImpl->ikSolver == SolverIK::global || pImpl->ikSolver == SolverIK::integrationbased) {
        if (!(config.check("useDirectBaseMeasurement")
              && config.find("useDirectBaseMeasurement").isBool())) {
            yError() << LogPrefix << "useDirectBaseMeasurement option not found or not valid";
            return false;
        }
        if (!(config.check("linVelTargetWeight")
              && config.find("linVelTargetWeight").isFloat64())) {
            yError() << LogPrefix << "linVelTargetWeight option not found or not valid";
            return false;
        }

        if (!(config.check("angVelTargetWeight")
              && config.find("angVelTargetWeight").isFloat64())) {
            yError() << LogPrefix << "angVelTargetWeight option not found or not valid";
            return false;
        }

        if (config.check("inverseVelocityKinematicsSolver")
            && config.find("inverseVelocityKinematicsSolver").isString()) {
            pImpl->inverseVelocityKinematicsSolver =
                config.find("inverseVelocityKinematicsSolver").asString();
        }
        else {
            pImpl->inverseVelocityKinematicsSolver = "moorePenrose";
            yInfo() << LogPrefix << "Using default inverse velocity kinematics solver";
        }

        pImpl->useDirectBaseMeasurement = config.find("useDirectBaseMeasurement").asBool();
        pImpl->linVelTargetWeight = config.find("linVelTargetWeight").asFloat64();
        pImpl->angVelTargetWeight = config.find("angVelTargetWeight").asFloat64();
        pImpl->costRegularization = config.find("costRegularization").asDouble();
    }

    if (pImpl->ikSolver == SolverIK::pairwised) {
        if (!(config.check("ikPoolSizeOption")
              && (config.find("ikPoolSizeOption").isString()
                  || config.find("ikPoolSizeOption").isInt()))) {
            yError() << LogPrefix << "ikPoolOption option not found or not valid";
            return false;
        }

        // Get ikPoolSizeOption
        if (config.find("ikPoolSizeOption").isString()
            && config.find("ikPoolSizeOption").asString() == "auto") {
            yInfo() << LogPrefix << "Using " << std::thread::hardware_concurrency()
                    << " available logical threads for ik pool";
            pImpl->ikPoolSize = static_cast<int>(std::thread::hardware_concurrency());
        }
        else if (config.find("ikPoolSizeOption").isInt()) {
            pImpl->ikPoolSize = config.find("ikPoolSizeOption").asInt();
        }

        // The pairwised IK will always use the measured base pose and velocity for the base link
        if (config.check("useDirectBaseMeasurement")
            && config.find("useDirectBaseMeasurement").isBool()
            && !config.find("useDirectBaseMeasurement").asBool()) {
            yWarning() << LogPrefix
                       << "useDirectBaseMeasurement is required from Pair-Wised IK. Assuming its "
                          "value to be true";
        }
        pImpl->useDirectBaseMeasurement = true;
    }

    if (pImpl->ikSolver == SolverIK::integrationbased) {

        if (!(config.check("integrationBasedIKMeasuredVelocityGainLinRot")
              && config.find("integrationBasedIKMeasuredVelocityGainLinRot").isList()
              && config.find("integrationBasedIKMeasuredVelocityGainLinRot").asList()->size()
                     == 2)) {
            yError()
                << LogPrefix
                << "integrationBasedIKMeasuredVelocityGainLinRot option not found or not valid";
            return false;
        }

        if (!(config.check("integrationBasedIKCorrectionGainsLinRot")
              && config.find("integrationBasedIKCorrectionGainsLinRot").isList()
              && config.find("integrationBasedIKCorrectionGainsLinRot").asList()->size() == 2)) {
            yError() << LogPrefix
                     << "integrationBasedIKCorrectionGainsLinRot option not found or not valid";
            return false;
        }

        if (!(config.check("integrationBasedIKIntegralCorrectionGainsLinRot")
              && config.find("integrationBasedIKIntegralCorrectionGainsLinRot").isList()
              && config.find("integrationBasedIKIntegralCorrectionGainsLinRot").asList()->size()
                     == 2)) {
            yError()
                << LogPrefix
                << "integrationBasedIKIntegralCorrectionGainsLinRot option not found or not valid";
            return false;
        }

        if (config.check("integrationBasedJointVelocityLimit")
            && config.find("integrationBasedJointVelocityLimit").isDouble()) {
            pImpl->integrationBasedJointVelocityLimit =
                config.find("integrationBasedJointVelocityLimit").asDouble();
        }
        else {
            pImpl->integrationBasedJointVelocityLimit = -1.0;
        }

        yarp::os::Bottle* integrationBasedIKMeasuredVelocityGainLinRot =
            config.find("integrationBasedIKMeasuredVelocityGainLinRot").asList();
        yarp::os::Bottle* integrationBasedIKCorrectionGainsLinRot =
            config.find("integrationBasedIKCorrectionGainsLinRot").asList();
        yarp::os::Bottle* integrationBasedIKIntegralCorrectionGainsLinRot =
            config.find("integrationBasedIKIntegralCorrectionGainsLinRot").asList();
        pImpl->integrationBasedIKMeasuredLinearVelocityGain =
            integrationBasedIKMeasuredVelocityGainLinRot->get(0).asFloat64();
        pImpl->integrationBasedIKMeasuredAngularVelocityGain =
            integrationBasedIKMeasuredVelocityGainLinRot->get(0).asFloat64();
        pImpl->integrationBasedIKLinearCorrectionGain =
            integrationBasedIKCorrectionGainsLinRot->get(0).asFloat64();
        pImpl->integrationBasedIKAngularCorrectionGain =
            integrationBasedIKCorrectionGainsLinRot->get(1).asFloat64();
        pImpl->integrationBasedIKIntegralLinearCorrectionGain =
            integrationBasedIKIntegralCorrectionGainsLinRot->get(0).asFloat64();
        pImpl->integrationBasedIKIntegralAngularCorrectionGain =
            integrationBasedIKIntegralCorrectionGainsLinRot->get(1).asFloat64();
    }

    // ===================================
    // PRINT CURRENT CONFIGURATION OPTIONS
    // ===================================

    yInfo() << LogPrefix << "*** ===========================================";
    yInfo() << LogPrefix << "*** Period                                    :" << pImpl->period;
    yInfo() << LogPrefix << "*** Urdf file name                            :" << urdfFileName;
    yInfo() << LogPrefix << "*** Accelerometer sensor measurements option  :" << pImpl->humanSensorData.accelerometerSensorMeasurementsOption;
    yInfo() << LogPrefix << "*** Ik solver                                 :" << solverName;
    yInfo() << LogPrefix << "*** Use Xsens joint angles                    :" << pImpl->useXsensJointsAngles;
    yInfo() << LogPrefix << "*** Use Directly base measurement             :" << pImpl->useDirectBaseMeasurement;
    if (pImpl->ikSolver == SolverIK::pairwised || pImpl->ikSolver == SolverIK::global) {
        yInfo() << LogPrefix << "*** Allow IK failures                 :" << pImpl->allowIKFailures;
        yInfo() << LogPrefix << "*** Max IK iterations                 :" << pImpl->maxIterationsIK;
        yInfo() << LogPrefix << "*** Cost Tolerance                    :" << pImpl->costTolerance;
        yInfo() << LogPrefix
                << "*** IK Solver Name                    :" << pImpl->linearSolverName;
        yInfo() << LogPrefix << "*** Position target weight            :" << pImpl->posTargetWeight;
        yInfo() << LogPrefix << "*** Rotation target weight            :" << pImpl->rotTargetWeight;
        yInfo() << LogPrefix
                << "*** Cost regularization              :" << pImpl->costRegularization;
        yInfo() << LogPrefix << "*** Size of thread pool               :" << pImpl->ikPoolSize;
    }
    if (pImpl->ikSolver == SolverIK::integrationbased) {
        yInfo() << LogPrefix << "*** Measured Linear velocity gain     :"
                << pImpl->integrationBasedIKMeasuredLinearVelocityGain;
        yInfo() << LogPrefix << "*** Measured Angular velocity gain    :"
                << pImpl->integrationBasedIKMeasuredAngularVelocityGain;
        yInfo() << LogPrefix << "*** Linear correction gain            :"
                << pImpl->integrationBasedIKLinearCorrectionGain;
        yInfo() << LogPrefix << "*** Angular correction gain           :"
                << pImpl->integrationBasedIKAngularCorrectionGain;
        yInfo() << LogPrefix << "*** Linear integral correction gain   :"
                << pImpl->integrationBasedIKIntegralLinearCorrectionGain;
        yInfo() << LogPrefix << "*** Angular integral correction gain  :"
                << pImpl->integrationBasedIKIntegralAngularCorrectionGain;
        yInfo() << LogPrefix
                << "*** Cost regularization              :" << pImpl->costRegularization;
        yInfo() << LogPrefix << "*** Joint velocity limit             :"
                << pImpl->integrationBasedJointVelocityLimit;
    }
    if (pImpl->ikSolver == SolverIK::integrationbased || pImpl->ikSolver == SolverIK::global) {
        yInfo() << LogPrefix << "*** Inverse Velocity Kinematics solver:"
                << pImpl->inverseVelocityKinematicsSolver;
    }
    yInfo() << LogPrefix << "*** ===========================================";

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
    yInfo() << LogPrefix << "----------------------------------------" << modelLoader.isValid();
    //yInfo() << LogPrefix << modelLoader.model().toString();
    yInfo() << LogPrefix << "Links : " << modelLoader.model().getNrOfLinks()
            << " , Joints: " << modelLoader.model().getNrOfJoints();

    yInfo() << LogPrefix << "Base Link: "
            << modelLoader.model().getLinkName(modelLoader.model().getDefaultBaseLink());

    // ====================
    // INITIALIZE VARIABLES
    // ====================

    // Get the model from the loader
    pImpl->humanModel = modelLoader.model();

    // Initialize kinDyn computation
    pImpl->kinDynComputations =
        std::unique_ptr<iDynTree::KinDynComputations>(new iDynTree::KinDynComputations());
    pImpl->kinDynComputations->loadRobotModel(modelLoader.model());
    pImpl->kinDynComputations->setFloatingBase(pImpl->floatingBaseFrame.model);

    // ================================
    // INITIALIZE ACCELEROMETER SENSORS
    // ================================

    pImpl->humanSensors = modelLoader.sensors();

    // Initialize human accelerometer sensor buffers from model data
    size_t nrOfAccelerometerSensors = pImpl->humanSensors.getNrOfSensors(iDynTree::ACCELEROMETER);

    pImpl->humanSensorData.accelerometerSensorNames.resize(nrOfAccelerometerSensors);
    pImpl->humanSensorData.accelerometerSensorMeasurements.resize(nrOfAccelerometerSensors);

    for (size_t i = 0; i < nrOfAccelerometerSensors; i++) {

        // Get the sensor pointer
        iDynTree::AccelerometerSensor *sensor = static_cast<iDynTree::AccelerometerSensor*>(pImpl->humanSensors.getSensor(iDynTree::ACCELEROMETER, i));

        if (!sensor->isValid() || !sensor->isConsistent(pImpl->humanModel)) {
            yError() << LogPrefix << "Error in reading human sensor";
            return false;
        }

        // Store the sensor name
        // The naming convention is assumed to be ParentLinkName_accelerometer e.g. LeftFoot_accelerometer
        pImpl->humanSensorData.accelerometerSensorNames.at(i) = sensor->getName();

        // Initialize zero default sensor measurements
        pImpl->humanSensorData.accelerometerSensorMeasurements.at(i) = std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    }

    // Initialize CoM proper acceleration to zero
    pImpl->CoMProperAcceleration = std::array<double, 3>{0.0, 0.0, 0.0};

    // Debug Info
    yInfo() << LogPrefix << "Accelerometers size : " << pImpl->humanSensorData.accelerometerSensorNames.size();

    // =========================
    // INITIALIZE JOINTS BUFFERS
    // =========================

    // Get the number of joints accordingly to the model
    const size_t nrOfDOFs = pImpl->humanModel.getNrOfDOFs();

    pImpl->solution.jointPositions.resize(nrOfDOFs);
    pImpl->solution.jointVelocities.resize(nrOfDOFs);

    pImpl->jointConfigurationSolution.resize(nrOfDOFs);
    pImpl->jointConfigurationSolution.zero();

    pImpl->jointVelocitiesSolution.resize(nrOfDOFs);
    pImpl->jointVelocitiesSolution.zero();

    pImpl->baseTransformSolution.setRotation(iDynTree::Rotation::Identity());

    // ====================================
    // INITIALIZE INVERSE KINEMATICS SOLVER
    // ====================================

    if (pImpl->ikSolver == SolverIK::pairwised) {
        if (!pImpl->initializePairwisedInverseKinematicsSolver()) {
            askToStop();
            return false;
        }
    }
    else if (pImpl->ikSolver == SolverIK::global) {
        if (!pImpl->initializeGlobalInverseKinematicsSolver()) {
            askToStop();
            return false;
        }
    }
    else if (pImpl->ikSolver == SolverIK::integrationbased) {
        if (!pImpl->initializeIntegrationBasedInverseKinematicsSolver()) {
            askToStop();
            return false;
        }
    }

    return true;
}

bool HumanStateProvider::close()
{
    return true;
}

void HumanStateProvider::run()
{
    // Get the link transformations from input data
    if (!pImpl->getLinkQuantitiesFromInputData(pImpl->linkTransformMatrices, pImpl->linkAccelerations)) {
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

    // If useXsensJointAngles is true get joint angles from input data
    if (pImpl->useXsensJointsAngles) {
        if (pImpl->getJointAnglesFromInputData(pImpl->jointConfigurationSolution)) {
            yError() << LogPrefix << "Failed to get joint angles from input data";
            askToStop();
            return;
        }
    }

    if (pImpl->useFBAccelerationFromWearableData) {

        // Get freebody acceleration from input wearable data
        if (!pImpl->getfbAccelerationFromInputData(pImpl->fbAccelerationMatrices)) {
            yError() << LogPrefix << "Failed to get freebody accelerometer values from input wearable data";
            askToStop();
            return;
        }

        // Get sensor orientation from input wearable data
        if (!pImpl->getOrientationFromInputData(pImpl->sensorOrientationMatrices)) {
            yError() << LogPrefix << "Failed to get sensor orientation values from input wearable data";
            askToStop();
            return;
        }
    }

    // Solve Inverse Kinematics and Inverse Velocity Problems
    auto tick = std::chrono::high_resolution_clock::now();
    bool inverseKinematicsFailure;
    if (pImpl->ikSolver == SolverIK::pairwised) {
        inverseKinematicsFailure = !(pImpl->solvePairwisedInverseKinematicsSolver());
    }
    else if (pImpl->ikSolver == SolverIK::global) {
        inverseKinematicsFailure = !pImpl->solveGlobalInverseKinematicsSolver();
    }
    else if (pImpl->ikSolver == SolverIK::integrationbased) {
        inverseKinematicsFailure = !pImpl->solveIntegrationBasedInverseKinematics();
    }

    // check if inverse kinematics failed
    if (inverseKinematicsFailure) {
        if (pImpl->allowIKFailures) {
            yWarning() << LogPrefix << "IK failed, keeping the previous solution";
            return;
        }
        else {
            yError() << LogPrefix << "Failed to solve IK";
            askToStop();
        }
        return;
    }

    auto tock = std::chrono::high_resolution_clock::now();
    yDebug() << LogPrefix << "IK took"
             << std::chrono::duration_cast<std::chrono::milliseconds>(tock - tick).count() << "ms";

    // If useDirectBaseMeasurement is true, set the measured base pose and velocity as solution
    if (pImpl->useDirectBaseMeasurement) {
        pImpl->baseTransformSolution = measuredBaseTransform;
        pImpl->baseVelocitySolution = measuredBaseVelocity;
    }

    // CoM position and velocity
    std::array<double, 3> CoM_position, CoM_velocity;
    iDynTree::KinDynComputations* kindyncomputations = pImpl->kinDynComputations.get();
    CoM_position = {kindyncomputations->getCenterOfMassPosition().getVal(0),
                    kindyncomputations->getCenterOfMassPosition().getVal(1),
                    kindyncomputations->getCenterOfMassPosition().getVal(2)};

    CoM_velocity = {kindyncomputations->getCenterOfMassVelocity().getVal(0),
                    kindyncomputations->getCenterOfMassVelocity().getVal(1),
                    kindyncomputations->getCenterOfMassVelocity().getVal(2)};

    // CoM acceleration
    std::array<double, 3> CoM_biasacceleration;
    CoM_biasacceleration = {kindyncomputations->getCenterOfMassBiasAcc().getVal(0),
                            kindyncomputations->getCenterOfMassBiasAcc().getVal(1),
                            kindyncomputations->getCenterOfMassBiasAcc().getVal(2)};

    // Compute proper acceleration
    if (pImpl->useFBAccelerationFromWearableData) {

        // Update kinDyn computations based on IK solution
        iDynTree::VectorDynSize solvedJointPositions(pImpl->solution.jointPositions.size());

        for (size_t j = 0; j < pImpl->solution.jointPositions.size(); j++) {
            solvedJointPositions.setVal(j, pImpl->solution.jointPositions.at(j));
        }

        iDynTree::VectorDynSize solvedJointVelocities(pImpl->solution.jointVelocities.size());

        for (size_t j = 0; j < pImpl->solution.jointVelocities.size(); j++) {
            solvedJointVelocities.setVal(j, pImpl->solution.jointVelocities.at(j));
        }

        pImpl->kinDynComputations->setRobotState(pImpl->baseTransformSolution,
                                                 solvedJointPositions,
                                                 pImpl->baseVelocitySolution,
                                                 solvedJointVelocities,
                                                 pImpl->worldGravity);

        // Iterate over the stored model sensors
        int accelerometerCount = 0;
        for (auto& accSensorName : pImpl->humanSensorData.accelerometerSensorNames) {

            // Extract the parent link name from the sensor name
            std::size_t separatorLocation = accSensorName.find_last_of("_");

            std::string accelerometerParentLinkName = accSensorName.substr(0, separatorLocation);

            if (pImpl->fbAccelerationMatrices.find(accelerometerParentLinkName)
                == pImpl->fbAccelerationMatrices.end()) {
                yWarning() << LogPrefix << "Failed to find" << accelerometerParentLinkName
                           << "entry in the fbAccelerationMatrices map. Skipping this link.";
                continue;
            }

            iDynTree::Transform base_H_sensor = kindyncomputations->getRelativeTransform(pImpl->humanModel.getFrameIndex(pImpl->humanSensorData.accelerometerSensorNames.at(accelerometerCount)),
                                                                                         pImpl->humanModel.getFrameIndex(pImpl->floatingBaseFrame.model));

            iDynTree::Transform world_H_accelerometer = pImpl->baseTransformSolution *
                                                        base_H_sensor;

            // Get accelerometer fbAcceleration value stored in the buffer
            iDynTree::LinAcceleration fbAcceleration = pImpl->fbAccelerationMatrices.at(accelerometerParentLinkName);

            //yInfo() << "====================Accelerometer name : " << pImpl->humanSensorData.accelerometerSensorNames.at(accelerometerCount) << "=================";
            //yInfo() << "Free body acceleration : " << fbAcceleration.toString().c_str();

            // Compute corrected acceleration
            iDynTree::SpatialAcc correctedAcceleration;

            if (pImpl->humanSensorData.accelerometerSensorMeasurementsOption == "proper") {

                // Set the linear part to corrected acceleartion
                correctedAcceleration.setVal(0, fbAcceleration.getVal(0) - pImpl->worldGravity(0));
                correctedAcceleration.setVal(1, fbAcceleration.getVal(1) - pImpl->worldGravity(1));
                correctedAcceleration.setVal(2, fbAcceleration.getVal(2) - pImpl->worldGravity(2));
            }
            else if (pImpl->humanSensorData.accelerometerSensorMeasurementsOption == "gravity") {

                // Set the linear part to corrected acceleartion
                correctedAcceleration.setVal(0,  - pImpl->worldGravity(0));
                correctedAcceleration.setVal(1,  - pImpl->worldGravity(1));
                correctedAcceleration.setVal(2,  - pImpl->worldGravity(2));
            }

            //yInfo() << LogPrefix << "Link accelerations : " << pImpl->linkAccelerations.at(accelerometerParentLinkName).toString().c_str();

            // Set the angular part to link angular acceleration
            iDynTree::AngAcceleration linkAngAcc = pImpl->linkAccelerations.at(accelerometerParentLinkName).getAngularVec3();

            //yInfo() << LogPrefix << "Link angular acceleration : " << linkAngAcc.toString().c_str();

            correctedAcceleration.setVal(3, linkAngAcc.getVal(0));
            correctedAcceleration.setVal(4, linkAngAcc.getVal(1));
            correctedAcceleration.setVal(5, linkAngAcc.getVal(2));

//            yInfo() << LogPrefix << "World_H_Base transform : ";
//            yInfo() << "Position : " << pImpl->baseTransformSolution.getPosition().toString().c_str();
//            yInfo() << "Rotation : " << pImpl->baseTransformSolution.getRotation().toString().c_str();

//            yInfo() << LogPrefix << "Base_H_Sensor transform : ";
//            yInfo() << "Position : " << base_H_sensor.getPosition().toString().c_str();
//            yInfo() << "Rotation : " << base_H_sensor.getRotation().toString().c_str();

//            yInfo() << LogPrefix << "World_H_Sensor transform : ";
//            yInfo() << "Position : " << world_H_accelerometer.getPosition().toString().c_str();
//            yInfo() << "Rotation : " << world_H_accelerometer.getRotation().toString().c_str();


//            yInfo() << "Corrected accceleration : " << correctedAcceleration.toString().c_str();

//            // TODO: Double check this computation
//            iDynTree::Rotation w_R_accelerometer = world_H_accelerometer.getRotation();
//            iDynTree::SpatialAcc properAcceleration = w_R_accelerometer.inverse() * correctedAcceleration;

//            yInfo() << "Sensor orientaion : " << pImpl->sensorOrientationMatrices.at(accelerometerParentLinkName).toString().c_str();

            // Compute proper acceleration
            iDynTree::SpatialAcc properAcceleration = pImpl->sensorOrientationMatrices.at(accelerometerParentLinkName).inverse() * correctedAcceleration;

            // Expose proper angular acceleration for IHumanState interface
            {

                std::lock_guard<std::mutex> lock(pImpl->mutex);

                std::array<double, 6> properAcce = {properAcceleration.getVal(0),
                                                    properAcceleration.getVal(1),
                                                    properAcceleration.getVal(2),
                                                    properAcceleration.getVal(3),
                                                    properAcceleration.getVal(4),
                                                    properAcceleration.getVal(5)};

                pImpl->humanSensorData.accelerometerSensorMeasurements.at(accelerometerCount) = properAcce;

//                yInfo() << "Proper acceleraton : " << properAcceleration.toString().c_str();
//                yInfo() << "================================================================================================================";

                // Increase accelerometer count
                accelerometerCount++;


            }

        }

        // Compute CoM proper acceleration
        iDynTree::SpatialAcc comSpatialAcc; // abuse of notation

        if (pImpl->humanSensorData.accelerometerSensorMeasurementsOption == "proper") {

            // Set the linear part of com spatial acceleartion
            comSpatialAcc.setVal(0, pImpl->humanModel.getTotalMass() * (CoM_biasacceleration[0] - pImpl->worldGravity(0)));
            comSpatialAcc.setVal(1, pImpl->humanModel.getTotalMass() * (CoM_biasacceleration[1] - pImpl->worldGravity(1)));
            comSpatialAcc.setVal(2, pImpl->humanModel.getTotalMass() * (CoM_biasacceleration[2] - pImpl->worldGravity(2)));
        }
        else if (pImpl->humanSensorData.accelerometerSensorMeasurementsOption == "gravity") {

            // Set the linear part of com spatial acceleartion
            comSpatialAcc.setVal(0,  - pImpl->worldGravity(0) * pImpl->humanModel.getTotalMass());
            comSpatialAcc.setVal(1,  - pImpl->worldGravity(1) * pImpl->humanModel.getTotalMass());
            comSpatialAcc.setVal(2,  - pImpl->worldGravity(2) * pImpl->humanModel.getTotalMass());
        }

        // Set the angular part of com spatial acceleration to zero
        comSpatialAcc.setVal(3, 0.0);
        comSpatialAcc.setVal(4, 0.0);
        comSpatialAcc.setVal(5, 0.0);

        // Compute com proper acceleration and multiply with the total model mass
        iDynTree::SpatialAcc comProperAcceleration = pImpl->baseTransformSolution.getRotation().inverse() * comSpatialAcc;

        // Expose proper com acceleration for IHumanState interface
        {

            std::lock_guard<std::mutex> lock(pImpl->mutex);

            pImpl->CoMProperAcceleration = {comProperAcceleration.getVal(0),
                                            comProperAcceleration.getVal(1),
                                            comProperAcceleration.getVal(2)};

        }

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
            pImpl->baseTransformSolution.getRotation().asQuaternion().getVal(3)};

        // Use measured base frame velocity
        pImpl->solution.baseVelocity = {pImpl->baseVelocitySolution.getVal(0),
                                        pImpl->baseVelocitySolution.getVal(1),
                                        pImpl->baseVelocitySolution.getVal(2),
                                        pImpl->baseVelocitySolution.getVal(3),
                                        pImpl->baseVelocitySolution.getVal(4),
                                        pImpl->baseVelocitySolution.getVal(5)};
        // CoM position and velocity
        pImpl->solution.CoMPosition = {CoM_position[0], CoM_position[1], CoM_position[2]};
        pImpl->solution.CoMVelocity = {CoM_velocity[0], CoM_velocity[1], CoM_velocity[2]};

        // CoM bias acceleration
        pImpl->solution.CoMBiasAcceleration = {CoM_biasacceleration[0], CoM_biasacceleration[1], CoM_biasacceleration[2]};
    }

    // compute the inverse kinematic errors (currently the result is unused, but it may be used for
    // evaluating the IK performance)
    // pImpl->computeLinksOrientationErrors(pImpl->linkTransformMatrices,
    //                                      pImpl->jointConfigurationSolution,
    //                                      pImpl->baseTransformSolution,
    //                                      pImpl->linkErrorOrientations);
    // pImpl->computeLinksAngularVelocityErrors(pImpl->linkVelocities,
    //                                          pImpl->jointConfigurationSolution,
    //                                          pImpl->baseTransformSolution,
    //                                          pImpl->jointVelocitiesSolution,
    //                                          pImpl->baseVelocitySolution,
    //                                          pImpl->linkErrorAngularVelocities);
}

bool HumanStateProvider::impl::getLinkQuantitiesFromInputData(
    std::unordered_map<std::string, iDynTree::Transform>& transforms,
    std::unordered_map<std::string, iDynTree::SpatialAcc>& linkAcc)
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

        // Get link transform from wearable data
        wearable::Vector3 position;
        if (!sensor->getLinkPosition(position)) {
            yError() << LogPrefix << "Failed to read link position from virtual link sensor " << wearableLinkName;
            return false;
        }

        iDynTree::Position pos(position.at(0), position.at(1), position.at(2));

        Quaternion orientation;
        if (!sensor->getLinkOrientation(orientation)) {
            yError() << LogPrefix << "Failed to read link orientation from virtual link sensor " << wearableLinkName;
            return false;
        }

        iDynTree::Rotation rotation;
        rotation.fromQuaternion({orientation.data(), 4});

        iDynTree::Transform transform(rotation, pos);

        // Note that this map is used during the IK step for setting a target transform to a
        // link of the model. For this reason the map keys are model names.
        transforms[modelLinkName] = std::move(transform);

        // Get link acceleration from wearable data
        wearable::Vector3 linkLinAcc;
        wearable::Vector3 linkAngAcc;
        if (!sensor->getLinkAcceleration(linkLinAcc, linkAngAcc)) {
            yError() << LogPrefix << "Failed to read link acceleration from virtual link sensor " << wearableLinkName;
            return false;
        }

        iDynTree::SpatialAcc linkSpaAcc;
        linkSpaAcc.setVal(0, linkLinAcc.at(0));
        linkSpaAcc.setVal(1, linkLinAcc.at(1));
        linkSpaAcc.setVal(2, linkLinAcc.at(2));
        linkSpaAcc.setVal(3, linkAngAcc.at(0));
        linkSpaAcc.setVal(4, linkAngAcc.at(1));
        linkSpaAcc.setVal(5, linkAngAcc.at(2));

        linkAcc[modelLinkName] = std::move(linkSpaAcc);

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
            yError() << LogPrefix
                     << "Failed to read link angular velocity from virtual link sensor";
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

bool HumanStateProvider::impl::getOrientationFromInputData(std::unordered_map<std::string, iDynTree::Rotation>& ori) {

    for (const auto& parentLinkMapEntry : wearableStorage.modelToWearable_SensorOrientationParentLinkName) {

        const ModelLinkName& modelParentLinkName = parentLinkMapEntry.first;
        const WearableLinkName& wearableParentLinkName = parentLinkMapEntry.second;

        if (wearableStorage.orientationSensorsMap.find(wearableParentLinkName)
                == wearableStorage.orientationSensorsMap.end()
            || !wearableStorage.orientationSensorsMap.at(wearableParentLinkName)) {

            yError() << LogPrefix << "Failed to get" << wearableParentLinkName
                     << "sensor from the device. Something happened after configuring it.";
            return false;

        }

        const wearable::SensorPtr<const sensor::IOrientationSensor> sensor =
                wearableStorage.orientationSensorsMap.at(wearableParentLinkName);

        if (!sensor) {
            yError() << LogPrefix << "Sensor" << wearableParentLinkName
                     << "has been added but not properly configured";
            return false;
        }


        if (sensor->getSensorStatus() != sensor::SensorStatus::Ok) {
            yError() << LogPrefix << "The sensor status of" << sensor->getSensorName()
                     << "is not ok (" << static_cast<double>(sensor->getSensorStatus()) << ")";
            return false;
        }

        // Wearable quaternion is defined with real part first in the thrif file
        // https://github.com/robotology/wearables/blob/master/msgs/thrift/WearableData.thrift#L38
        wearable::Quaternion accelerometerOrientation;
        if (!sensor->getOrientationAsQuaternion(accelerometerOrientation)) {
            yError() << LogPrefix << "Failed to get orientation from orientation wearable sensor";
            return false;
        }

        // iDynTree quaternion takes real part first
        // http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1Rotation.html#a40da839b3ed7cf1be092ce4c8056467a
        iDynTree::Vector4 quat;
        quat.setVal(0, accelerometerOrientation.at(0));
        quat.setVal(1, accelerometerOrientation.at(1));
        quat.setVal(2, accelerometerOrientation.at(2));
        quat.setVal(3, accelerometerOrientation.at(3));

        iDynTree::Rotation rot;
        rot.fromQuaternion(quat);

        ori[modelParentLinkName] = std::move(rot);
    }

    return true;
}

bool HumanStateProvider::impl::getfbAccelerationFromInputData(std::unordered_map<std::string, iDynTree::AngAcceleration>& acc) {

    for (const auto& parentLinkMapEntry : wearableStorage.modelToWearable_AccelerometerParentLinkName) {

        const ModelLinkName& modelParentLinkName = parentLinkMapEntry.first;
        const WearableLinkName& wearableParentLinkName = parentLinkMapEntry.second;

        if (wearableStorage.accelerometerSensorsMap.find(wearableParentLinkName)
                == wearableStorage.accelerometerSensorsMap.end()
            || !wearableStorage.accelerometerSensorsMap.at(wearableParentLinkName)) {

            yError() << LogPrefix << "Failed to get" << wearableParentLinkName
                     << "sensor from the device. Something happened after configuring it.";
            return false;

        }

        const wearable::SensorPtr<const sensor::IFreeBodyAccelerationSensor> sensor =
            wearableStorage.accelerometerSensorsMap.at(wearableParentLinkName);

        if (!sensor) {
            yError() << LogPrefix << "Sensor" << wearableParentLinkName
                     << "has been added but not properly configured";
            return false;
        }

        if (sensor->getSensorStatus() != sensor::SensorStatus::Ok) {
            yError() << LogPrefix << "The sensor status of" << sensor->getSensorName()
                     << "is not ok (" << static_cast<double>(sensor->getSensorStatus()) << ")";
            return false;
        }

        wearable::Vector3 freeBodyAcceleration;
        if (!sensor->getFreeBodyAcceleration(freeBodyAcceleration)) {
            yError() << LogPrefix << "Failed to get freebody acceleration from freeBodyAcceleration sensor";
            return false;
        }

        iDynTree::AngAcceleration fbAcceleration(freeBodyAcceleration.at(0), freeBodyAcceleration.at(1), freeBodyAcceleration.at(2));

        acc[modelParentLinkName] = std::move(fbAcceleration);

    }

    return true;
}

bool HumanStateProvider::impl::initializePairwisedInverseKinematicsSolver()
{
    // Get the model link names according to the modelToWearable link sensor map
    const size_t nrOfSegments = wearableStorage.modelToWearable_LinkName.size();

    segments.resize(nrOfSegments);

    unsigned segmentIndex = 0;
    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        // Get the name of the link from the model and its prefix from iWear
        std::string modelLinkName = humanModel.getLinkName(linkIndex);

        if (wearableStorage.modelToWearable_LinkName.find(modelLinkName)
            == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        // TODO check if we need this initialization
        // segments[segmentIndex].velocities.zero();

        // Store the name of the link as segment name
        segments[segmentIndex].segmentName = modelLinkName;
        segmentIndex++;
    }

    // Get all the possible pairs composing the model
    std::vector<std::pair<std::string, std::string>> pairNames;
    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>> pairSegmentIndeces;

    // Get the link pair names
    createEndEffectorsPairs(humanModel, segments, pairNames, pairSegmentIndeces);
    linkPairs.reserve(pairNames.size());

    for (unsigned index = 0; index < pairNames.size(); ++index) {
        LinkPairInfo pairInfo;

        pairInfo.parentFrameName = pairNames[index].first;
        pairInfo.parentFrameSegmentsIndex = pairSegmentIndeces[index].first;

        pairInfo.childFrameName = pairNames[index].second;
        pairInfo.childFrameSegmentsIndex = pairSegmentIndeces[index].second;

        // Get the reduced pair model
        if (!getReducedModel(humanModel,
                             pairInfo.parentFrameName,
                             pairInfo.childFrameName,
                             pairInfo.pairModel)) {

            yWarning() << LogPrefix << "failed to get reduced model for the segment pair "
                       << pairInfo.parentFrameName.c_str() << ", "
                       << pairInfo.childFrameName.c_str();
            continue;
        }

        // Allocate the ik solver
        pairInfo.ikSolver = std::make_unique<iDynTree::InverseKinematics>();

        // Set ik parameters
        pairInfo.ikSolver->setVerbosity(1);
        pairInfo.ikSolver->setLinearSolverName(linearSolverName);
        pairInfo.ikSolver->setMaxIterations(maxIterationsIK);
        pairInfo.ikSolver->setCostTolerance(costTolerance);
        pairInfo.ikSolver->setDefaultTargetResolutionMode(
            iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
        pairInfo.ikSolver->setRotationParametrization(
            iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

        // Set ik model
        if (!pairInfo.ikSolver->setModel(pairInfo.pairModel)) {
            yWarning() << LogPrefix << "failed to configure IK solver for the segment pair"
                       << pairInfo.parentFrameName.c_str() << ", "
                       << pairInfo.childFrameName.c_str() << " Skipping pair";
            continue;
        }

        // Add parent link as fixed base constraint with identity transform
        pairInfo.ikSolver->addFrameConstraint(pairInfo.parentFrameName,
                                              iDynTree::Transform::Identity());

        // Add child link as a target and set initial transform to be identity
        pairInfo.ikSolver->addTarget(pairInfo.childFrameName, iDynTree::Transform::Identity());

        // Add target position and rotation weights
        pairInfo.positionTargetWeight = posTargetWeight;
        pairInfo.rotationTargetWeight = rotTargetWeight;

        // Add cost regularization term
        pairInfo.costRegularization = costRegularization;

        // Get floating base for the pair model
        pairInfo.floatingBaseIndex = pairInfo.pairModel.getFrameLink(
            pairInfo.pairModel.getFrameIndex(pairInfo.parentFrameName));

        // Set ik floating base
        if (!pairInfo.ikSolver->setFloatingBaseOnFrameNamed(
                pairInfo.pairModel.getLinkName(pairInfo.floatingBaseIndex))) {
            yError() << "Failed to set floating base frame for the segment pair"
                     << pairInfo.parentFrameName.c_str() << ", " << pairInfo.childFrameName.c_str()
                     << " Skipping pair";
            return false;
        }

        // Set initial joint positions size
        pairInfo.sInitial.resize(pairInfo.pairModel.getNrOfJoints());

        // Obtain the joint location index in full model and the lenght of DoFs i.e joints map
        // This information will be used to put the IK solutions together for the full model
        std::vector<std::string> solverJoints;

        // Resize to number of joints in the pair model
        solverJoints.resize(pairInfo.pairModel.getNrOfJoints());

        for (int i = 0; i < pairInfo.pairModel.getNrOfJoints(); i++) {
            solverJoints[i] = pairInfo.pairModel.getJointName(i);
        }

        pairInfo.consideredJointLocations.reserve(solverJoints.size());
        for (auto& jointName : solverJoints) {
            iDynTree::JointIndex jointIndex = humanModel.getJointIndex(jointName);
            if (jointIndex == iDynTree::JOINT_INVALID_INDEX) {
                yWarning() << LogPrefix << "IK considered joint " << jointName
                           << " not found in the complete model";
                continue;
            }
            iDynTree::IJointConstPtr joint = humanModel.getJoint(jointIndex);

            // Save location index and length of each DoFs
            pairInfo.consideredJointLocations.push_back(
                std::pair<size_t, size_t>(joint->getDOFsOffset(), joint->getNrOfDOFs()));
        }

        // Set the joint configurations size and initialize to zero
        pairInfo.jointConfigurations.resize(solverJoints.size());
        pairInfo.jointConfigurations.zero();

        // Set the joint velocities size and initialize to zero
        pairInfo.jointVelocities.resize(solverJoints.size());
        pairInfo.jointVelocities.zero();

        // Save the indeces
        // TODO: check if link or frame
        pairInfo.parentFrameModelIndex = pairInfo.pairModel.getFrameIndex(pairInfo.parentFrameName);
        pairInfo.childFrameModelIndex = pairInfo.pairModel.getFrameIndex(pairInfo.childFrameName);

        // Configure KinDynComputation
        pairInfo.kinDynComputations =
            std::unique_ptr<iDynTree::KinDynComputations>(new iDynTree::KinDynComputations());
        pairInfo.kinDynComputations->loadRobotModel(pairInfo.pairModel);

        // Configure relative Jacobian
        pairInfo.relativeJacobian.resize(6, pairInfo.pairModel.getNrOfDOFs());
        pairInfo.relativeJacobian.zero();

        // Move the link pair instance into the vector
        linkPairs.push_back(std::move(pairInfo));
    }

    // Initialize IK Worker Pool
    ikPool = std::unique_ptr<IKWorkerPool>(new IKWorkerPool(ikPoolSize, linkPairs, segments));
    if (!ikPool) {
        yError() << LogPrefix << "failed to create IK worker pool";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex);

        // Set the initial solution in the middle between lower and upper limits
        for (auto& linkPair : linkPairs) {
            for (size_t i = 0; i < linkPair.pairModel.getNrOfJoints(); i++) {
                double minJointLimit = linkPair.pairModel.getJoint(i)->getMinPosLimit(i);
                double maxJointLimit = linkPair.pairModel.getJoint(i)->getMaxPosLimit(i);
                double averageJointLimit = (minJointLimit + maxJointLimit) / 2.0;
                linkPair.sInitial.setVal(i, averageJointLimit);
            }
        }
    }

    return true;
}

bool HumanStateProvider::impl::initializeGlobalInverseKinematicsSolver()
{
    // Set global ik parameters
    globalIK.setVerbosity(1);
    globalIK.setLinearSolverName(linearSolverName);
    globalIK.setMaxIterations(maxIterationsIK);
    globalIK.setCostTolerance(costTolerance);
    globalIK.setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
    globalIK.setRotationParametrization(
        iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

    if (!globalIK.setModel(humanModel)) {
        yError() << LogPrefix << "globalIK: failed to load the model";
        return false;
    }

    if (!globalIK.setFloatingBaseOnFrameNamed(floatingBaseFrame.model)) {
        yError() << LogPrefix << "Failed to set the globalIK floating base frame on link"
                 << floatingBaseFrame.model;
        return false;
    }

    if (!addInverseKinematicTargets()) {
        yError() << LogPrefix << "Failed to set the globalIK targets";
        return false;
    }

    // Set global Inverse Velocity Kinematics parameters
    inverseVelocityKinematics.setResolutionMode(inverseVelocityKinematicsSolver);
    inverseVelocityKinematics.setRegularization(costRegularization);

    if (!inverseVelocityKinematics.setModel(humanModel)) {
        yError() << LogPrefix << "IBIK: failed to load the model";
        return false;
    }

    if (!inverseVelocityKinematics.setFloatingBaseOnFrameNamed(floatingBaseFrame.model)) {
        yError() << LogPrefix << "Failed to set the IBIK floating base frame on link"
                 << floatingBaseFrame.model;
        return false;
    }

    if (!addInverseVelocityKinematicsTargets()) {
        yError() << LogPrefix << "Failed to set the inverse velocity kinematics targets";
        return false;
    }
    return true;
}

bool HumanStateProvider::impl::initializeIntegrationBasedInverseKinematicsSolver()
{
    // Initialize state integrator
    stateIntegrator.setInterpolatorType(iDynTreeHelper::State::integrator::trapezoidal);
    stateIntegrator.setNJoints(humanModel.getNrOfDOFs());

    iDynTree::VectorDynSize jointLowerLimits;
    jointLowerLimits.resize(humanModel.getNrOfDOFs());
    iDynTree::VectorDynSize jointUpperLimits;
    jointUpperLimits.resize(humanModel.getNrOfDOFs());
    for (int jointIndex = 0; jointIndex < humanModel.getNrOfDOFs(); jointIndex++) {
        jointLowerLimits.setVal(jointIndex, humanModel.getJoint(jointIndex)->getMinPosLimit(0));
        jointUpperLimits.setVal(jointIndex, humanModel.getJoint(jointIndex)->getMaxPosLimit(0));
    }
    stateIntegrator.setJointLimits(jointLowerLimits, jointUpperLimits);

    integralOrientationError.zero();

    // Set global Inverse Velocity Kinematics parameters
    inverseVelocityKinematics.setResolutionMode(inverseVelocityKinematicsSolver);
    // Set Regularization Term:
    inverseVelocityKinematics.setRegularization(costRegularization);

    if (!inverseVelocityKinematics.setModel(humanModel)) {
        yError() << LogPrefix << "IBIK: failed to load the model";
        return false;
    }

    if (!inverseVelocityKinematics.setFloatingBaseOnFrameNamed(floatingBaseFrame.model)) {
        yError() << LogPrefix << "Failed to set the IBIK floating base frame on link"
                 << floatingBaseFrame.model;
        return false;
    }

    if (!addInverseVelocityKinematicsTargets()) {
        yError() << LogPrefix << "Failed to set the inverse velocity kinematics targets";
        return false;
    }

    return true;
}

bool HumanStateProvider::impl::solvePairwisedInverseKinematicsSolver()
{
    {
        std::lock_guard<std::mutex> lock(mutex);

        // Set link segments transformation and velocity
        for (size_t segmentIndex = 0; segmentIndex < segments.size(); segmentIndex++) {

            SegmentInfo& segmentInfo = segments.at(segmentIndex);
            segmentInfo.poseWRTWorld = linkTransformMatrices.at(segmentInfo.segmentName);
            segmentInfo.velocities = linkVelocities.at(segmentInfo.segmentName);
        }
    }

    // Call IK worker pool to solve
    ikPool->runAndWait();

    // Joint link pair ik solutions using joints map from link pairs initialization
    // to solution struct for exposing data through interface
    for (auto& linkPair : linkPairs) {
        size_t jointIndex = 0;
        for (auto& pairJoint : linkPair.consideredJointLocations) {

            // Check if it is a valid 1 DoF joint
            if (pairJoint.second == 1) {
                jointConfigurationSolution.setVal(pairJoint.first,
                                                  linkPair.jointConfigurations.getVal(jointIndex));
                jointVelocitiesSolution.setVal(pairJoint.first,
                                               linkPair.jointVelocities.getVal(jointIndex));

                linkPair.sInitial.setVal(jointIndex,
                                         linkPair.jointConfigurations.getVal(jointIndex));
                jointIndex++;
            }
            else {
                yWarning()
                    << LogPrefix
                    << " Invalid DoFs for the joint, skipping the ik solution for this joint";
                continue;
            }
        }
    }

    return true;
}

bool HumanStateProvider::impl::solveGlobalInverseKinematicsSolver()
{
    // Set global IK initial condition
    if (!globalIK.setFullJointsInitialCondition(&baseTransformSolution,
                                                &jointConfigurationSolution)) {
        yError() << LogPrefix
                 << "Failed to set the joint configuration for initializing the global IK";
        return false;
    }

    // Update ik targets based on wearable input data
    if (!updateInverseKinematicTargets()) {
        yError() << LogPrefix << "Failed to update the targets for the global IK";
        return false;
    }

    // Use a postural task for regularization
    iDynTree::VectorDynSize posturalTaskJointAngles;
    posturalTaskJointAngles.resize(jointConfigurationSolution.size());
    posturalTaskJointAngles.zero();
    if (!globalIK.setDesiredFullJointsConfiguration(posturalTaskJointAngles, costRegularization)) {
        yError() << LogPrefix << "Failed to set the postural configuration of the IK";
        return false;
    }

    if (!globalIK.solve()) {
        yError() << LogPrefix << "Failed to solve global IK";
        return false;
    }

    // Get the global inverse kinematics solution
    globalIK.getFullJointsSolution(baseTransformSolution, jointConfigurationSolution);

    // INVERSE VELOCITY KINEMATICS
    // Set joint configuration
    if (!inverseVelocityKinematics.setConfiguration(baseTransformSolution,
                                                    jointConfigurationSolution)) {
        yError() << LogPrefix
                 << "Failed to set the joint configuration for initializing the inverse velocity "
                    "kinematics";
        return false;
    }

    // Update ivk velocity targets based on wearable input data
    if (!updateInverseVelocityKinematicTargets()) {
        yError() << LogPrefix << "Failed to update the targets for the inverse velocity kinematics";
        return false;
    }

    if (!inverseVelocityKinematics.solve()) {
        yError() << LogPrefix << "Failed to solve inverse velocity kinematics";
        return false;
    }

    inverseVelocityKinematics.getVelocitySolution(baseVelocitySolution, jointVelocitiesSolution);

    return true;
}

bool HumanStateProvider::impl::solveIntegrationBasedInverseKinematics()
{
    // compute timestep
    double dt;
    if (lastTime < 0.0) {
        dt = period;
    }
    else {
        dt = yarp::os::Time::now() - lastTime;
    };
    lastTime = yarp::os::Time::now();

    // LINK VELOCITY CORRECTION
    iDynTree::KinDynComputations* computations = kinDynComputations.get();

    if (useDirectBaseMeasurement) {
        computations->setRobotState(
            jointConfigurationSolution, jointVelocitiesSolution, worldGravity);
    }
    else {
        computations->setRobotState(baseTransformSolution,
                                    jointConfigurationSolution,
                                    baseVelocitySolution,
                                    jointVelocitiesSolution,
                                    worldGravity);
    }

    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        std::string linkName = humanModel.getLinkName(linkIndex);

        // skip fake links
        if (wearableStorage.modelToWearable_LinkName.find(linkName)
            == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        iDynTree::Rotation rotationError =
            computations->getWorldTransform(humanModel.getFrameIndex(linkName)).getRotation()
            * linkTransformMatrices[linkName].getRotation().inverse();
        iDynTree::Vector3 angularVelocityError;

        angularVelocityError = iDynTreeHelper::Rotation::skewVee(rotationError);
        iDynTree::toEigen(integralOrientationError) =
            iDynTree::toEigen(integralOrientationError)
            + iDynTree::toEigen(angularVelocityError) * dt;

        // for floating base link use error also on position if not useDirectBaseMeasurement,
        // otherwise skip the link
        if (linkName == floatingBaseFrame.model) {
            if (useDirectBaseMeasurement) {
                continue;
            }

            iDynTree::Vector3 linearVelocityError;
            linearVelocityError =
                computations->getWorldTransform(humanModel.getFrameIndex(linkName)).getPosition()
                - linkTransformMatrices[linkName].getPosition();
            iDynTree::toEigen(integralLinearVelocityError) =
                iDynTree::toEigen(integralLinearVelocityError)
                + iDynTree::toEigen(linearVelocityError) * dt;
            for (int i = 0; i < 3; i++) {
                linkVelocities[linkName].setVal(i,
                                                integrationBasedIKMeasuredLinearVelocityGain
                                                        * linkVelocities[linkName].getVal(i)
                                                    - integrationBasedIKLinearCorrectionGain
                                                          * linearVelocityError.getVal(i)
                                                    - integrationBasedIKIntegralLinearCorrectionGain
                                                          * integralLinearVelocityError.getVal(i));
            }
        }

        // correct the links angular velocities
        for (int i = 3; i < 6; i++) {
            linkVelocities[linkName].setVal(
                i,
                integrationBasedIKMeasuredAngularVelocityGain * linkVelocities[linkName].getVal(i)
                    - integrationBasedIKAngularCorrectionGain * angularVelocityError.getVal(i - 3)
                    - integrationBasedIKIntegralAngularCorrectionGain
                          * integralOrientationError.getVal(i - 3));
        }
    }

    // INVERSE VELOCITY KINEMATICS
    // Set joint configuration
    if (!inverseVelocityKinematics.setConfiguration(baseTransformSolution,
                                                    jointConfigurationSolution)) {
        yError() << LogPrefix
                 << "Failed to set the joint configuration for initializing the global IK";
        return false;
    }

    // Update ivk velocity targets based on wearable input data
    if (!updateInverseVelocityKinematicTargets()) {
        return false;
    }

    if (!inverseVelocityKinematics.solve()) {
        yError() << LogPrefix << "Failed to solve inverse velocity kinematics";
        return false;
    }

    inverseVelocityKinematics.getVelocitySolution(baseVelocitySolution, jointVelocitiesSolution);

    // Threshold to limitate joint velocity
    for (unsigned i = 0; i < jointVelocitiesSolution.size(); i++) {
        if (integrationBasedJointVelocityLimit > 0
            && jointVelocitiesSolution.getVal(i) > integrationBasedJointVelocityLimit) {
            yWarning() << LogPrefix << "joint velocity out of limit: " << humanModel.getJointName(i)
                       << " : " << jointVelocitiesSolution.getVal(i);
            jointVelocitiesSolution.setVal(i, integrationBasedJointVelocityLimit);
        }
        else if (integrationBasedJointVelocityLimit > 0
                 && jointVelocitiesSolution.getVal(i)
                        < (-1.0 * integrationBasedJointVelocityLimit)) {
            yWarning() << LogPrefix << "joint velocity out of limit: " << humanModel.getJointName(i)
                       << " : " << jointVelocitiesSolution.getVal(i);
            jointVelocitiesSolution.setVal(i, -1.0 * integrationBasedJointVelocityLimit);
        }
    }

    // VELOCITY INTEGRATION
    // integrate velocities measurements
    if (!useDirectBaseMeasurement) {
        stateIntegrator.integrate(jointVelocitiesSolution,
                                  baseVelocitySolution.getLinearVec3(),
                                  baseVelocitySolution.getAngularVec3(),
                                  dt);

        stateIntegrator.getJointConfiguration(jointConfigurationSolution);
        stateIntegrator.getBasePose(baseTransformSolution);
    }
    else {
        stateIntegrator.integrate(jointVelocitiesSolution, dt);

        stateIntegrator.getJointConfiguration(jointConfigurationSolution);
    }

    return true;
}

bool HumanStateProvider::impl::updateInverseKinematicTargets()
{
    iDynTree::Transform linkTransform;

    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        std::string linkName = humanModel.getLinkName(linkIndex);

        // Skip links with no associated measures (use only links from the configuration)
        if (wearableStorage.modelToWearable_LinkName.find(linkName)
            == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        // For the link used as base insert both the rotation and position cost if not using direcly
        // measurement from xsens
        if (linkName == floatingBaseFrame.model) {
            if (!useDirectBaseMeasurement
                && !globalIK.updateTarget(linkName, linkTransformMatrices.at(linkName), 1.0, 1.0)) {
                yError() << LogPrefix << "Failed to update target for floating base" << linkName;
                return false;
            }
            continue;
        }

        if (linkTransformMatrices.find(linkName) == linkTransformMatrices.end()) {
            yError() << LogPrefix << "Failed to find transformation matrix for link" << linkName;
            return false;
        }

        linkTransform = linkTransformMatrices.at(linkName);
        // if useDirectBaseMeasurement, use the link transform relative to the base
        if (useDirectBaseMeasurement) {
            linkTransform =
                linkTransformMatrices.at(floatingBaseFrame.model).inverse() * linkTransform;
        }

        if (!globalIK.updateTarget(linkName, linkTransform, posTargetWeight, rotTargetWeight)) {
            yError() << LogPrefix << "Failed to update target for link" << linkName;
            return false;
        }
    }
    return true;
}

bool HumanStateProvider::impl::addInverseKinematicTargets()
{
    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        std::string linkName = humanModel.getLinkName(linkIndex);

        // Skip the fake links
        if (wearableStorage.modelToWearable_LinkName.find(linkName)
            == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        // Insert in the cost the rotation and position of the link used as base
        if (linkName == floatingBaseFrame.model) {
            if (!useDirectBaseMeasurement
                && !globalIK.addTarget(linkName, iDynTree::Transform::Identity(), 1.0, 1.0)) {
                yError() << LogPrefix << "Failed to add target for floating base link" << linkName;
                return false;
            }
            else if (useDirectBaseMeasurement
                     && !globalIK.addFrameConstraint(linkName, iDynTree::Transform::Identity())) {
                yError() << LogPrefix << "Failed to add constraint for base link" << linkName;
                return false;
            }
            continue;
        }

        // Add ik targets and set to identity
        if (!globalIK.addTarget(
                linkName, iDynTree::Transform::Identity(), posTargetWeight, rotTargetWeight)) {
            yError() << LogPrefix << "Failed to add target for link" << linkName;
            return false;
        }
    }
    return true;
}

bool HumanStateProvider::impl::updateInverseVelocityKinematicTargets()
{
    iDynTree::Twist linkTwist;

    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        std::string linkName = humanModel.getLinkName(linkIndex);

        // Skip links with no associated measures (use only links from the configuration)
        if (wearableStorage.modelToWearable_LinkName.find(linkName)
            == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        // For the link used as base insert both the rotation and position cost if not using direcly
        // measurement from xsens
        if (linkName == floatingBaseFrame.model) {
            if (!useDirectBaseMeasurement
                && !inverseVelocityKinematics.updateTarget(
                    linkName, linkVelocities.at(linkName), 1.0, 1.0)) {
                yError() << LogPrefix << "Failed to update velocity target for floating base"
                         << linkName;
                return false;
            }
            continue;
        }

        if (linkVelocities.find(linkName) == linkVelocities.end()) {
            yError() << LogPrefix << "Failed to find twist for link" << linkName;
            return false;
        }

        linkTwist = linkVelocities.at(linkName);
        if (useDirectBaseMeasurement) {
            linkTwist = linkTwist - linkVelocities.at(floatingBaseFrame.model);
        }

        if (!inverseVelocityKinematics.updateTarget(
                linkName, linkTwist, linVelTargetWeight, angVelTargetWeight)) {
            yError() << LogPrefix << "Failed to update velocity target for link" << linkName;
            return false;
        }
    }

    return true;
}

bool HumanStateProvider::impl::addInverseVelocityKinematicsTargets()
{
    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        std::string linkName = humanModel.getLinkName(linkIndex);

        // skip the fake links
        if (wearableStorage.modelToWearable_LinkName.find(linkName)
            == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        // Insert in the cost the twist of the link used as base
        if (linkName == floatingBaseFrame.model) {
            if (!useDirectBaseMeasurement
                && !inverseVelocityKinematics.addTarget(
                    linkName, iDynTree::Twist::Zero(), 1.0, 1.0)) {
                yError() << LogPrefix << "Failed to add velocity target for floating base link"
                         << linkName;
                return false;
            }
            continue;
        }

        // Add ivk targets and set to zero
        if (!inverseVelocityKinematics.addAngularVelocityTarget(
                linkName, iDynTree::Twist::Zero(), angVelTargetWeight)) {
            yError() << LogPrefix << "Failed to add velocity target for link" << linkName;
            return false;
        }
    }

    return true;
}

bool HumanStateProvider::impl::computeLinksOrientationErrors(
    std::unordered_map<std::string, iDynTree::Transform> linkDesiredTransforms,
    iDynTree::VectorDynSize jointConfigurations,
    iDynTree::Transform floatingBasePose,
    std::unordered_map<std::string, iDynTreeHelper::Rotation::rotationDistance>&
        linkErrorOrientations)
{
    iDynTree::VectorDynSize zeroJointVelocities = jointConfigurations;
    zeroJointVelocities.zero();

    iDynTree::Twist zeroBaseVelocity;
    zeroBaseVelocity.zero();

    iDynTree::KinDynComputations* computations = kinDynComputations.get();
    computations->setRobotState(
        floatingBasePose, jointConfigurations, zeroBaseVelocity, zeroJointVelocities, worldGravity);

    for (const auto& linkMapEntry : linkDesiredTransforms) {
        const ModelLinkName& linkName = linkMapEntry.first;
        linkErrorOrientations[linkName] = iDynTreeHelper::Rotation::rotationDistance(
            computations->getWorldTransform(linkName).getRotation(),
            linkDesiredTransforms[linkName].getRotation());
    }
    return true;
}

bool HumanStateProvider::impl::computeLinksAngularVelocityErrors(
    std::unordered_map<std::string, iDynTree::Twist> linkDesiredVelocities,
    iDynTree::VectorDynSize jointConfigurations,
    iDynTree::Transform floatingBasePose,
    iDynTree::VectorDynSize jointVelocities,
    iDynTree::Twist baseVelocity,
    std::unordered_map<std::string, iDynTree::Vector3>& linkAngularVelocityError)
{
    iDynTree::KinDynComputations* computations = kinDynComputations.get();
    computations->setRobotState(
        floatingBasePose, jointConfigurations, baseVelocity, jointVelocities, worldGravity);

    for (const auto& linkMapEntry : linkDesiredVelocities) {
        const ModelLinkName& linkName = linkMapEntry.first;
        iDynTree::toEigen(linkAngularVelocityError[linkName]) =
            iDynTree::toEigen(linkDesiredVelocities[linkName].getLinearVec3())
            - iDynTree::toEigen(computations->getFrameVel(linkName).getLinearVec3());
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
            // yWarning() << LogPrefix << "Failed to find" << modelLinkName
            //           << "entry in the configuration map. Skipping this link.";
            continue;
        }

        // Get the name of the sensor associated to the link
        std::string wearableLinkName =
            pImpl->wearableStorage.modelToWearable_LinkName.at(modelLinkName);

        // Try to get the sensor
        auto sensor = pImpl->iWear->getVirtualLinkKinSensor(wearableLinkName);
        if (!sensor) {
            // yError() << LogPrefix << "Failed to find sensor associated to link" <<
            // wearableLinkName
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

    // ====================
    // CHECK ACCELEROMETERS
    // ====================

    if (pImpl->useFBAccelerationFromWearableData) {


        // Check that the attached IWear interface contains all the accelerometer sensors
        for (size_t accelerometerIndex = 0; accelerometerIndex < pImpl->humanSensors.getNrOfSensors(iDynTree::ACCELEROMETER); accelerometerIndex++) {

            // Get acceleration sensor
            iDynTree::AccelerometerSensor *accelerometerSenor = static_cast<iDynTree::AccelerometerSensor*>(pImpl->humanSensors.getSensor(iDynTree::ACCELEROMETER, accelerometerIndex));

            // Get model accelerometer sensor parent link name
            std::string accelerometerModelParentLinkName = accelerometerSenor->getParentLink();

            if (pImpl->wearableStorage.modelToWearable_AccelerometerParentLinkName.find(accelerometerModelParentLinkName)
                == pImpl->wearableStorage.modelToWearable_AccelerometerParentLinkName.end()) {
                yWarning() << LogPrefix << "Failed to find" << accelerometerModelParentLinkName
                           << "entry in the AccelerometerParentLinkName map. Skipping this link.";
                continue;
            }

            // Get the name of the accleration sensor associated to the link
            std::string accelerometerWearableParentLinkName =
                pImpl->wearableStorage.modelToWearable_AccelerometerParentLinkName.at(accelerometerModelParentLinkName);

            // Try to get the acceleration sensor
            auto accelerationSensor = pImpl->iWear->getFreeBodyAccelerationSensor(accelerometerWearableParentLinkName);
            if (!accelerationSensor) {
                // yError() << LogPrefix << "Failed to find acceleration sensor associated to link" <<
                // accelerometerWearableParentLinkName
                //<< "from the IWear interface";
                return false;
            }

            // Create acceleration sensor map entry using the wearable sensor name as key
            pImpl->wearableStorage.accelerometerSensorsMap[accelerometerWearableParentLinkName] =
                    pImpl->iWear->getFreeBodyAccelerationSensor(accelerometerWearableParentLinkName);

            // Get the name of the orientation sensor associated to the link
            // NOTE: In the configuration a check has been put to ensure that the acceleration and orientation sensor parent link name from the configuration
            // are similar. So, using the accelerometerModelParentLinkName directly to access sensor name in modelToWearable_SensorOrientationParentLinkName map
            std::string orientationWearableParentLinkName =
                    pImpl->wearableStorage.modelToWearable_SensorOrientationParentLinkName.at(accelerometerModelParentLinkName);

            // Try to get the orientation sensor
            auto orientationSensor = pImpl->iWear->getOrientationSensor(orientationWearableParentLinkName);
            if (!orientationSensor) {
                // yError() << LogPrefix << "Failed to find orientation sensor associated to link" <<
                // orientationWearableParentLinkName
                //<< "from the IWear interface";
                return false;
            }

            // Create orientation sensor map entry using wearable sensor name as key
            pImpl->wearableStorage.orientationSensorsMap[orientationWearableParentLinkName] =
                    pImpl->iWear->getOrientationSensor(orientationWearableParentLinkName);

        }

    }


    // ============
    // CHECK JOINTS
    // ============

    if (pImpl->useXsensJointsAngles) {
        yDebug() << "Checking joints";

        for (size_t jointIndex = 0; jointIndex < pImpl->humanModel.getNrOfDOFs(); ++jointIndex) {
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
    if (!pImpl->ikPool->closeIKWorkerPool()) {
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
        if (pImpl->humanModel.getJoint(jointIndex)->getNrOfDOFs() == 1) {
            jointNames.emplace_back(pImpl->humanModel.getJointName(jointIndex));
        }
    }

    return jointNames;
}

size_t HumanStateProvider::getNumberOfJoints() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->humanModel.getNrOfDOFs();
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
std::array<double, 3> HumanStateProvider::getCoMPosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.CoMPosition;
}

std::array<double, 3> HumanStateProvider::getCoMVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.CoMVelocity;
}

std::array<double, 3> HumanStateProvider::getCoMBiasAcceleration() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.CoMBiasAcceleration;
}

std::array<double, 3> HumanStateProvider::getCoMProperAcceleration() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->CoMProperAcceleration;
}

std::vector<std::string> HumanStateProvider::getAccelerometerNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->humanSensorData.accelerometerSensorNames;
}

std::vector<std::array<double, 6>> HumanStateProvider::getProperAccelerations() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->humanSensorData.accelerometerSensorMeasurements;
}

std::vector<std::array<double, 3>> HumanStateProvider::getProperLinAccelerations() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);

    std::vector<std::array<double, 3>> properLinAccelerations;
    properLinAccelerations.resize(pImpl->humanSensorData.accelerometerSensorMeasurements.size());

    for (size_t a = 0; a < pImpl->humanSensorData.accelerometerSensorMeasurements.size(); a++) {

        properLinAccelerations.at(a) = {pImpl->humanSensorData.accelerometerSensorMeasurements.at(a)[0],
                                        pImpl->humanSensorData.accelerometerSensorMeasurements.at(a)[1],
                                        pImpl->humanSensorData.accelerometerSensorMeasurements.at(a)[2]};
    }

    return properLinAccelerations;
}

std::vector<std::array<double, 3>> HumanStateProvider::getProperAngAccelerations() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);

    std::vector<std::array<double, 3>> properAngAccelerations;
    properAngAccelerations.resize(pImpl->humanSensorData.accelerometerSensorMeasurements.size());

    for (size_t a = 0; a < pImpl->humanSensorData.accelerometerSensorMeasurements.size(); a++) {

        properAngAccelerations.at(a) = {pImpl->humanSensorData.accelerometerSensorMeasurements.at(a)[3],
                                        pImpl->humanSensorData.accelerometerSensorMeasurements.at(a)[4],
                                        pImpl->humanSensorData.accelerometerSensorMeasurements.at(a)[5]};
    }

    return properAngAccelerations;
}

// This method returns the all link pair names from the full human model
static void createEndEffectorsPairs(
    const iDynTree::Model& model,
    std::vector<SegmentInfo>& humanSegments,
    std::vector<std::pair<std::string, std::string>>& framePairs,
    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>>& framePairIndeces)
{
    // for each element in human segments
    // extract it from the vector (to avoid duplications)
    // Look for it in the model and get neighbours
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

        // this for loop should not be necessary, but this can help keeps the backtrace short
        // as we do not assume that we can go back further that this node
        for (unsigned neighbourIndex = 0; neighbourIndex < model.getNrOfNeighbors(linkIndex);
             ++neighbourIndex) {
            // remember the "biforcations"
            std::vector<iDynTree::LinkIndex> backtrace;
            std::vector<iDynTree::LinkIndex>::iterator Iterator_backtrace;

            // and the visited nodes
            std::vector<iDynTree::LinkIndex> visited;

            // I've already visited the starting node
            visited.push_back(linkIndex);
            iDynTree::Neighbor neighbour = model.getNeighbor(linkIndex, neighbourIndex);
            backtrace.push_back(neighbour.neighborLink);

            while (!backtrace.empty()) {
                iDynTree::LinkIndex currentLink = backtrace.back();
                backtrace.pop_back();
                // add the current link to the visited
                visited.push_back(currentLink);

                std::string linkName = model.getLinkName(currentLink);

                // check if this is a human segment
                std::vector<SegmentInfo>::iterator foundSegment =
                    std::find_if(segments.begin(), segments.end(), [&](SegmentInfo& frame) {
                        return frame.segmentName == linkName;
                    });
                if (foundSegment != segments.end()) {
                    std::vector<SegmentInfo>::difference_type foundLinkIndex =
                        std::distance(segments.begin(), foundSegment);
                    // Found! This is a segment
                    framePairs.push_back(
                        std::pair<std::string, std::string>(segment.segmentName, linkName));
                    framePairIndeces.push_back(
                        std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>(segmentCount,
                                                                              foundLinkIndex));
                    yInfo() << "Segment : " << segment.segmentName
                            << " , associated neighbor : " << linkName
                            << " , found segment: " << foundSegment->segmentName
                            << " , distance: " << foundLinkIndex;
                    break;
                }

                for (unsigned i = 0; i < model.getNrOfNeighbors(currentLink); ++i) {
                    iDynTree::LinkIndex link = model.getNeighbor(currentLink, i).neighborLink;
                    // check if we already visited this segment
                    if (std::find(visited.begin(), visited.end(), link) != visited.end()) {
                        // Yes => skip
                        continue;
                    }
                    Iterator_backtrace = backtrace.begin();
                    backtrace.insert(Iterator_backtrace, link);
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

    if (parentFrameIndex == iDynTree::FRAME_INVALID_INDEX) {
        yError() << LogPrefix << " Invalid parent frame: " << parentFrame;
        return false;
    }
    else if (endEffectorFrameIndex == iDynTree::FRAME_INVALID_INDEX) {
        yError() << LogPrefix << " Invalid End Effector Frame: " << endEffectorFrame;
        return false;
    }

    // Select joint from traversal
    modelInput.computeFullTreeTraversal(traversal, modelInput.getFrameLink(parentFrameIndex));

    iDynTree::LinkIndex visitedLink = modelInput.getFrameLink(endEffectorFrameIndex);

    while (visitedLink != traversal.getBaseLink()->getIndex()) {
        parentLinkIdx = traversal.getParentLinkFromLinkIndex(visitedLink)->getIndex();
        joint = traversal.getParentJointFromLinkIndex(visitedLink);

        // Check if the joint is supported
        if (modelInput.getJoint(joint->getIndex())->getNrOfDOFs() == 1) {
            consideredJoints.insert(consideredJoints.begin(),
                                    modelInput.getJointName(joint->getIndex()));
        }
        else {
            yWarning() << LogPrefix << "Joint " << modelInput.getJointName(joint->getIndex())
                       << " is ignored as it has ("
                       << modelInput.getJoint(joint->getIndex())->getNrOfDOFs() << " DOFs)";
        }

        visitedLink = parentLinkIdx;
    }

    if (!loader.loadReducedModelFromFullModel(modelInput, consideredJoints)) {
        std::cerr << LogPrefix << " failed to select joints: ";
        for (std::vector<std::string>::const_iterator i = consideredJoints.begin();
             i != consideredJoints.end();
             ++i) {
            std::cerr << *i << ' ';
        }
        std::cerr << std::endl;
        return false;
    }

    modelOutput = loader.model();

    return true;
}
