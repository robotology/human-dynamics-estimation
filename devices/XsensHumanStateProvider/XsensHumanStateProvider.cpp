/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "XsensHumanStateProvider.h"

#include <Wearable/IWear/IWear.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <array>
#include <mutex>
#include <string>
#include <unordered_map>

#include <atomic>
#include <chrono>
#include <thread>

const std::string DeviceName = "XsensHumanStateProvider";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;
using namespace wearable;

// ==============
// IMPL AND UTILS
// ==============

using ModelJointName = std::string;
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
struct ModelState
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
    std::unordered_map<ModelJointName, WearableJointInfo> modelToWearable_JointInfo;

    // Maps [wearable virtual sensor name] ==> [virtual sensor]
    std::unordered_map<WearableJointName, SensorPtr<const sensor::IVirtualSphericalJointKinSensor>>
        jointSensorsMap;
};

class XsensHumanStateProvider::impl
{
public:
    // Attached interface
    wearable::IWear* iWear = nullptr;

    mutable std::mutex mutex;

    // Wearable variables
    WearableStorage wearableStorage;

    // Model variables
    iDynTree::Model humanModel;
    FloatingBaseName floatingBaseFrame;

    // Buffers
    iDynTree::VectorDynSize jointAngles;
    std::unordered_map<std::string, iDynTree::Rotation> linkRotationMatrices;
    iDynTree::VectorDynSize jointConfigurationSolution;

    // State in the Model space
    ModelState state;

    bool getJointAnglesFromInputData(iDynTree::VectorDynSize& jointAngles);
};

// =========================
// XsensHumanStateProvider DEVICE
// =========================

XsensHumanStateProvider::XsensHumanStateProvider()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

XsensHumanStateProvider::~XsensHumanStateProvider()
{
    detachAll();
}

bool XsensHumanStateProvider::open(yarp::os::Searchable& config)
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

    if (!(config.check("floatingBaseFrame") && config.find("floatingBaseFrame").isList()
          && config.find("floatingBaseFrame").asList()->size() == 2)) {
        yError() << LogPrefix << "floatingBaseFrame option not found or not valid";
        return false;
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

    const std::string urdfFileName = config.find("urdf").asString();
    pImpl->floatingBaseFrame.model = floatingBaseFrameList->get(0).asString();
    pImpl->floatingBaseFrame.wearable = floatingBaseFrameList->get(1).asString();
    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();

    setPeriod(period);

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

    pImpl->state.jointPositions.resize(nrOfJoints);
    pImpl->state.jointVelocities.resize(nrOfJoints);

    pImpl->jointAngles.resize(nrOfJoints);
    pImpl->jointAngles.zero();

    pImpl->jointConfigurationSolution.resize(nrOfJoints);
    pImpl->jointConfigurationSolution.zero();

    return true;
}

bool XsensHumanStateProvider::close()
{
    return true;
}

void XsensHumanStateProvider::run()
{
        // Get joint angles
        if (!pImpl->getJointAnglesFromInputData(pImpl->jointAngles)) {
            yError() << LogPrefix << "Failed to get joint angles from input data";
            askToStop();
            return;
        }

        // Get the position and orientation of the base link, and convert it into iDynTree objects
        Vector3 position;
        Quaternion orientation;

        if (!pImpl->wearableStorage.baseLinkSensor->getLinkPose(position, orientation)) {
            yError() << LogPrefix << "Failed to get position and orientation of the base"
                     << "from the input data";
            askToStop();
            return;
        }

        iDynTree::Position positioniDynTree(position[0], position[1], position[2]);
        iDynTree::Rotation rotationiDynTree;
        rotationiDynTree.fromQuaternion({orientation.data(), 4});

        iDynTree::Transform baseTransform(std::move(rotationiDynTree), std::move(positioniDynTree));
        
        // Compute the model joint angles and save it in the state
        for (unsigned i = 0; i < pImpl->jointAngles.size(); ++i) {
            pImpl->state.jointPositions[i] = pImpl->jointAngles.getVal(i) * 3.14159 / 180;
        }

        // Save poisition and orientation of the base link
        pImpl->state.basePosition = {baseTransform.getPosition().getVal(0),
                                        baseTransform.getPosition().getVal(1),
                                        baseTransform.getPosition().getVal(2)};

        pImpl->state.baseOrientation = {
            baseTransform.getRotation().asQuaternion().getVal(0),
            baseTransform.getRotation().asQuaternion().getVal(1),
            baseTransform.getRotation().asQuaternion().getVal(2),
            baseTransform.getRotation().asQuaternion().getVal(3),};
}

bool XsensHumanStateProvider::impl::getJointAnglesFromInputData(iDynTree::VectorDynSize& jointAngles)
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
        jointAngles.setVal(humanModel.getJointIndex(modelJointName),
                           anglesXYZ[wearableJointInfo.index]);
    }

    return true;
}

bool XsensHumanStateProvider::attach(yarp::dev::PolyDriver* poly)
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

    pImpl->wearableStorage.baseLinkSensor =
        pImpl->iWear->getVirtualLinkKinSensor(pImpl->floatingBaseFrame.wearable);

    // ============
    // CHECK JOINTS
    // ============

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

bool XsensHumanStateProvider::detach()
{
    askToStop();

    {
        std::lock_guard<std::mutex>(pImpl->mutex);
        pImpl->state.clear();
    }

    pImpl->iWear = nullptr;
    return true;
}

bool XsensHumanStateProvider::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool XsensHumanStateProvider::detachAll()
{
    return detach();
}

std::vector<std::string> XsensHumanStateProvider::getJointNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    std::vector<std::string> jointNames;

    for (size_t jointIndex = 0; jointIndex < pImpl->humanModel.getNrOfJoints(); ++jointIndex) {
        jointNames.emplace_back(pImpl->humanModel.getJointName(jointIndex));
    }

    return jointNames;
}

size_t XsensHumanStateProvider::getNumberOfJoints() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->humanModel.getNrOfJoints();
}

std::vector<double> XsensHumanStateProvider::getJointPositions() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->state.jointPositions;
}

std::vector<double> XsensHumanStateProvider::getJointVelocities() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->state.jointVelocities;
}

std::array<double, 6> XsensHumanStateProvider::getBaseVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->state.baseVelocity;
}

std::array<double, 4> XsensHumanStateProvider::getBaseOrientation() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->state.baseOrientation;
}

std::array<double, 3> XsensHumanStateProvider::getBasePosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->state.basePosition;
}
