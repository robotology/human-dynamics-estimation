/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WearableTargetsRemapper.h"

#include <hde/msgs/WearableTargets.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <unordered_map>
#include <iostream>

const std::string RemapperName = "WearableTargetsRemapper";
const std::string logPrefix = RemapperName + " :";

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

const std::map<hde::msgs::KinematicTargetType, hde::KinematicTargetType> mapKinematicTargetType = {
    {hde::msgs::KinematicTargetType::NONE, hde::KinematicTargetType::none},
    {hde::msgs::KinematicTargetType::POSE, hde::KinematicTargetType::pose},
    {hde::msgs::KinematicTargetType::POSEANDVELOCITY, hde::KinematicTargetType::poseAndVelocity},
    {hde::msgs::KinematicTargetType::POSITION, hde::KinematicTargetType::position},
    {hde::msgs::KinematicTargetType::POSITIONANDVELOCITY, hde::KinematicTargetType::positionAndVelocity},
    {hde::msgs::KinematicTargetType::ORIENTATION, hde::KinematicTargetType::orientation},
    {hde::msgs::KinematicTargetType::ORIENTATIONANDVELOCITY, hde::KinematicTargetType::orientationAndVelocity},
};

iDynTree::Vector3 generateVector3FromMsg(const hde::msgs::Vector3& input)
{
    iDynTree::Vector3 vector3;
    vector3.setVal(0, input.x);
    vector3.setVal(1, input.y);
    vector3.setVal(2, input.z);
    return vector3;
}

iDynTree::Rotation generateRotationFromMsg(const hde::msgs::Quaternion& input)
{
    iDynTree::Vector4 vector4;
    vector4.setVal(0, input.w);
    vector4.setVal(1, input.imaginary.x);
    vector4.setVal(2, input.imaginary.y);
    vector4.setVal(3, input.imaginary.z);
    return iDynTree::Rotation::RotationFromQuaternion(vector4);
}

iDynTree::Transform generateTransformFromMsg(const hde::msgs::Transform& input)
{
    return iDynTree::Transform(generateRotationFromMsg(input.orientation), iDynTree::Position(generateVector3FromMsg(input.position)));
}

class WearableTargetsRemapper::impl
{
public:
    yarp::os::Network network;
    yarp::os::BufferedPort<hde::msgs::WearableTargets> inputPort;
    bool terminationCall = false;

    mutable std::recursive_mutex mutex;

    // Buffer WearableSensorTarget variables
    std::unordered_map<hde::TargetName, std::shared_ptr<hde::WearableSensorTarget>> wearableTargets;
};

// ==============
// IWEAR REMAPPER
// ==============

WearableTargetsRemapper::WearableTargetsRemapper()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

WearableTargetsRemapper::~WearableTargetsRemapper() = default;

bool WearableTargetsRemapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    if (!(config.check("wearableTargetsDataPort") && config.find("wearableTargetsDataPort").isString())) {
        yError() << logPrefix << "wearableTargetsDataPort option does not exist or it is not a list";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    std::string wearableTargetsDataPortName = config.find("wearableTargetsDataPort").asString();

    // Initialize the network
    // TODO: is this required in every DeviceDriver?
    pImpl->network = yarp::os::Network();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << logPrefix << "YARP server wasn't found active.";
        return false;
    }

    // ==========================
    // CONFIGURE INPUT DATA PORTS
    // ==========================
    yDebug() << logPrefix << "Configuring input data ports";

    pImpl->inputPort.useCallback(*this);
    if (!pImpl->inputPort.open("...")) {
        yError() << logPrefix << "Failed to open port" << wearableTargetsDataPortName;
        return false;
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << logPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(wearableTargetsDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << logPrefix << "Failed to connect " << wearableTargetsDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << logPrefix << "Opened correctly";
    return true;
}

void WearableTargetsRemapper::threadRelease()
{}

bool WearableTargetsRemapper::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void WearableTargetsRemapper::run()
{
    return;
}

void WearableTargetsRemapper::onRead(hde::msgs::WearableTargets& wearableTargetsData)
{
    if(!pImpl->terminationCall) {

        for (auto wearableTarget : wearableTargetsData.targets)
        {
            if (pImpl->wearableTargets.find(wearableTarget.first) == pImpl->wearableTargets.end())
            {
                pImpl->wearableTargets[wearableTarget.first] = std::make_shared<hde::WearableSensorTarget>(wearableTarget.second.wearableSensorName,
                                                                                                           wearableTarget.second.linkName,
                                                                                                           mapKinematicTargetType.at(wearableTarget.second.type));
            }

            // TODO: this can be done more efficiently considering different target type have incomplete information
            pImpl->wearableTargets[wearableTarget.first].get()->position = generateVector3FromMsg(wearableTarget.second.position);
            pImpl->wearableTargets[wearableTarget.first].get()->rotation = generateRotationFromMsg(wearableTarget.second.orientation);
            pImpl->wearableTargets[wearableTarget.first].get()->linearVelocity = generateVector3FromMsg(wearableTarget.second.linearVelocity);
            pImpl->wearableTargets[wearableTarget.first].get()->angularVelocity = generateVector3FromMsg(wearableTarget.second.angularVelocity);
            pImpl->wearableTargets[wearableTarget.first].get()->calibrationWorldToMeasurementWorld = generateTransformFromMsg(wearableTarget.second.calibrationWorldToMeasurementWorld);
            pImpl->wearableTargets[wearableTarget.first].get()->calibrationMeasurementToLink = generateTransformFromMsg(wearableTarget.second.calibrationMeasurementToLink);
            pImpl->wearableTargets[wearableTarget.first].get()->positionScaleFactor = generateVector3FromMsg(wearableTarget.second.positionScaleFactor);
        }
    }
}

std::vector<hde::TargetName> WearableTargetsRemapper::getAllTargetsName() const {
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    std::vector<std::string> targetsName;
    for (auto wearableTargetEntry : pImpl->wearableTargets)
    {
        targetsName.push_back(wearableTargetEntry.first);
    }
    return targetsName;
}

std::shared_ptr<hde::WearableSensorTarget> WearableTargetsRemapper::getTarget(const TargetName name) const {
     std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);

    // TODO: what to do if the target name do not exist
    return pImpl->wearableTargets[name];
}
