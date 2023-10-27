// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "WearableTargets_nwc_yarp.h"

#include <hde/msgs/WearableTargets.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <unordered_map>
#include <iostream>

const std::string ClientName = "WearableTargets_nwc_yarp";
const std::string LogPrefix = ClientName + " :";

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
    {hde::msgs::KinematicTargetType::GRAVITY, hde::KinematicTargetType::gravity},
    {hde::msgs::KinematicTargetType::FLOORCONTACT, hde::KinematicTargetType::floorContact},
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

class WearableTargets_nwc_yarp::impl
{
public:
    yarp::os::Network network;
    yarp::os::BufferedPort<hde::msgs::WearableTargets> inputPort;
    bool terminationCall = false;

    mutable std::recursive_mutex mutex;

    // Buffer WearableSensorTarget variables
    std::unordered_map<hde::TargetName, std::shared_ptr<hde::WearableSensorTarget>> wearableTargets;
};

// ========================
// WEARABLETARGETS CLIENT
// ========================

WearableTargets_nwc_yarp::WearableTargets_nwc_yarp()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

WearableTargets_nwc_yarp::~WearableTargets_nwc_yarp() = default;

bool WearableTargets_nwc_yarp::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    if (!(config.check("wearableTargetsDataPort") && config.find("wearableTargetsDataPort").isString())) {
        yError() << LogPrefix << "wearableTargetsDataPort option does not exist or it is not a list";
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
        yError() << LogPrefix << "YARP server wasn't found active.";
        return false;
    }

    // ==========================
    // CONFIGURE INPUT DATA PORTS
    // ==========================
    yDebug() << LogPrefix << "Configuring input data ports";

    pImpl->inputPort.useCallback(*this);
    if (!pImpl->inputPort.open("...")) {
        yError() << LogPrefix << "Failed to open port" << wearableTargetsDataPortName;
        return false;
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << LogPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(wearableTargetsDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << LogPrefix << "Failed to connect " << wearableTargetsDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << LogPrefix << "Opened correctly";
    return true;
}

void WearableTargets_nwc_yarp::threadRelease()
{}

bool WearableTargets_nwc_yarp::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void WearableTargets_nwc_yarp::run()
{
    return;
}

void WearableTargets_nwc_yarp::onRead(hde::msgs::WearableTargets& wearableTargetsData)
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

std::vector<hde::TargetName> WearableTargets_nwc_yarp::getAllTargetsName() const {
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    std::vector<std::string> targetsName;
    for (auto wearableTargetEntry : pImpl->wearableTargets)
    {
        targetsName.push_back(wearableTargetEntry.first);
    }
    return targetsName;
}

std::shared_ptr<hde::WearableSensorTarget> WearableTargets_nwc_yarp::getTarget(const TargetName name) const {
     std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);

    // TODO: what to do if the target name do not exist
    return pImpl->wearableTargets[name];
}
