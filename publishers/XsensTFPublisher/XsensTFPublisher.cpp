// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "XsensTFPublisher.h"
#include "Wearable/IWear/IWear.h"

#include <iDynTree/Core/Transform.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>

#include <array>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

const std::string DeviceName = "XsensTFPublisher";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::publishers;

struct XsensTFResources
{
    yarp::dev::PolyDriver transformClientDevice;
    yarp::dev::IFrameTransform* iFrameTransform;
};

class XsensTFPublisher::impl
{
public:
    using LinkSensor = wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>;

    wearable::IWear* iWear = nullptr;

    std::string tfPrefix;

    std::string baseLinkName;
    std::string baseLinkVirtualSensor;

    yarp::sig::Matrix linkTransformBuffer = {4, 4};
    yarp::sig::Matrix world_H_base = {4, 4};

    std::unordered_map<wearable::sensor::SensorName, LinkSensor> linkSensorsMap;
    XsensTFResources xsensTfResources;

    yarp::os::Node node = {"/" + DeviceName};
};

XsensTFPublisher::XsensTFPublisher()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

XsensTFPublisher::~XsensTFPublisher() = default;

bool XsensTFPublisher::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    bool useDefaultPeriod = false;
    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period: " << DefaultPeriod << "s";
        useDefaultPeriod = true;
    }

    if (!(config.check("baseLinkName") && config.find("baseLinkName").isString())) {
        yError() << LogPrefix << "Parameter 'baseLinkName' missing or invalid";
        return false;
    }

    if (!(config.check("baseLinkVirtualSensor")
          && config.find("baseLinkVirtualSensor").isString())) {
        yError() << LogPrefix << "Parameter 'baseLinkVirtualSensor' missing or invalid";
        return false;
    }

    if (!(config.check("tfPrefix") && config.find("tfPrefix").isString())) {
        yError() << LogPrefix << "Parameter 'tfPrefix' missing or invalid";
        return false;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    double period = DefaultPeriod;
    if (!useDefaultPeriod) {
        period = config.find("period").asFloat64();
    }

    pImpl->tfPrefix = config.find("tfPrefix").asString();
    pImpl->baseLinkName = config.find("baseLinkName").asString();
    pImpl->baseLinkVirtualSensor = config.find("baseLinkVirtualSensor").asString();

    yInfo() << LogPrefix << "*** ==========================";
    yInfo() << LogPrefix << "*** TF prefix                :" << pImpl->tfPrefix;
    yInfo() << LogPrefix << "*** Base link name           :" << pImpl->baseLinkName;
    yInfo() << LogPrefix << "*** Base link virtual sensor :" << pImpl->baseLinkVirtualSensor;
    yInfo() << LogPrefix << "*** ==========================";

    // =========================
    // OPEN THE TRANSFORM CLIENT
    // =========================

    yarp::os::Property options;
    options.put("device", "transformClient");
    options.put("local", "/" + DeviceName + "/transformClient");
    options.put("remote", "/transformServer");

    if (!pImpl->xsensTfResources.transformClientDevice.open(options)) {
        yError() << LogPrefix << "Failed to open the transformClient device";
        return false;
    }

    if (!pImpl->xsensTfResources.transformClientDevice.view(
            pImpl->xsensTfResources.iFrameTransform)) {
        yError() << "The IFrameTransform is not implemented by the opened device";
        return false;
    }

    // =====
    // OTHER
    // =====

    setPeriod(period);
    return true;
}

bool XsensTFPublisher::close()
{
    detach();
    pImpl->node.interrupt();

    return true;
}

// TODO: this might be done only in the initialization phase instead than in the run
std::string splitSensorName(const wearable::sensor::SensorName& name)
{
    std::stringstream ss(name);

    std::string item;
    std::vector<std::string> elems;

    while (std::getline(ss, item, ':')) {
        elems.push_back(item);
    }

    return elems.back();
}

void XsensTFPublisher::run()
{
    // ====================
    // WORLD-BASE TRANSFORM
    // ====================

    // Check that the base link sensor is present
    if (pImpl->linkSensorsMap.find(pImpl->baseLinkVirtualSensor) == pImpl->linkSensorsMap.end()
        || !pImpl->linkSensorsMap[pImpl->baseLinkVirtualSensor]) {
        yError() << LogPrefix << "Failed to find link sensor" << pImpl->baseLinkVirtualSensor
                 << "in the IWear interface";
        close();
        return;
    }

    // Get the pose of the base link sensor
    wearable::Vector3 position;
    wearable::Quaternion orientation;
    if (!pImpl->linkSensorsMap[pImpl->baseLinkVirtualSensor]->getLinkPose(position, orientation)) {
        yError() << LogPrefix << "Failed to get base link pose";
        close();
        return;
    }

    // Convert the data to an homogenous matrix
    yarp::math::Quaternion orientation_yarp;
    orientation_yarp.w() = orientation[0];
    orientation_yarp.x() = orientation[1];
    orientation_yarp.y() = orientation[2];
    orientation_yarp.z() = orientation[3];

    pImpl->world_H_base = orientation_yarp.toRotationMatrix4x4();

    pImpl->world_H_base[0][3] = position[0];
    pImpl->world_H_base[1][3] = position[1];
    pImpl->world_H_base[2][3] = position[2];

    // Send it to the transformServer
    pImpl->xsensTfResources.iFrameTransform->setTransform(
        pImpl->tfPrefix + pImpl->baseLinkName, "ground", pImpl->world_H_base);

    // ================
    // LINKS TRANSFORMS
    // ================

    for (const auto& linkSensorEntry : pImpl->linkSensorsMap) {
        const wearable::sensor::SensorName& sensorName = linkSensorEntry.first;
        const impl::LinkSensor linkSensor = linkSensorEntry.second;

        // Strip the prefix from the sensor name
        std::string sensorNameWithoutPrefix = splitSensorName(sensorName);

        // Skip the base link
        if (sensorNameWithoutPrefix == pImpl->baseLinkName) {
            continue;
        }

        if (!linkSensor) {
            yError() << LogPrefix << "Sensor" << sensorName
                     << "is present but it has not been initialized properly";
            askToStop();
            return;
        }

        // Get the pose of the link sensor
        wearable::Vector3 position;
        wearable::Quaternion orientation;
        if (!linkSensor->getLinkPose(position, orientation)) {
            yError() << LogPrefix << "Failed to get pose for link" << linkSensor->getSensorName();
            close();
            return;
        }

        // Convert the data to an homogenous matrix
        yarp::math::Quaternion orientation_yarp;
        orientation_yarp.w() = orientation[0];
        orientation_yarp.x() = orientation[1];
        orientation_yarp.y() = orientation[2];
        orientation_yarp.z() = orientation[3];

        pImpl->linkTransformBuffer = orientation_yarp.toRotationMatrix4x4();

        pImpl->linkTransformBuffer[0][3] = position[0];
        pImpl->linkTransformBuffer[1][3] = position[1];
        pImpl->linkTransformBuffer[2][3] = position[2];

        // Calculate the transform from the link to the base
        pImpl->linkTransformBuffer =
            yarp::math::SE3inv(pImpl->world_H_base) * pImpl->linkTransformBuffer;

        // Send it to the transformServer
        pImpl->xsensTfResources.iFrameTransform->setTransform(pImpl->tfPrefix
                                                                  + sensorNameWithoutPrefix,
                                                              pImpl->tfPrefix + pImpl->baseLinkName,
                                                              pImpl->linkTransformBuffer);
    }
}

bool XsensTFPublisher::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->iWear || !poly->view(pImpl->iWear) || !pImpl->iWear) {
        yError() << LogPrefix << "Failed to view the iWear interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    while (pImpl->iWear->getStatus() == wearable::WearStatus::WaitingForFirstRead) {
        yInfo() << LogPrefix << "IWear interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
    }

    if (pImpl->iWear->getStatus() != wearable::WearStatus::Ok) {
        yError() << LogPrefix << "The status of the attached IWear interface is not ok ("
                 << static_cast<int>(pImpl->iWear->getStatus()) << ")";
        return false;
    }

    // Get the sensors
    auto linkSensors = pImpl->iWear->getVirtualLinkKinSensors();

    if (linkSensors.size() == 0) {
        yError() << LogPrefix
                 << "The IWear interface is Ok but does not contain any virtual link sensor";
        return false;
    }

    // Store the sensors inside a map
    for (const auto& linkSensor : linkSensors) {
        if (linkSensor->getSensorStatus() != wearable::sensor::SensorStatus::Ok) {
            yError() << LogPrefix << "Sensor" << linkSensor->getSensorName() << "status is not Ok ("
                     << static_cast<unsigned>(linkSensor->getSensorStatus()) << ")";
            return false;
        }
        else {
            yInfo() << LogPrefix << "Found sensor" << linkSensor->getSensorName();
            pImpl->linkSensorsMap.emplace(linkSensor->getSensorName(), linkSensor);
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

    return true;
}

bool XsensTFPublisher::detach()
{
    askToStop();
    pImpl->iWear = nullptr;
    return true;
}

bool XsensTFPublisher::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool XsensTFPublisher::detachAll()
{
    return detach();
}
