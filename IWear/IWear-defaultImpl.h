/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IWEAR_DEFAULT_IMPL
#define WEAR_IWEAR_DEFAULT_IMPL

#include "IWear/IWear.h"
#include "IWear/Sensors/ISensor.h"

// =========
// UTILITIES
// =========

template <typename S>
void wearable::IWear::castVectorOfSensorPtr(
    const wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> isensors,
    wearable::VectorOfSensorPtr<const S> sensors)
{
    for (const auto& s : isensors) {
        sensors.push_back(std::dynamic_pointer_cast<const S>(s));
    }
}

// =======
// GENERAL
// =======

// Unqualified sensors retrieving
wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> wearable::IWear::getAllSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> allSensors, tmp;
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::Accelerometer);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::EmgSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::Force3DSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::ForceTorque6DSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::FreeBodyAccelerationSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::Gyroscope);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::Magnetometer);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::OrientationSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::PoseSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::PositionSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::SkinSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::TemperatureSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::Torque3DSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::VirtualLinkKinSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wearable::IWear::getSensors(wearable::sensor::SensorType::VirtualSphericalJointKinSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    return allSensors;
}

// Sensors name list retrieving
wearable::VectorOfSensorNames wearable::IWear::getAllSensorNames() const
{
    const wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> sensors = getAllSensors();
    wearable::VectorOfSensorNames sensorNames;
    sensorNames.reserve(sensors.size());
    for (const auto& s : sensors) {
        sensorNames.push_back(s->getSensorName());
    }
    return sensorNames;
}

// ==============
// SINGLE SENSORS
// ==============

wearable::SensorPtr<const wearable::sensor::IAccelerometer>
wearable::IWear::getAccelerometer(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IEmgSensor>
wearable::IWear::getEmgSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
wearable::IWear::getForce3DSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
wearable::IWear::getForceTorque6DSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
wearable::IWear::getFreeBodyAccelerationSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IGyroscope>
wearable::IWear::getGyroscope(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IMagnetometer>
wearable::IWear::getMagnetometer(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
wearable::IWear::getOrientationSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IPoseSensor>
wearable::IWear::getPoseSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IPositionSensor>
wearable::IWear::getPositionSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ISkinSensor>
wearable::IWear::getSkinSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
wearable::IWear::getTemperatureSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
wearable::IWear::getTorque3DSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
wearable::IWear::getVirtualLinkKinSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
wearable::IWear::getVirtualSphericalJointKinSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

// ================
// MULTIPLE SENSORS
// ================

wearable::VectorOfSensorNames
wearable::IWear::getSensorNames(const wearable::sensor::SensorType type) const
{
    const wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> sensors = getSensors(type);
    wearable::VectorOfSensorNames sensorNames;
    sensorNames.reserve(sensors.size());
    for (const auto& s : sensors) {
        sensorNames.push_back(s->getSensorName());
    }
    return sensorNames;
}

// Qualified sensor retrieving
wearable::VectorOfSensorPtr<const wearable::sensor::IAccelerometer>
wearable::IWear::getAccelerometers() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IAccelerometer> accelerometers;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::Accelerometer), accelerometers);
    return accelerometers;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IEmgSensor>
wearable::IWear::getEmgSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IEmgSensor> emgSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::EmgSensor), emgSensors);
    return emgSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IForce3DSensor>
wearable::IWear::getForce3DSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IForce3DSensor> force3DSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::Force3DSensor), force3DSensors);
    return force3DSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IForceTorque6DSensor>
wearable::IWear::getForceTorque6DSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IForceTorque6DSensor> forceTorque6DSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::ForceTorque6DSensor),
        forceTorque6DSensors);
    return forceTorque6DSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
wearable::IWear::getFreeBodyAccelerationSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
        freeBodyAccelerationSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::FreeBodyAccelerationSensor),
        freeBodyAccelerationSensors);
    return freeBodyAccelerationSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IGyroscope>
wearable::IWear::getGyroscopes() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IGyroscope> gyroscopes;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::Gyroscope), gyroscopes);
    return gyroscopes;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IMagnetometer>
wearable::IWear::getMagnetometers() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IMagnetometer> magnetometers;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::Magnetometer), magnetometers);
    return magnetometers;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IOrientationSensor>
wearable::IWear::getOrientationSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IOrientationSensor> orientationSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::OrientationSensor),
        orientationSensors);
    return orientationSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IPoseSensor>
wearable::IWear::getPoseSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IPoseSensor> poseSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::PoseSensor), poseSensors);
    return poseSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IPositionSensor>
wearable::IWear::getPositionSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IPositionSensor> positionSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::PositionSensor), positionSensors);
    return positionSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::ISkinSensor>
wearable::IWear::getSkinSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISkinSensor> skinSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::SkinSensor), skinSensors);
    return skinSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::ITemperatureSensor>
wearable::IWear::getTemperatureSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ITemperatureSensor> temperatureSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::TemperatureSensor),
        temperatureSensors);
    return temperatureSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::ITorque3DSensor>
wearable::IWear::get3DTorqueSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ITorque3DSensor> torque3DSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::Torque3DSensor), torque3DSensors);
    return torque3DSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
wearable::IWear::getVirtualLinkKinSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
        virtualLinkKinSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::VirtualLinkKinSensor),
        virtualLinkKinSensors);
    return virtualLinkKinSensors;
}

wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
wearable::IWear::getVirtualSphericalJointKinSensors() const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
        virtualSphericalJointKinSensors;
    wearable::IWear::castVectorOfSensorPtr(
        wearable::IWear::getSensors(wearable::sensor::SensorType::VirtualSphericalJointKinSensor),
        virtualSphericalJointKinSensors);
    return virtualSphericalJointKinSensors;
}

#endif // WEAR_IWEAR_DEFAULT_IMPL
