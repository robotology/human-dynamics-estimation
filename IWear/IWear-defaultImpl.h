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
void wear::IWear::castVectorOfSensorPtr(
    const wear::VectorOfSensorPtr<const wear::sensor::ISensor> isensors,
    wear::VectorOfSensorPtr<const S> sensors)
{
    for (const auto& s : isensors) {
        sensors.push_back(std::dynamic_pointer_cast<const S>(s));
    }
}

// =======
// GENERAL
// =======

// Unqualified sensors retrieving
wear::VectorOfSensorPtr<const wear::sensor::ISensor> wear::IWear::getAllSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::ISensor> allSensors, tmp;
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::Accelerometer);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::EmgSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::Force3DSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::ForceTorque6DSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::FreeBodyAccelerationSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::Gyroscope);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::Magnetometer);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::OrientationSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::PoseSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::PositionSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::SkinSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::TemperatureSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::Torque3DSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::VirtualLinkKinSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = wear::IWear::getSensors(wear::sensor::SensorType::VirtualSphericalJointKinSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    return allSensors;
}

// Sensors name list retrieving
wear::VectorOfSensorNames wear::IWear::getAllSensorNames() const
{
    const wear::VectorOfSensorPtr<const wear::sensor::ISensor> sensors = getAllSensors();
    wear::VectorOfSensorNames sensorNames;
    sensorNames.reserve(sensors.size());
    for (const auto& s : sensors) {
        sensorNames.push_back(s->getSensorName());
    }
    return sensorNames;
}

// ==============
// SINGLE SENSORS
// ==============

wear::SensorPtr<const wear::sensor::IAccelerometer>
wear::IWear::getAccelerometer(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IEmgSensor>
wear::IWear::getEmgSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IForce3DSensor>
wear::IWear::getForce3DSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IForceTorque6DSensor>
wear::IWear::getForceTorque6DSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IFreeBodyAccelerationSensor>
wear::IWear::getFreeBodyAccelerationSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IGyroscope>
wear::IWear::getGyroscope(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IMagnetometer>
wear::IWear::getMagnetometer(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IOrientationSensor>
wear::IWear::getOrientationSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IPoseSensor>
wear::IWear::getPoseSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IPositionSensor>
wear::IWear::getPositionSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::ISkinSensor>
wear::IWear::getSkinSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::ITemperatureSensor>
wear::IWear::getTemperatureSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::ITorque3DSensor>
wear::IWear::getTorque3DSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IVirtualLinkKinSensor>
wear::IWear::getVirtualLinkKinSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wear::SensorPtr<const wear::sensor::IVirtualSphericalJointKinSensor>
wear::IWear::getVirtualSphericalJointKinSensor(const wear::sensor::SensorName /*name*/) const
{
    return nullptr;
}

// ================
// MULTIPLE SENSORS
// ================

wear::VectorOfSensorNames wear::IWear::getSensorNames(const wear::sensor::SensorType type) const
{
    const wear::VectorOfSensorPtr<const wear::sensor::ISensor> sensors = getSensors(type);
    wear::VectorOfSensorNames sensorNames;
    sensorNames.reserve(sensors.size());
    for (const auto& s : sensors) {
        sensorNames.push_back(s->getSensorName());
    }
    return sensorNames;
}

// Qualified sensor retrieving
wear::VectorOfSensorPtr<const wear::sensor::IAccelerometer> wear::IWear::getAccelerometers() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IAccelerometer> accelerometers;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::Accelerometer), accelerometers);
    return accelerometers;
}

wear::VectorOfSensorPtr<const wear::sensor::IEmgSensor> wear::IWear::getEmgSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IEmgSensor> emgSensors;
    wear::IWear::castVectorOfSensorPtr(wear::IWear::getSensors(wear::sensor::SensorType::EmgSensor),
                                       emgSensors);
    return emgSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::IForce3DSensor> wear::IWear::getForce3DSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IForce3DSensor> force3DSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::Force3DSensor), force3DSensors);
    return force3DSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::IForceTorque6DSensor>
wear::IWear::getForceTorque6DSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IForceTorque6DSensor> forceTorque6DSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::ForceTorque6DSensor),
        forceTorque6DSensors);
    return forceTorque6DSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::IFreeBodyAccelerationSensor>
wear::IWear::getFreeBodyAccelerationSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IFreeBodyAccelerationSensor>
        freeBodyAccelerationSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::FreeBodyAccelerationSensor),
        freeBodyAccelerationSensors);
    return freeBodyAccelerationSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::IGyroscope> wear::IWear::getGyroscopes() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IGyroscope> gyroscopes;
    wear::IWear::castVectorOfSensorPtr(wear::IWear::getSensors(wear::sensor::SensorType::Gyroscope),
                                       gyroscopes);
    return gyroscopes;
}

wear::VectorOfSensorPtr<const wear::sensor::IMagnetometer> wear::IWear::getMagnetometers() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IMagnetometer> magnetometers;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::Magnetometer), magnetometers);
    return magnetometers;
}

wear::VectorOfSensorPtr<const wear::sensor::IOrientationSensor>
wear::IWear::getOrientationSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IOrientationSensor> orientationSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::OrientationSensor), orientationSensors);
    return orientationSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::IPoseSensor> wear::IWear::getPoseSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IPoseSensor> poseSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::PoseSensor), poseSensors);
    return poseSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::IPositionSensor> wear::IWear::getPositionSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IPositionSensor> positionSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::PositionSensor), positionSensors);
    return positionSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::ISkinSensor> wear::IWear::getSkinSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::ISkinSensor> skinSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::SkinSensor), skinSensors);
    return skinSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::ITemperatureSensor>
wear::IWear::getTemperatureSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::ITemperatureSensor> temperatureSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::TemperatureSensor), temperatureSensors);
    return temperatureSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::ITorque3DSensor> wear::IWear::get3DTorqueSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::ITorque3DSensor> torque3DSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::Torque3DSensor), torque3DSensors);
    return torque3DSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::IVirtualLinkKinSensor>
wear::IWear::getVirtualLinkKinSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IVirtualLinkKinSensor> virtualLinkKinSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::VirtualLinkKinSensor),
        virtualLinkKinSensors);
    return virtualLinkKinSensors;
}

wear::VectorOfSensorPtr<const wear::sensor::IVirtualSphericalJointKinSensor>
wear::IWear::getVirtualSphericalJointKinSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::IVirtualSphericalJointKinSensor>
        virtualSphericalJointKinSensors;
    wear::IWear::castVectorOfSensorPtr(
        wear::IWear::getSensors(wear::sensor::SensorType::VirtualSphericalJointKinSensor),
        virtualSphericalJointKinSensors);
    return virtualSphericalJointKinSensors;
}

#endif // WEAR_IWEAR_DEFAULT_IMPL
