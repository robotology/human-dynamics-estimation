/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "Wearable/IWear/Sensors/impl/SensorsImpl.h"

using namespace wearable::sensor::impl;

// =============
// Accelerometer
// =============

Accelerometer::Accelerometer(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IAccelerometer(n, s)
{}

bool Accelerometer::getLinearAcceleration(wearable::Vector3& linearAcceleration) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    linearAcceleration = m_buffer;
    return true;
}

void Accelerometer::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// =========
// EmgSensor
// =========

EmgSensor::EmgSensor(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IEmgSensor(n, s)
{}

bool EmgSensor::getEmgSignal(double& emgSignal) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    emgSignal = m_value;
    return true;
}

bool EmgSensor::getNormalizationValue(double& normalizationValue) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    normalizationValue = m_normalization;
    return true;
}

void EmgSensor::setBuffer(const double value, const double normalization)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_value = value;
    m_normalization = normalization;
}

// =============
// Force3DSensor
// =============

Force3DSensor::Force3DSensor(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IForce3DSensor(n, s)
{}

bool Force3DSensor::getForce3D(wearable::Vector3& force) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    force = m_buffer;
    return true;
}

void Force3DSensor::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ===================
// ForceTorque6DSensor
// ===================

ForceTorque6DSensor::ForceTorque6DSensor(wearable::sensor::SensorName n,
                                         wearable::sensor::SensorStatus s)
    : wearable::sensor::IForceTorque6DSensor(n, s)
{}

bool ForceTorque6DSensor::getForceTorque6D(wearable::Vector3& force3D,
                                           wearable::Vector3& torque3D) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    force3D = m_force;
    torque3D = m_torque;
    return true;
}

void ForceTorque6DSensor::setBuffer(const wearable::Vector3& force, const wearable::Vector3& torque)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_force = force;
    m_torque = torque;
}

// ==========================
// FreeBodyAccelerationSensor
// ==========================

FreeBodyAccelerationSensor::FreeBodyAccelerationSensor(wearable::sensor::SensorName n,
                                                       wearable::sensor::SensorStatus s)
    : wearable::sensor::IFreeBodyAccelerationSensor(n, s)
{}

bool FreeBodyAccelerationSensor::getFreeBodyAcceleration(
    wearable::Vector3& freeBodyAcceleration) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    freeBodyAcceleration = m_buffer;
    return true;
}

void FreeBodyAccelerationSensor::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// =========
// Gyroscope
// =========

Gyroscope::Gyroscope(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IGyroscope(n, s)
{}

bool Gyroscope::getAngularRate(wearable::Vector3& angularRate) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    angularRate = m_buffer;
    return true;
}

void Gyroscope::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ============
// Magnetometer
// ============

Magnetometer::Magnetometer(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IMagnetometer(n, s)
{}

bool Magnetometer::getMagneticField(wearable::Vector3& magneticField) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    magneticField = m_buffer;
    return true;
}

void Magnetometer::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// =================
// OrientationSensor
// =================

OrientationSensor::OrientationSensor(wearable::sensor::SensorName n,
                                     wearable::sensor::SensorStatus s)
    : wearable::sensor::IOrientationSensor(n, s)
{}

bool OrientationSensor::getOrientationAsQuaternion(wearable::Quaternion& orientation) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    orientation = m_buffer;
    return true;
}

void OrientationSensor::setBuffer(const wearable::Quaternion& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ==========
// PoseSensor
// ==========

PoseSensor::PoseSensor(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IPoseSensor(n, s)
{}

bool PoseSensor::getPose(wearable::Quaternion& orientation, wearable::Vector3& position) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    orientation = m_orientation;
    position = m_position;
    return true;
}

void PoseSensor::setBuffer(const wearable::Quaternion& orientation,
                           const wearable::Vector3& position)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_orientation = orientation;
    m_position = position;
}

// ==============
// PositionSensor
// ==============

PositionSensor::PositionSensor(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IPositionSensor(n, s)
{}

bool PositionSensor::getPosition(wearable::Vector3& position) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    position = m_buffer;
    return true;
}

void PositionSensor::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ==========
// SkinSensor
// ==========

SkinSensor::SkinSensor(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::ISkinSensor(n, s)
{}

bool SkinSensor::getPressure(std::vector<double>& pressure) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    pressure = m_values;
    return true;
}

void SkinSensor::setBuffer(const std::vector<double> values)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_values = values;
}

// =================
// TemperatureSensor
// =================

TemperatureSensor::TemperatureSensor(wearable::sensor::SensorName n,
                                     wearable::sensor::SensorStatus s)
    : wearable::sensor::ITemperatureSensor(n, s)
{}

bool TemperatureSensor::getTemperature(double& temperature) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    temperature = m_value;
    return true;
}

void TemperatureSensor::setBuffer(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_value = value;
}

// ==============
// Torque3DSensor
// ==============

Torque3DSensor::Torque3DSensor(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::ITorque3DSensor(n, s)
{}

bool Torque3DSensor::getTorque3D(wearable::Vector3& torque) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    torque = m_buffer;
    return true;
}

void Torque3DSensor::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ====================
// VirtualLinkKinSensor
// ====================

VirtualLinkKinSensor::VirtualLinkKinSensor(wearable::sensor::SensorName n,
                                           wearable::sensor::SensorStatus s)
    : wearable::sensor::IVirtualLinkKinSensor(n, s)
{}

bool VirtualLinkKinSensor::getLinkAcceleration(wearable::Vector3& linear,
                                               wearable::Vector3& angular) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    linear = m_linearAcc;
    angular = m_angularAcc;
    return true;
}

//#include <iostream>
bool VirtualLinkKinSensor::getLinkPose(wearable::Vector3& position,
                                       wearable::Quaternion& orientation) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    position = m_position;
    orientation = m_orientation;

    //    std::cerr << "sensorImpl" << orientation[0] << orientation[1] << orientation[2]
    //              << orientation[3] << m_name << std::endl;

    return true;
}

bool VirtualLinkKinSensor::getLinkVelocity(wearable::Vector3& linear,
                                           wearable::Vector3& angular) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    linear = m_linearVel;
    angular = m_angularVel;
    return true;
}

void VirtualLinkKinSensor::setBuffer(const wearable::Vector3& linearAcc,
                                     const wearable::Vector3& angularAcc,
                                     const wearable::Vector3& linearVel,
                                     const wearable::Vector3& angularVel,
                                     const wearable::Vector3& position,
                                     const wearable::Quaternion& orientation)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_linearAcc = linearAcc;
    m_angularAcc = angularAcc;
    m_linearVel = linearVel;
    m_angularVel = angularVel;
    m_position = position;
    m_orientation = orientation;
}

// ==============================
// VirtualJointKinSensor
// ==============================

VirtualJointKinSensor::VirtualJointKinSensor(wearable::sensor::SensorName n,
                                             wearable::sensor::SensorStatus s)
    : wearable::sensor::IVirtualJointKinSensor(n, s)
{}

bool VirtualJointKinSensor::getJointPosition(double& position) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    position = m_position;
    return true;
}

bool VirtualJointKinSensor::getJointVelocity(double& velocity) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    velocity = m_velocity;
    return true;
}

bool VirtualJointKinSensor::getJointAcceleration(double& acceleration) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    acceleration = m_acceleration;
    return true;
}

void VirtualJointKinSensor::setBuffer(const double& position,
                                      const double& velocity,
                                      const double& acceleration)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_position = position;
    m_velocity = velocity;
    m_acceleration = acceleration;
}


// ==============================
// VirtualSphericalJointKinSensor
// ==============================

VirtualSphericalJointKinSensor::VirtualSphericalJointKinSensor(wearable::sensor::SensorName n,
                                                               wearable::sensor::SensorStatus s)
    : wearable::sensor::IVirtualSphericalJointKinSensor(n, s)
{}

bool VirtualSphericalJointKinSensor::getJointAnglesAsRPY(wearable::Vector3& angleAsRPY) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    angleAsRPY = m_angleAsRPY;
    return true;
}

bool VirtualSphericalJointKinSensor::getJointVelocities(wearable::Vector3& velocities) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    velocities = m_velocities;
    return true;
}

bool VirtualSphericalJointKinSensor::getJointAccelerations(wearable::Vector3& accelerations) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    accelerations = m_accelerations;
    return true;
}

void VirtualSphericalJointKinSensor::setBuffer(const wearable::Vector3& angleAsRPY,
                                               const wearable::Vector3& velocities,
                                               const wearable::Vector3& accelerations)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_angleAsRPY = angleAsRPY;
    m_velocities = velocities;
    m_accelerations = accelerations;
}
