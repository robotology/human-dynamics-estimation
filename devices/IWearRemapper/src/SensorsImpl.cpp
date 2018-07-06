/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "SensorsImpl.h"

// =============
// Accelerometer
// =============

sensorImpl::Accelerometer::Accelerometer(wearable::sensor::SensorName n,
                                         wearable::sensor::SensorStatus s)
    : wearable::sensor::IAccelerometer(n, s)
{}

bool sensorImpl::Accelerometer::getLinearAcceleration(wearable::Vector3& linearAcceleration) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    linearAcceleration = m_buffer;
    return true;
}

void sensorImpl::Accelerometer::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// =========
// EmgSensor
// =========

sensorImpl::EmgSensor::EmgSensor(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IEmgSensor(n, s)
{}

bool sensorImpl::EmgSensor::getEmgSignal(double& emgSignal) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    emgSignal = m_value;
    return true;
}

bool sensorImpl::EmgSensor::getNormalizationValue(double& normalizationValue) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    normalizationValue = m_normalization;
    return true;
}

void sensorImpl::EmgSensor::setBuffer(const double value, const double normalization)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_value = value;
    m_normalization = normalization;
}

// =============
// Force3DSensor
// =============

sensorImpl::Force3DSensor::Force3DSensor(wearable::sensor::SensorName n,
                                         wearable::sensor::SensorStatus s)
    : wearable::sensor::IForce3DSensor(n, s)
{}

bool sensorImpl::Force3DSensor::getForce3D(wearable::Vector3& force) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    force = m_buffer;
    return true;
}

void sensorImpl::Force3DSensor::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ===================
// ForceTorque6DSensor
// ===================

sensorImpl::ForceTorque6DSensor::ForceTorque6DSensor(wearable::sensor::SensorName n,
                                                     wearable::sensor::SensorStatus s)
    : wearable::sensor::IForceTorque6DSensor(n, s)
{}

bool sensorImpl::ForceTorque6DSensor::getForceTorque6D(wearable::Vector3& force3D,
                                                       wearable::Vector3& torque3D) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    force3D = m_force;
    torque3D = m_torque;
    return true;
}

void sensorImpl::ForceTorque6DSensor::setBuffer(const wearable::Vector3& force,
                                                const wearable::Vector3& torque)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_force = force;
    m_torque = torque;
}

// ==========================
// FreeBodyAccelerationSensor
// ==========================

sensorImpl::FreeBodyAccelerationSensor::FreeBodyAccelerationSensor(wearable::sensor::SensorName n,
                                                                   wearable::sensor::SensorStatus s)
    : wearable::sensor::IFreeBodyAccelerationSensor(n, s)
{}

bool sensorImpl::FreeBodyAccelerationSensor::getFreeBodyAcceleration(
    wearable::Vector3& freeBodyAcceleration) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    freeBodyAcceleration = m_buffer;
    return true;
}

void sensorImpl::FreeBodyAccelerationSensor::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// =========
// Gyroscope
// =========

sensorImpl::Gyroscope::Gyroscope(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IGyroscope(n, s)
{}

bool sensorImpl::Gyroscope::getAngularRate(wearable::Vector3& angularRate) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    angularRate = m_buffer;
    return true;
}

void sensorImpl::Gyroscope::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ============
// Magnetometer
// ============

sensorImpl::Magnetometer::Magnetometer(wearable::sensor::SensorName n,
                                       wearable::sensor::SensorStatus s)
    : wearable::sensor::IMagnetometer(n, s)
{}

bool sensorImpl::Magnetometer::getMagneticField(wearable::Vector3& magneticField) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    magneticField = m_buffer;
    return true;
}

void sensorImpl::Magnetometer::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// =================
// OrientationSensor
// =================

sensorImpl::OrientationSensor::OrientationSensor(wearable::sensor::SensorName n,
                                                 wearable::sensor::SensorStatus s)
    : wearable::sensor::IOrientationSensor(n, s)
{}

bool sensorImpl::OrientationSensor::getOrientationAsQuaternion(
    wearable::Quaternion& orientation) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    orientation = m_buffer;
    return true;
}

void sensorImpl::OrientationSensor::setBuffer(const wearable::Quaternion& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ==========
// PoseSensor
// ==========

sensorImpl::PoseSensor::PoseSensor(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::IPoseSensor(n, s)
{}

bool sensorImpl::PoseSensor::getPose(wearable::Quaternion& orientation,
                                     wearable::Vector3& position) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    orientation = m_orientation;
    position = m_position;
    return true;
}

void sensorImpl::PoseSensor::setBuffer(const wearable::Quaternion& orientation,
                                       const wearable::Vector3& position)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_orientation = orientation;
    m_position = position;
}

// ==============
// PositionSensor
// ==============

sensorImpl::PositionSensor::PositionSensor(wearable::sensor::SensorName n,
                                           wearable::sensor::SensorStatus s)
    : wearable::sensor::IPositionSensor(n, s)
{}

bool sensorImpl::PositionSensor::getPosition(wearable::Vector3& position) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    position = m_buffer;
    return true;
}

void sensorImpl::PositionSensor::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ==========
// SkinSensor
// ==========

sensorImpl::SkinSensor::SkinSensor(wearable::sensor::SensorName n, wearable::sensor::SensorStatus s)
    : wearable::sensor::ISkinSensor(n, s)
{}

// =================
// TemperatureSensor
// =================

sensorImpl::TemperatureSensor::TemperatureSensor(wearable::sensor::SensorName n,
                                                 wearable::sensor::SensorStatus s)
    : wearable::sensor::ITemperatureSensor(n, s)
{}

bool sensorImpl::TemperatureSensor::getTemperature(double& temperature) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    temperature = m_value;
    return true;
}

void sensorImpl::TemperatureSensor::setBuffer(const double value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_value = value;
}

// ==============
// Torque3DSensor
// ==============

sensorImpl::Torque3DSensor::Torque3DSensor(wearable::sensor::SensorName n,
                                           wearable::sensor::SensorStatus s)
    : wearable::sensor::ITorque3DSensor(n, s)
{}

bool sensorImpl::Torque3DSensor::getTorque3D(wearable::Vector3& torque) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    torque = m_buffer;
    return true;
}

void sensorImpl::Torque3DSensor::setBuffer(const wearable::Vector3& data)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_buffer = data;
}

// ====================
// VirtualLinkKinSensor
// ====================

sensorImpl::VirtualLinkKinSensor::VirtualLinkKinSensor(wearable::sensor::SensorName n,
                                                       wearable::sensor::SensorStatus s)
    : wearable::sensor::IVirtualLinkKinSensor(n, s)
{}

bool sensorImpl::VirtualLinkKinSensor::getLinkAcceleration(wearable::Vector3& linear,
                                                           wearable::Vector3& angular) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    linear = m_linearAcc;
    angular = m_angularAcc;
    return true;
}

bool sensorImpl::VirtualLinkKinSensor::getLinkPose(wearable::Vector3& position,
                                                   wearable::Quaternion& orientation) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    position = m_position;
    orientation = m_orientation;
    return true;
}

bool sensorImpl::VirtualLinkKinSensor::getLinkVelocity(wearable::Vector3& linear,
                                                       wearable::Vector3& angular) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    linear = m_linearVel;
    angular = m_angularVel;
    return true;
}

void sensorImpl::VirtualLinkKinSensor::setBuffer(const wearable::Vector3& linearAcc,
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
// VirtualSphericalJointKinSensor
// ==============================

sensorImpl::VirtualSphericalJointKinSensor::VirtualSphericalJointKinSensor(
    wearable::sensor::SensorName n,
    wearable::sensor::SensorStatus s)
    : wearable::sensor::IVirtualSphericalJointKinSensor(n, s)
{}

bool sensorImpl::VirtualSphericalJointKinSensor::getJointAnglesAsRPY(
    wearable::Vector3 angleAsRPY) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    angleAsRPY = m_angleAsRPY;
    return true;
}

bool sensorImpl::VirtualSphericalJointKinSensor::getJointVelocities(
    wearable::Vector3 velocities) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    velocities = m_velocities;
    return true;
}

bool sensorImpl::VirtualSphericalJointKinSensor::getJointAccelerations(
    wearable::Vector3 accelerations) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    accelerations = m_accelerations;
    return true;
}

void sensorImpl::VirtualSphericalJointKinSensor::setBuffer(const wearable::Vector3& angleAsRPY,
                                                           const wearable::Vector3& velocities,
                                                           const wearable::Vector3& accelerations)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_angleAsRPY = angleAsRPY;
    m_velocities = velocities;
    m_accelerations = accelerations;
}
