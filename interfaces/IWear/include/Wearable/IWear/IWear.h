/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IWEAR_H
#define WEARABLE_IWEAR_H

#include "Wearable/IWear/Sensors/ISensor.h"

#include "Wearable/IWear/Sensors/IAccelerometer.h"
#include "Wearable/IWear/Sensors/IEmgSensor.h"
#include "Wearable/IWear/Sensors/IForce3DSensor.h"
#include "Wearable/IWear/Sensors/IForceTorque6DSensor.h"
#include "Wearable/IWear/Sensors/IFreeBodyAccelerationSensor.h"
#include "Wearable/IWear/Sensors/IGyroscope.h"
#include "Wearable/IWear/Sensors/IMagnetometer.h"
#include "Wearable/IWear/Sensors/IOrientationSensor.h"
#include "Wearable/IWear/Sensors/IPoseSensor.h"
#include "Wearable/IWear/Sensors/IPositionSensor.h"
#include "Wearable/IWear/Sensors/ISkinSensor.h"
#include "Wearable/IWear/Sensors/ITemperatureSensor.h"
#include "Wearable/IWear/Sensors/ITorque3DSensor.h"
#include "Wearable/IWear/Sensors/IVirtualLinkKinSensor.h"
#include "Wearable/IWear/Sensors/IVirtualSphericalJointKinSensor.h"

#include <memory>
#include <string>
#include <vector>

namespace wearable {
    using WearableName = std::string;

    template <typename S>
    using SensorVector = std::vector<S>;

    template <typename S>
    using SensorPtr = std::shared_ptr<S>;

    template <typename S>
    using VectorOfSensorPtr = SensorVector<SensorPtr<S>>;

    using WearStatus = sensor::SensorStatus;
    using VectorOfSensorNames = SensorVector<sensor::SensorName>;

    using TimeStamp = struct
    {
        double time = 0;
        size_t sequenceNumber = 0;
    };

    class IWear;
} // namespace wearable

class wearable::IWear
{
private:
    template <typename S>
    static VectorOfSensorPtr<const S>
    castVectorOfSensorPtr(const VectorOfSensorPtr<const sensor::ISensor>& iSensors);

public:
    virtual ~IWear() = default;

    // ===============
    // GENERIC METHODS
    // ===============

    virtual WearableName getWearableName() const = 0;
    virtual WearStatus getStatus() const = 0;
    virtual TimeStamp getTimeStamp() const = 0;

    virtual SensorPtr<const sensor::ISensor> getSensor(const sensor::SensorName name) const = 0;
    virtual VectorOfSensorPtr<const sensor::ISensor>
    getSensors(const sensor::SensorType type) const = 0;

    // ==============
    // SINGLE SENSORS
    // ==============

    virtual SensorPtr<const sensor::IAccelerometer>
    getAccelerometer(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IEmgSensor>
    getEmgSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IForce3DSensor>
    getForce3DSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IGyroscope>
    getGyroscope(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IMagnetometer>
    getMagnetometer(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IOrientationSensor>
    getOrientationSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IPoseSensor>
    getPoseSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IPositionSensor>
    getPositionSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::ISkinSensor>
    getSkinSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::ITemperatureSensor>
    getTemperatureSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::ITorque3DSensor>
    getTorque3DSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const sensor::SensorName /*name*/) const = 0;

    virtual SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const sensor::SensorName /*name*/) const = 0;

    // =================
    // GENERIC UTILITIES
    // =================

    inline VectorOfSensorPtr<const sensor::ISensor> getAllSensors() const;

    inline VectorOfSensorNames getSensorNames(const sensor::SensorType type) const;

    inline VectorOfSensorNames getAllSensorNames() const;

    // ================
    // SENSOR UTILITIES
    // ================

    inline VectorOfSensorPtr<const sensor::IAccelerometer> getAccelerometers() const;

    inline VectorOfSensorPtr<const sensor::IEmgSensor> getEmgSensors() const;

    inline VectorOfSensorPtr<const sensor::IForce3DSensor> getForce3DSensors() const;

    inline VectorOfSensorPtr<const sensor::IForceTorque6DSensor> getForceTorque6DSensors() const;

    inline VectorOfSensorPtr<const sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensors() const;

    inline VectorOfSensorPtr<const sensor::IGyroscope> getGyroscopes() const;

    inline VectorOfSensorPtr<const sensor::IMagnetometer> getMagnetometers() const;

    inline VectorOfSensorPtr<const sensor::IOrientationSensor> getOrientationSensors() const;

    inline VectorOfSensorPtr<const sensor::IPoseSensor> getPoseSensors() const;

    inline VectorOfSensorPtr<const sensor::IPositionSensor> getPositionSensors() const;

    inline VectorOfSensorPtr<const sensor::ISkinSensor> getSkinSensors() const;

    inline VectorOfSensorPtr<const sensor::ITemperatureSensor> getTemperatureSensors() const;

    inline VectorOfSensorPtr<const sensor::ITorque3DSensor> getTorque3DSensors() const;

    inline VectorOfSensorPtr<const sensor::IVirtualLinkKinSensor> getVirtualLinkKinSensors() const;

    inline VectorOfSensorPtr<const sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensors() const;
};

// ============================================
// IMPLEMENTATION OF INLINE AND PRIVATE METHODS
// ============================================

template <typename S>
wearable::VectorOfSensorPtr<const S>
wearable::IWear::castVectorOfSensorPtr(const VectorOfSensorPtr<const sensor::ISensor>& iSensors)
{
    VectorOfSensorPtr<const S> sensors;
    sensors.reserve(iSensors.size());

    for (const auto& iSensor : iSensors) {
        wearable::SensorPtr<const S> castSensor = std::dynamic_pointer_cast<const S>(iSensor);

        if (!castSensor) {
            wError << "Failed to cast sensor";
            return {};
        }

        sensors.push_back(castSensor);
    }

    return sensors;
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
wearable::IWear::getAllSensors() const
{
    const std::vector<sensor::SensorType> allSensorTypes = {
        sensor::SensorType::Accelerometer,
        sensor::SensorType::EmgSensor,
        sensor::SensorType::Force3DSensor,
        sensor::SensorType::ForceTorque6DSensor,
        sensor::SensorType::FreeBodyAccelerationSensor,
        sensor::SensorType::Gyroscope,
        sensor::SensorType::Magnetometer,
        sensor::SensorType::OrientationSensor,
        sensor::SensorType::PoseSensor,
        sensor::SensorType::PositionSensor,
        sensor::SensorType::SkinSensor,
        sensor::SensorType::TemperatureSensor,
        sensor::SensorType::Torque3DSensor,
        sensor::SensorType::VirtualLinkKinSensor,
        sensor::SensorType::VirtualSphericalJointKinSensor,
    };

    VectorOfSensorPtr<const sensor::ISensor> allSensors;

    for (const auto& sensorType : allSensorTypes) {
        VectorOfSensorPtr<const sensor::ISensor> tmp;
        tmp = getSensors(sensorType);
        allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    }

    return allSensors;
}

inline wearable::VectorOfSensorNames
wearable::IWear::getSensorNames(const sensor::SensorType type) const
{
    const wearable::VectorOfSensorPtr<const sensor::ISensor> sensors = getSensors(type);

    VectorOfSensorNames sensorNames;
    sensorNames.reserve(sensors.size());

    for (const auto& s : sensors) {
        sensorNames.push_back(s->getSensorName());
    }
    return sensorNames;
}

inline wearable::VectorOfSensorNames wearable::IWear::getAllSensorNames() const
{
    const VectorOfSensorPtr<const sensor::ISensor> sensors = getAllSensors();

    VectorOfSensorNames sensorNames;
    sensorNames.reserve(sensors.size());

    for (const auto& s : sensors) {
        sensorNames.push_back(s->getSensorName());
    }

    return sensorNames;
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IAccelerometer>
wearable::IWear::getAccelerometers() const
{
    return castVectorOfSensorPtr<sensor::IAccelerometer>(
        getSensors(sensor::SensorType::Accelerometer));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IEmgSensor>
wearable::IWear::getEmgSensors() const
{
    return castVectorOfSensorPtr<sensor::IEmgSensor>(getSensors(sensor::SensorType::EmgSensor));
}

wearable::VectorOfSensorPtr<const wearable::sensor::IForce3DSensor>
wearable::IWear::getForce3DSensors() const
{
    return castVectorOfSensorPtr<sensor::IForce3DSensor>(
        getSensors(sensor::SensorType::Force3DSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IForceTorque6DSensor>
wearable::IWear::getForceTorque6DSensors() const
{
    return castVectorOfSensorPtr<sensor::IForceTorque6DSensor>(
        getSensors(sensor::SensorType::ForceTorque6DSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
wearable::IWear::getFreeBodyAccelerationSensors() const
{
    return castVectorOfSensorPtr<sensor::IFreeBodyAccelerationSensor>(
        getSensors(sensor::SensorType::FreeBodyAccelerationSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IGyroscope>
wearable::IWear::getGyroscopes() const
{
    return castVectorOfSensorPtr<sensor::IGyroscope>(getSensors(sensor::SensorType::Gyroscope));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IMagnetometer>
wearable::IWear::getMagnetometers() const
{
    return castVectorOfSensorPtr<sensor::IMagnetometer>(
        getSensors(sensor::SensorType::Magnetometer));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IOrientationSensor>
wearable::IWear::getOrientationSensors() const
{
    return castVectorOfSensorPtr<sensor::IOrientationSensor>(
        getSensors(sensor::SensorType::OrientationSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IPoseSensor>
wearable::IWear::getPoseSensors() const
{
    return castVectorOfSensorPtr<sensor::IPoseSensor>(getSensors(sensor::SensorType::PoseSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IPositionSensor>
wearable::IWear::getPositionSensors() const
{
    return castVectorOfSensorPtr<sensor::IPositionSensor>(
        getSensors(sensor::SensorType::PositionSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::ISkinSensor>
wearable::IWear::getSkinSensors() const
{
    return castVectorOfSensorPtr<sensor::ISkinSensor>(getSensors(sensor::SensorType::SkinSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::ITemperatureSensor>
wearable::IWear::getTemperatureSensors() const
{
    return castVectorOfSensorPtr<sensor::ITemperatureSensor>(
        getSensors(sensor::SensorType::TemperatureSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::ITorque3DSensor>
wearable::IWear::getTorque3DSensors() const
{
    return castVectorOfSensorPtr<sensor::ITorque3DSensor>(
        getSensors(sensor::SensorType::Torque3DSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
wearable::IWear::getVirtualLinkKinSensors() const
{
    return castVectorOfSensorPtr<sensor::IVirtualLinkKinSensor>(
        getSensors(sensor::SensorType::VirtualLinkKinSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
wearable::IWear::getVirtualSphericalJointKinSensors() const
{
    return castVectorOfSensorPtr<sensor::IVirtualSphericalJointKinSensor>(
        getSensors(sensor::SensorType::VirtualSphericalJointKinSensor));
}

#endif // WEARABLE_IWEAR_H
