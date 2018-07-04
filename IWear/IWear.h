/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IWEAR
#define WEAR_IWEAR

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
        size_t time = 0;
        size_t sequenceNumber = 0;
    };

    template <typename S>
    void castVectorOfSensorPtr(const VectorOfSensorPtr<const sensor::ISensor> isensors,
                               VectorOfSensorPtr<const S>& sensors);

    class IWear;
} // namespace wearable

class wearable::IWear
{
private:
    //    template <typename S>
    //    static void castVectorOfSensorPtr(
    //        const wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> isensors,
    //        wearable::VectorOfSensorPtr<const S> sensors);
    template <typename S>
    static VectorOfSensorPtr<const S>
    castVectorOfSensorPtr(const VectorOfSensorPtr<const sensor::ISensor> iSensors);

public:
    virtual ~IWear() = 0;

    // ===============
    // GENERIC METHODS
    // ===============

    virtual wearable::WearStatus getStatus() const = 0;
    virtual wearable::TimeStamp getTimeStamp() const = 0;

    virtual wearable::SensorPtr<const wearable::sensor::ISensor>
    getSensor(const wearable::sensor::SensorName name) const = 0;
    virtual wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
    getSensors(const wearable::sensor::SensorType type) const = 0;

    // ==============
    // SINGLE SENSORS
    // ==============

    virtual wearable::SensorPtr<const wearable::sensor::IAccelerometer>
    getAccelerometer(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IEmgSensor>
    getEmgSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
    getForce3DSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IGyroscope>
    getGyroscope(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IMagnetometer>
    getMagnetometer(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
    getOrientationSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IPoseSensor>
    getPoseSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IPositionSensor>
    getPositionSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::ISkinSensor>
    getSkinSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
    getTemperatureSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
    getTorque3DSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const wearable::sensor::SensorName /*name*/) const;

    virtual wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const wearable::sensor::SensorName /*name*/) const;

    // =================
    // GENERIC UTILITIES
    // =================

    inline wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> getAllSensors() const;

    inline wearable::VectorOfSensorNames
    getSensorNames(const wearable::sensor::SensorType type) const;

    inline wearable::VectorOfSensorNames getAllSensorNames() const;

    // ================
    // SENSOR UTILITIES
    // ================

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IAccelerometer>
    getAccelerometers() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IEmgSensor> getEmgSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IForce3DSensor>
    getForce3DSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IForceTorque6DSensor>
    getForceTorque6DSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IGyroscope> getGyroscopes() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IMagnetometer>
    getMagnetometers() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IOrientationSensor>
    getOrientationSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IPoseSensor> getPoseSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IPositionSensor>
    getPositionSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::ISkinSensor> getSkinSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::ITemperatureSensor>
    getTemperatureSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::ITorque3DSensor>
    get3DTorqueSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensors() const;

    inline wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensors() const;
};

// =========================================
// DEFAULT IMPLEMENTATION OF VIRTUAL METHODS
// =========================================

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

// ============================================
// IMPLEMENTATION OF INLINE AND PRIVATE METHODS
// ============================================

template <typename S>
static wearable::VectorOfSensorPtr<const S>
castVectorOfSensorPtr(const wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> iSensors)
{
    wearable::VectorOfSensorPtr<const S> sensors;
    sensors.reserve(iSensors.size());

    for (const auto& s : iSensors) {
        sensors.push_back(std::dynamic_pointer_cast<const S>(s));
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

    VectorOfSensorPtr<const sensor::ISensor> allSensors, tmp;

    for (const auto& sensorType : allSensorTypes) {
        VectorOfSensorPtr<const sensor::ISensor> tmp;
        tmp = getSensors(sensorType);
        allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    }

    return allSensors;
}

inline wearable::VectorOfSensorNames
wearable::IWear::getSensorNames(const wearable::sensor::SensorType type) const
{
    const wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> sensors = getSensors(type);

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
wearable::IWear::get3DTorqueSensors() const
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

#endif // WEAR_IWEAR
