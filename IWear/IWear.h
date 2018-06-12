/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IWEAR
#define WEAR_IWEAR

#include "IWear/IWear.h"

#include "IWear/Sensors/IAccelerometer.h"
#include "IWear/Sensors/IEmgSensor.h"
#include "IWear/Sensors/IForce3DSensor.h"
#include "IWear/Sensors/IForceTorque6DSensor.h"
#include "IWear/Sensors/IFreeBodyAccelerationSensor.h"
#include "IWear/Sensors/IGyroscope.h"
#include "IWear/Sensors/IMagnetometer.h"
#include "IWear/Sensors/IOrientationSensor.h"
#include "IWear/Sensors/IPoseSensor.h"
#include "IWear/Sensors/IPositionSensor.h"
#include "IWear/Sensors/ISkinSensor.h"
#include "IWear/Sensors/ITemperatureSensor.h"
#include "IWear/Sensors/ITorque3DSensor.h"
#include "IWear/Sensors/IVirtualLinkKinSensor.h"
#include "IWear/Sensors/IVirtualSphericalJointKinSensor.h"

#include <memory>
#include <string>
#include <vector>

namespace wear {

    template <typename S>
    using SensorVector = std::vector<S>;

    template <typename S>
    using SensorPtr = std::shared_ptr<S>;

    template <typename S>
    using VectorOfSensorPtr = wear::SensorVector<wear::SensorPtr<S>>;

    using VectorOfSensorNames = wear::SensorVector<wear::sensor::SensorName>;

    class IWear;
} // namespace wear

class wear::IWear
{
private:
    template <typename S>
    static void
    castVectorOfSensorPtr(const wear::VectorOfSensorPtr<const wear::sensor::ISensor> isensors,
                          wear::VectorOfSensorPtr<const S> sensors);

public:
    virtual ~IWear() {}

    // =======
    // GENERIC
    // =======

    virtual wear::SensorPtr<const wear::sensor::ISensor>
    getSensor(const wear::sensor::SensorName name) const = 0;

    virtual wear::VectorOfSensorPtr<const wear::sensor::ISensor>
    getSensors(const wear::sensor::SensorType type) const = 0;

    virtual wear::VectorOfSensorPtr<const wear::sensor::ISensor> getAllSensors() const;

    virtual wear::VectorOfSensorNames getSensorNames(const wear::sensor::SensorType type) const;

    virtual wear::VectorOfSensorNames getAllSensorNames() const;

    // ==============
    // SINGLE SENSORS
    // ==============

    virtual wear::SensorPtr<const wear::sensor::IAccelerometer>
    getAccelerometer(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IEmgSensor>
    getEmgSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IForce3DSensor>
    getForce3DSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IGyroscope>
    getGyroscope(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IMagnetometer>
    getMagnetometer(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IOrientationSensor>
    getOrientationSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IPoseSensor>
    getPoseSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IPositionSensor>
    getPositionSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::ISkinSensor>
    getSkinSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::ITemperatureSensor>
    getTemperatureSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::ITorque3DSensor>
    getTorque3DSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<const wear::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    // ================
    // MULTIPLE SENSORS
    // ================

    virtual wear::VectorOfSensorPtr<const wear::sensor::IAccelerometer> getAccelerometers() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IEmgSensor> getEmgSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IForce3DSensor> getForce3DSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IForceTorque6DSensor>
    getForceTorque6DSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IGyroscope> getGyroscopes() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IMagnetometer> getMagnetometers() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IOrientationSensor>
    getOrientationSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IPoseSensor> getPoseSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IPositionSensor> getPositionSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::ISkinSensor> getSkinSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::ITemperatureSensor>
    getTemperatureSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::ITorque3DSensor> get3DTorqueSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensors() const;

    virtual wear::VectorOfSensorPtr<const wear::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensors() const;
};

template <typename S>
void wear::IWear::castVectorOfSensorPtr(
    const wear::VectorOfSensorPtr<const wear::sensor::ISensor> isensors,
    wear::VectorOfSensorPtr<const S> sensors)
{
    for (const auto& s : isensors) {
        sensors.push_back(std::static_pointer_cast<const S>(s));
    }
}

wear::VectorOfSensorPtr<const wear::sensor::ISensor> wear::IWear::getAllSensors() const
{
    wear::VectorOfSensorPtr<const wear::sensor::ISensor> allSensors, tmp;
    tmp = getSensors(wear::sensor::SensorType::Accelerometer);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::EmgSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::Force3DSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::ForceTorque6DSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::FreeBodyAccelerationSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::Gyroscope);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::Magnetometer);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::OrientationSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::PoseSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::PositionSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::SkinSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::TemperatureSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::Torque3DSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::VirtualLinkKinSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    tmp.clear();
    tmp = getSensors(wear::sensor::SensorType::VirtualSphericalJointKinSensor);
    allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    return allSensors;
}

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

#endif // WEAR_IWEAR
