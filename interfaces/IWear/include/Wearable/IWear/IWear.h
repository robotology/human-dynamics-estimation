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
#include "Wearable/IWear/Sensors/IVirtualJointKinSensor.h"
#include "Wearable/IWear/Sensors/IVirtualSphericalJointKinSensor.h"

#include "Wearable/IWear/Actuators/IActuator.h"

#include "Wearable/IWear/Actuators/IHaptic.h"
#include "Wearable/IWear/Actuators/IMotor.h"
#include "Wearable/IWear/Actuators/IHeater.h"

#include <memory>
#include <string>
#include <vector>

namespace wearable {
    using WearableName = std::string;
    using WearStatus = sensor::SensorStatus;

    // Wearable sensors generic variables
    template <typename S>
    using SensorVector = std::vector<S>;

    template <typename S>
    using SensorPtr = std::shared_ptr<S>;

    template <typename S>
    using VectorOfSensorPtr = SensorVector<SensorPtr<S>>;

    using VectorOfSensorNames = SensorVector<sensor::SensorName>;

    // Wearable device (referes to either sensors or actuator) generic variables
    template <typename D>
    using DeviceVector = std::vector<D>;

    template <typename D>
    using DevicePtr = std::shared_ptr<D>;

    template <typename D>
    using VectorOfDevicePtr = DeviceVector<DevicePtr<D>>;

    using VectorOfActuatorNames = DeviceVector<actuator::ActuatorName>;

    using TimeStamp = struct
    {
        double time = 0;
        size_t sequenceNumber = 0;
    };

    const std::vector<sensor::SensorType> AllSensorTypes = {
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
        sensor::SensorType::VirtualJointKinSensor,
        sensor::SensorType::VirtualSphericalJointKinSensor,
    };

    const std::vector<actuator::ActuatorType> AllActuatorTypes = {
        actuator::ActuatorType::Haptic,
        actuator::ActuatorType::Motor,
        actuator::ActuatorType::Heater,
    };

    class IWear;
} // namespace wearable

class wearable::IWear
{
private:
    template <typename S>
    static VectorOfSensorPtr<const S>
    castVectorOfSensorPtr(const VectorOfSensorPtr<const sensor::ISensor>& iSensors);

    template <typename D, typename T> // Device, Type
    static VectorOfDevicePtr<const D>
    castVectorOfDevicePtr(const VectorOfDevicePtr<const T>& iDevices);

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

    // NOTE: Templatized virtual functions are not allowed
    virtual DevicePtr<const actuator::IActuator> getActuator(const actuator::ActuatorName name) const = 0;
    virtual VectorOfDevicePtr<const actuator::IActuator>
    getActuators(const actuator::ActuatorType type) const = 0;

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

    virtual SensorPtr<const sensor::IVirtualJointKinSensor>
    getVirtualJointKinSensor(const sensor::SensorName /*name*/) const = 0;

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

    inline VectorOfSensorPtr<const sensor::IVirtualJointKinSensor> getVirtualJointKinSensors() const;

    inline VectorOfSensorPtr<const sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensors() const;

    // ===============
    // SINGLE ACTUATOR
    // ===============

    virtual DevicePtr<const actuator::IHaptic>
    getHapticActuator(const actuator::ActuatorName) const = 0;

    virtual DevicePtr<const actuator::IMotor>
    getMotorActuator(const actuator::ActuatorName) const = 0;

    virtual DevicePtr<const actuator::IHeater>
    getHeaterActuator(const actuator::ActuatorName) const = 0;

    // ==========================
    // GENERIC ACTUATOR UTILITIES
    // ==========================

    inline VectorOfDevicePtr<const actuator::IActuator> getAllActuators() const;

    inline VectorOfActuatorNames getActuatorNames(const actuator::ActuatorType type) const;

    inline VectorOfActuatorNames getAllActuatorNames() const;

    // ===================
    // ACTUATORS UTILITIES
    // ===================

    inline VectorOfDevicePtr<const actuator::IHaptic> getHapticActuators() const;

    inline VectorOfDevicePtr<const actuator::IMotor> getMotorActuators() const;

    inline VectorOfDevicePtr<const actuator::IHeater> getHeaterActuators() const;

//    // ========================
//    // GENERIC DEVICE UTILITIES
//    // ========================

//    // TODO: The following methods can replace sensor generic utilities
//    // TODO: Two template names are redundant, check how to use one
//    template<typename D>
//    inline VectorOfDevicePtr<const D> getAllDevices() const;
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

template <typename D, typename T>
wearable::VectorOfDevicePtr<const D>
wearable::IWear::castVectorOfDevicePtr(const VectorOfDevicePtr<const T>& iDevices)
{
    VectorOfDevicePtr<const D> devices;
    devices.reserve(iDevices.size());

    for (const auto& iDevice : iDevices)
    {
        wearable::DevicePtr<const D> castDevice = std::dynamic_pointer_cast<const D>(iDevice);

        if (!castDevice)
        {

            if (castDevice->getWearableDeviceType() == wearable::DeviceType::WearableSensor)
            {
                wError << "Failed to cast wearable sensor device";
            }
            else if (castDevice->getWearableDeviceType() == wearable::DeviceType::WearableActuator)
            {
                wError << "Failed to case wearable actuator device";
            }
        }
    }

    return devices;
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
wearable::IWear::getAllSensors() const
{
    VectorOfSensorPtr<const sensor::ISensor> allSensors;

    for (const auto& sensorType : AllSensorTypes) {
        VectorOfSensorPtr<const sensor::ISensor> tmp;
        tmp = getSensors(sensorType);
        allSensors.insert(allSensors.end(), tmp.begin(), tmp.end());
    }

    return allSensors;
}

inline wearable::VectorOfDevicePtr<const wearable::actuator::IActuator>
wearable::IWear::getAllActuators() const
{
    VectorOfSensorPtr<const actuator::IActuator> allActuators;

    for (const auto& actuatorType : AllActuatorTypes) {
        VectorOfDevicePtr<const actuator::IActuator> tmp;
        tmp = getActuators(actuatorType);
        allActuators.insert(allActuators.end(), tmp.begin(), tmp.end());
    }

    return allActuators;
}

//template<typename D>
//inline wearable::VectorOfDevicePtr<const D> wearable::IWear::getAllDevices() const
//{
//    wearable::VectorOfDevicePtr<const D> allDevices;

//    if (allDevices.at(0)->getWearableDeviceType() == wearable::DeviceType::WearableSensor)
//    {
//        for (const auto& type : wearable::AllSensorTypes)
//        {
//            wearable::VectorOfDevicePtr<const sensor::ISensor> tmp;
//            tmp = getSensors(type);

//            allDevices.insert(allDevices.end(), tmp.begin(), tmp.end());
//        }

//    }

//    if (allDevices.at(0)->getWearableDeviceType() == wearable::DeviceType::WearableActuator)
//    {
//        for (const auto& type : wearable::AllActuatorTypes)
//        {
//            wearable::VectorOfDevicePtr<const actuator::IActuator> tmp;
//            tmp = getActuators(type);

//            allDevices.insert(allDevices.end(), tmp.begin(), tmp.end());
//        }
//    }

//    return allDevices;
//}

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

inline wearable::VectorOfActuatorNames wearable::IWear::getActuatorNames(const actuator::ActuatorType type) const
{
    const wearable::VectorOfDevicePtr<const actuator::IActuator> actuators = getActuators(type);

    VectorOfActuatorNames actuatorNames;
    actuatorNames.reserve(actuators.size());

    for (const auto& a : actuators) {
        actuatorNames.push_back(a->getActuatorName());
    }

    return actuatorNames;
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

inline wearable::VectorOfActuatorNames wearable::IWear::getAllActuatorNames() const
{
    const VectorOfDevicePtr<const actuator::IActuator> actuators = getAllActuators();

    VectorOfActuatorNames actuatorNames;
    actuatorNames.reserve(actuators.size());

    for (const auto& a : actuators) {
        actuatorNames.push_back(a->getActuatorName());
    }

    return actuatorNames;
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

inline wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualJointKinSensor>
wearable::IWear::getVirtualJointKinSensors() const
{
    return castVectorOfSensorPtr<sensor::IVirtualJointKinSensor>(
        getSensors(sensor::SensorType::VirtualJointKinSensor));
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
wearable::IWear::getVirtualSphericalJointKinSensors() const
{
    return castVectorOfSensorPtr<sensor::IVirtualSphericalJointKinSensor>(
        getSensors(sensor::SensorType::VirtualSphericalJointKinSensor));
}

inline wearable::VectorOfDevicePtr<const wearable::actuator::IHaptic>
wearable::IWear::getHapticActuators() const
{
    return castVectorOfDevicePtr<actuator::IHaptic, actuator::IActuator>(
                getActuators(actuator::ActuatorType::Haptic));
}

inline wearable::VectorOfDevicePtr<const wearable::actuator::IMotor>
wearable::IWear::getMotorActuators() const
{
    return castVectorOfDevicePtr<actuator::IMotor, actuator::IActuator>(
                getActuators(actuator::ActuatorType::Motor));
}

inline wearable::VectorOfDevicePtr<const wearable::actuator::IHeater>
wearable::IWear::getHeaterActuators() const
{
    return castVectorOfDevicePtr<actuator::IHeater, actuator::IActuator>(
                getActuators(actuator::ActuatorType::Heater));
}

#endif // WEARABLE_IWEAR_H
