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
public:
    virtual ~IWear() {}

    // =======
    // GENERIC
    // =======

    virtual wear::SensorPtr<wear::sensor::ISensor>
    getSensor(const wear::sensor::SensorName name) const = 0;

    // ==============
    // SINGLE SENSORS
    // ==============

    virtual wear::SensorPtr<wear::sensor::IAccelerometer>
    getAccelerometer(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IEmgSensor>
    getEmgSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IForce3DSensor>
    getForce3DSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IGyroscope>
    getGyroscope(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IMagnetometer>
    getMagnetometer(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IOrientationSensor>
    getOrientationSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IPoseSensor>
    getPoseSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IPositionSensor>
    getPositionSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::ISkinSensor>
    getSkinSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::ITemperatureSensor>
    getTemperatureSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::ITorque3DSensor>
    getTorque3DSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    virtual wear::SensorPtr<wear::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const wear::sensor::SensorName name) const
    {
        return nullptr;
    }

    // ================
    // MULTIPLE SENSORS
    // ================

    virtual wear::VectorOfSensorPtr<wear::sensor::IAccelerometer> getAccelerometers() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::IEmgSensor> getEmgSensors() const { return {}; }

    virtual wear::VectorOfSensorPtr<wear::sensor::IForce3DSensor> getForce3DSensors() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::IForceTorque6DSensor>
    getForceTorque6DSensors() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensors() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::IGyroscope> getGyroscopes() const { return {}; }

    virtual wear::VectorOfSensorPtr<wear::sensor::IMagnetometer> getMagnetometers() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::IOrientationSensor> getOrientationSensors() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::IPoseSensor> getPoseSensors() const { return {}; }

    virtual wear::VectorOfSensorPtr<wear::sensor::IPositionSensor> getPositionSensors() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::ISkinSensor> getSkinSensors() const { return {}; }

    virtual wear::VectorOfSensorPtr<wear::sensor::ITemperatureSensor> getTemperatureSensors() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::ITorque3DSensor> get3DTorqueSensors() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensors() const
    {
        return {};
    }

    virtual wear::VectorOfSensorPtr<wear::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensors() const
    {
        return {};
    }
};

#endif // WEAR_IWEAR
