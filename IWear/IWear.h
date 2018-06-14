/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IWEAR
#define WEAR_IWEAR

#include "IWear/Sensors/ISensor.h"

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
    virtual ~IWear() = 0;

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
    getAccelerometer(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IEmgSensor>
    getEmgSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IForce3DSensor>
    getForce3DSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IGyroscope>
    getGyroscope(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IMagnetometer>
    getMagnetometer(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IOrientationSensor>
    getOrientationSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IPoseSensor>
    getPoseSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IPositionSensor>
    getPositionSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::ISkinSensor>
    getSkinSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::ITemperatureSensor>
    getTemperatureSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::ITorque3DSensor>
    getTorque3DSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const wear::sensor::SensorName /*name*/) const;

    virtual wear::SensorPtr<const wear::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const wear::sensor::SensorName /*name*/) const;

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

#include "IWear/IWear-defaultImpl.h"

#endif // WEAR_IWEAR
