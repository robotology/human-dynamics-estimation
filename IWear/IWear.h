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

#include "IWear/Sensors/I3DForce.h"
#include "IWear/Sensors/I3DTorque.h"
#include "IWear/Sensors/I6DForceTorque.h"
#include "IWear/Sensors/IAccelerometer.h"
#include "IWear/Sensors/IEmg.h"
#include "IWear/Sensors/IFreeBodyAcceleration.h"
#include "IWear/Sensors/IGyroscope.h"
#include "IWear/Sensors/IMagnetometer.h"
#include "IWear/Sensors/IOrientation.h"
#include "IWear/Sensors/ISkin.h"
#include "IWear/Sensors/ITemperature.h"
#include "IWear/Sensors/IVirtualJointKin.h"
#include "IWear/Sensors/IVirtualLinkKin.h"

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

    virtual wear::SensorPtr<wear::sensor::ISensor> getSensor(const wear::SensorName name) const = 0;

    // ==============
    // SINGLE SENSORS
    // ==============

    virtual wear::SensorPtr<wear::sensor::I3DForce>
    get3DForceSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::I3DTorque>
    get3DTorqueSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::I6DForceTorque>
    get6DForceTorqueSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::IFreeBodyAcceleration>
    getFreeBodyAccelerationSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::IOrientation>
    getOrientationSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::ITemperature>
    getTemperatureSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::IVirtualSphericalJointKin>
    getVirtualSphericalJointKinSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::IVirtualLinkKin>
    getVirtualLinkKinSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::IAccelerometer>
    getAccelerometerSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::IGyroscope>
    getGyroscopeSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::IMagnetometer>
    getMagnetometerSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::ISkin>
    getSkinSensor(const wear::SensorName name) const = 0;

    virtual wear::SensorPtr<wear::sensor::IEmg> getEmgSensor(const wear::SensorName name) const = 0;

    // ================
    // MULTIPLE SENSORS
    // ================

    virtual wear::VectorOfSensorPtr<wear::sensor::I3DForce> getOrientationSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::I3DTorque> get3DTorqueSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::I6DForceTorque>
    get6DForceTorqueSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::IFreeBodyAcceleration>
    getFreeBodyAccelerationSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::IVirtualSphericalJointKin>
    getVirtualSphericalJointKinSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::IVirtualLinkKin>
    getVirtualLinkKinSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::IAccelerometer>
    getAccelerometerSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::IGyroscope> getGyroscopeSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::IMagnetometer> getMagnetometerSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::ISkin> getSkinSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::IEmg> getEmgSensors() const = 0;

    virtual wear::VectorOfSensorPtr<wear::sensor::ITemperature> getTemperatureSensors() const = 0;
};

#endif // WEAR_IWEAR
