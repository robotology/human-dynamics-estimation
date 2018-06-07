/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_XSENSSUIT
#define WEAR_XSENSSUIT

#include "IWear/IWear.h"

namespace wear {
    namespace suit {
        class XSensSuit;
    }
} // namespace wear

class wear::suit::XSensSuit final : public wear::IWear
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    XSensSuit();
    ~XSensSuit() override;

    XSensSuit(const XSensSuit& other) = delete;
    XSensSuit(XSensSuit&& other) = delete;
    XSensSuit& operator=(const XSensSuit& other) = delete;
    XSensSuit& operator=(XSensSuit&& other) = delete;

    // =======
    // GENERIC
    // =======

    wear::SensorPtr<wear::sensor::ISensor> getSensor(const wear::SensorName name) const override;

    // ==============
    // SINGLE SENSORS
    // ==============

    wear::SensorPtr<wear::sensor::I3DForce>
    get3DForceSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::I3DTorque>
    get3DTorqueSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::I6DForceTorque>
    get6DForceTorqueSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::IFreeBodyAcceleration>
    getFreeBodyAccelerationSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::IOrientation>
    getOrientationSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::ITemperature>
    getTemperatureSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::IVirtualSphericalJointKin>
    getVirtualSphericalJointKinSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::IVirtualLinkKin>
    getVirtualLinkKinSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::IAccelerometer>
    getAccelerometerSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::IGyroscope>
    getGyroscopeSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::IMagnetometer>
    getMagnetometerSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::ISkin> getSkinSensor(const wear::SensorName name) const override;

    wear::SensorPtr<wear::sensor::IEmg> getEmgSensor(const wear::SensorName name) const override;

    // ================
    // MULTIPLE SENSORS
    // ================

    wear::VectorOfSensorPtr<wear::sensor::I3DForce> getOrientationSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::I3DTorque> get3DTorqueSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::I6DForceTorque> get6DForceTorqueSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::IFreeBodyAcceleration>
    getFreeBodyAccelerationSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::IVirtualSphericalJointKin>
    getVirtualSphericalJointKinSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::IVirtualLinkKin>
    getVirtualLinkKinSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::IAccelerometer> getAccelerometerSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::IGyroscope> getGyroscopeSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::IMagnetometer> getMagnetometerSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::ISkin> getSkinSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::IEmg> getEmgSensors() const override;

    wear::VectorOfSensorPtr<wear::sensor::ITemperature> getTemperatureSensors() const override;
};

#endif // WEAR_XSENSSUIT
