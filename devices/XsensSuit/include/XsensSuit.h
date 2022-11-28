/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENSSUIT_H
#define XSENSSUIT_H

#include "IXsensMVNControl.h"
#include "Wearable/IWear/IWear.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>

#include <memory>

namespace wearable {
    namespace devices {
        class XsensSuit;
    } // namespace devices
} // namespace wearable

class wearable::devices::XsensSuit final
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IPreciselyTimed
    , public xsensmvn::IXsensMVNControl
    , public wearable::IWear
{
private:
    class XsensSuitImpl;
    std::unique_ptr<XsensSuitImpl> pImpl;

public:
    XsensSuit();
    ~XsensSuit() override;

    XsensSuit(const XsensSuit& other) = delete;
    XsensSuit(XsensSuit&& other) = delete;
    XsensSuit& operator=(const XsensSuit& other) = delete;
    XsensSuit& operator=(XsensSuit&& other) = delete;

    // =============
    // DEVICE_DRIVER
    // =============

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // ================
    // IPRECISELY_TIMED
    // ================

    yarp::os::Stamp getLastInputStamp() override;

    // ==================
    // IXSENS_MVN_CONTROL
    // ==================

    bool setBodyDimensions(const std::map<std::string, double>& dimensions) override;
    bool getBodyDimensions(std::map<std::string, double>& dimensions) const override;
    bool getBodyDimension(const std::string bodyName, double& dimension) const override;

    // Calibration methods
    bool calibrate(const std::string& calibrationType = {}) override;
    bool abortCalibration() override;

    // Acquisition methods
    bool startAcquisition() override;
    bool stopAcquisition() override;

    // =====
    // IWEAR
    // =====

    // GENERIC
    // -------
    WearableName getWearableName() const override;
    WearStatus getStatus() const override;
    TimeStamp getTimeStamp() const override;

    SensorPtr<const sensor::ISensor> getSensor(const sensor::SensorName name) const override;

    VectorOfSensorPtr<const sensor::ISensor> getSensors(const sensor::SensorType) const override;

    // IMPLEMENTED SENSORS
    // -------------------

    SensorPtr<const sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IMagnetometer>
    getMagnetometer(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IOrientationSensor>
    getOrientationSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IPoseSensor>
    getPoseSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IPositionSensor>
    getPositionSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const sensor::SensorName name) const override;

    // NOT IMPLEMENTED SENSORS
    // -----------------------

    inline SensorPtr<const sensor::IAccelerometer>
    getAccelerometer(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IEmgSensor>
    getEmgSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IForce3DSensor>
    getForce3DSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IGyroscope>
    getGyroscope(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::ISkinSensor>
    getSkinSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::ITemperatureSensor>
    getTemperatureSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::ITorque3DSensor>
    getTorque3DSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IVirtualJointKinSensor>
    getVirtualJointKinSensor(const sensor::SensorName /*name*/) const override;

    inline ElementPtr<const actuator::IActuator>
    getActuator(const actuator::ActuatorName name) const override;

    inline VectorOfElementPtr<const actuator::IActuator>
    getActuators(const actuator::ActuatorType type) const override;

    inline ElementPtr<const actuator::IHaptic>
    getHapticActuator(const actuator::ActuatorName) const override;

    inline ElementPtr<const actuator::IMotor>
    getMotorActuator(const actuator::ActuatorName) const override;

    inline ElementPtr<const actuator::IHeater>
    getHeaterActuator(const actuator::ActuatorName) const override;
};

inline wearable::SensorPtr<const wearable::sensor::IAccelerometer>
wearable::devices::XsensSuit::getAccelerometer(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IEmgSensor>
wearable::devices::XsensSuit::getEmgSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
wearable::devices::XsensSuit::getForce3DSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
wearable::devices::XsensSuit::getForceTorque6DSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IGyroscope>
wearable::devices::XsensSuit::getGyroscope(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ISkinSensor>
wearable::devices::XsensSuit::getSkinSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
wearable::devices::XsensSuit::getTemperatureSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
wearable::devices::XsensSuit::getTorque3DSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
wearable::devices::XsensSuit::getVirtualJointKinSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::ElementPtr<const wearable::actuator::IActuator>
wearable::devices::XsensSuit::getActuator(const actuator::ActuatorName name) const
{
    return nullptr;
}

inline wearable::VectorOfElementPtr<const wearable::actuator::IActuator>
wearable::devices::XsensSuit::getActuators(const actuator::ActuatorType type) const
{
    return {};
}

inline wearable::ElementPtr<const wearable::actuator::IHaptic>
wearable::devices::XsensSuit::getHapticActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

inline wearable::ElementPtr<const wearable::actuator::IMotor>
wearable::devices::XsensSuit::getMotorActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

inline wearable::ElementPtr<const wearable::actuator::IHeater>
wearable::devices::XsensSuit::getHeaterActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

#endif // XSENSSUIT_H
