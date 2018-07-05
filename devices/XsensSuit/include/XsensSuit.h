/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENSSUIT_H
#define XSENSSUIT_H

#include "Wearable/IWear/IWear.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PreciselyTimed.h>

#include <memory>

namespace wearable {
    namespace devices {
        class XsensSuit;
    }
} // namespace wearable

class wearable::devices::XsensSuit final
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IPreciselyTimed
    , public wearable::IWear
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    XsensSuit();
    ~XsensSuit() override;

    XsensSuit(const XsensSuit& other) = delete;
    XsensSuit(XsensSuit&& other) = delete;
    XsensSuit& operator=(const XsensSuit& other) = delete;
    XsensSuit& operator=(XsensSuit&& other) = delete;

    // ============
    // DEVICEDRIVER
    // ============

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // ===============
    // IPRECISELYTIMED
    // ===============

    yarp::os::Stamp getLastInputStamp() override;

    // =====
    // IWEAR
    // =====

    // GENERIC
    // -------

    wearable::WearStatus getStatus() const override;
    wearable::TimeStamp getTimeStamp() const override;

    wearable::SensorPtr<const wearable::sensor::ISensor>
    getSensor(const wearable::sensor::SensorName name) const override;

    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
    getSensors(const wearable::sensor::SensorType) const override;

    // IMPLEMENTED SENSORS
    // -------------------

    wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IMagnetometer>
    getMagnetometer(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
    getOrientationSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IPoseSensor>
    getPoseSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IPositionSensor>
    getPositionSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const wearable::sensor::SensorName name) const override;

    // NOT IMPLEMENTED SENSORS
    // -----------------------

    inline wearable::SensorPtr<const wearable::sensor::IAccelerometer>
    getAccelerometer(const wearable::sensor::SensorName /*name*/) const override;

    inline wearable::SensorPtr<const wearable::sensor::IEmgSensor>
    getEmgSensor(const wearable::sensor::SensorName /*name*/) const override;

    inline wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
    getForce3DSensor(const wearable::sensor::SensorName /*name*/) const override;

    inline wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const wearable::sensor::SensorName /*name*/) const override;

    inline wearable::SensorPtr<const wearable::sensor::IGyroscope>
    getGyroscope(const wearable::sensor::SensorName /*name*/) const override;

    inline wearable::SensorPtr<const wearable::sensor::ISkinSensor>
    getSkinSensor(const wearable::sensor::SensorName /*name*/) const override;

    inline wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
    getTemperatureSensor(const wearable::sensor::SensorName /*name*/) const override;

    inline wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
    getTorque3DSensor(const wearable::sensor::SensorName /*name*/) const override;
};

inline wearable::SensorPtr<const wearable::sensor::IAccelerometer>
wearable::devices::XsensSuit::getAccelerometer(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IEmgSensor>
wearable::devices::XsensSuit::getEmgSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
wearable::devices::XsensSuit::getForce3DSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
wearable::devices::XsensSuit::getForceTorque6DSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IGyroscope>
wearable::devices::XsensSuit::getGyroscope(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ISkinSensor>
wearable::devices::XsensSuit::getSkinSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
wearable::devices::XsensSuit::getTemperatureSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
wearable::devices::XsensSuit::getTorque3DSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

#endif // XSENSSUIT_H
