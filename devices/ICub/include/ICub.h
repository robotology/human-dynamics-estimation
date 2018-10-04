/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef ICUB_H
#define ICUB_H

#include "Wearable/IWear/IWear.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PreciselyTimed.h>

#include <memory>

namespace wearable {
    namespace   devices {
        class ICub;
    } // namespace devices
} // namespace wearable

class wearable::devices::ICub final
        : public yarp::dev::DeviceDriver
        , public yarp::dev::IPreciselyTimed
        , public wearable::IWear
{
private:
    class ICubImpl;
    std::unique_ptr<ICubImpl> pImpl;

public:
    ICub();
    ~ICub() override;

    ICub(const ICub& other) = delete;
    ICub(ICub&& other) = delete;
    ICub& operator=(const ICub& other) = delete;
    ICub& operator=(ICub&& other) = delete;

    // =============
    // DEVICE DRIVER
    // =============

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // ================
    // IPRECISELY TIMED
    // ================

    yarp::os::Stamp getLastInputStamp() override;

    // =====
    // IWEAR
    // =====

    // GENERIC
    // -------

    WearableName getWearableName() const override;
    WearStatus getStatus() const override;
    //TimeStamp getTimeStamp() const override;

    // IMPLEMENTED SENSORS
    // -------------------

    SensorPtr<const sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const sensor::SensorName name) const override;

    // UNIMPLEMENTED SENSORS
    // ---------------------

    inline SensorPtr<const sensor::ISensor>
    getSensor(const sensor::SensorName /*name*/) const override;

    inline VectorOfSensorPtr<const sensor::ISensor>
    getSensors(const sensor::SensorType) const override;

    inline SensorPtr<const sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IMagnetometer>
    getMagnetometer(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IOrientationSensor>
    getOrientationSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IPoseSensor>
    getPoseSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IPositionSensor>
    getPositionSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IAccelerometer>
    getAccelerometer(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IEmgSensor>
    getEmgSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IForce3DSensor>
    getForce3DSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::IGyroscope>
    getGyroscope(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::ISkinSensor>
    getSkinSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::ITemperatureSensor>
    getTemperatureSensor(const sensor::SensorName /*name*/) const override;

    inline SensorPtr<const sensor::ITorque3DSensor>
    getTorque3DSensor(const sensor::SensorName /*name*/) const override;

};

inline wearable::SensorPtr<const wearable::sensor::ISensor>
wearable::devices::ICub::getSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
wearable::devices::ICub::getSensors(const sensor::SensorType) const
{
    return {nullptr};
}

inline wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
wearable::devices::ICub::getFreeBodyAccelerationSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IMagnetometer>
wearable::devices::ICub::getMagnetometer(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
wearable::devices::ICub::getOrientationSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IPoseSensor>
wearable::devices::ICub::getPoseSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IPositionSensor>
wearable::devices::ICub::getPositionSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
wearable::devices::ICub::getVirtualLinkKinSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
wearable::devices::ICub::getVirtualSphericalJointKinSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IAccelerometer>
wearable::devices::ICub::getAccelerometer(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IEmgSensor>
wearable::devices::ICub::getEmgSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
wearable::devices::ICub::getForce3DSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::IGyroscope>
wearable::devices::ICub::getGyroscope(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ISkinSensor>
wearable::devices::ICub::getSkinSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
wearable::devices::ICub::getTemperatureSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

inline wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
wearable::devices::ICub::getTorque3DSensor(const sensor::SensorName /*name*/) const
{
    return nullptr;
}

#endif // ICUB_H
