/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef IWEARREMAPPER_H
#define IWEARREMAPPER_H

#include "Wearable/IWear/IWear.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace wearable {
    namespace msg {
        class WearableData;
    }
    namespace devices {
        class IWearRemapper;
    }
} // namespace wearable

class wearable::devices::IWearRemapper
    : public yarp::dev::DeviceDriver
    , public wearable::IWear
    , public yarp::os::TypedReaderCallback<msg::WearableData>
    , public yarp::os::PeriodicThread
    , public yarp::dev::IPreciselyTimed
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    IWearRemapper();
    ~IWearRemapper() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

    // TypedReaderCallback
    void onRead(msg::WearableData& wearData) override;

    // PreciselyTimed interface
    yarp::os::Stamp getLastInputStamp() override;

    // IWear interface
    WearableName getWearableName() const override;
    WearStatus getStatus() const override;
    TimeStamp getTimeStamp() const override;

    SensorPtr<const sensor::ISensor> getSensor(const sensor::SensorName name) const override;
    VectorOfSensorPtr<const sensor::ISensor>
    getSensors(const sensor::SensorType type) const override;

    SensorPtr<const sensor::IAccelerometer>
    getAccelerometer(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IEmgSensor>
    getEmgSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IForce3DSensor>
    getForce3DSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IGyroscope>
    getGyroscope(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IMagnetometer>
    getMagnetometer(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IOrientationSensor>
    getOrientationSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IPoseSensor>
    getPoseSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IPositionSensor>
    getPositionSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::ISkinSensor>
    getSkinSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::ITemperatureSensor>
    getTemperatureSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::ITorque3DSensor>
    getTorque3DSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IVirtualJointKinSensor>
    getVirtualJointKinSensor(const sensor::SensorName /*name*/) const override;

    SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const sensor::SensorName /*name*/) const override;
};

#endif // IWEARREMAPPER_H
