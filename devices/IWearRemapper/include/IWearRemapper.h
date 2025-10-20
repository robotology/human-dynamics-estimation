// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IWEARREMAPPER_H
#define IWEARREMAPPER_H

#include "Wearable/IWear/IWear.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace trintrin::msgs {
    class WearableData;
} // namespace trintrin::msgs
namespace wearable {
    namespace devices {
        class IWearRemapper;
    }
} // namespace wearable

class wearable::devices::IWearRemapper
    : public yarp::dev::DeviceDriver
    , public wearable::IWear
    , public yarp::os::TypedReaderCallback<trintrin::msgs::WearableData>
    , public yarp::os::PeriodicThread
    , public yarp::dev::IMultipleWrapper
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
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(trintrin::msgs::WearableData& wearData, const yarp::os::TypedReader<trintrin::msgs::WearableData>& typedReader) override;

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

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;
};

inline wearable::ElementPtr<const wearable::actuator::IActuator>
wearable::devices::IWearRemapper::getActuator(const actuator::ActuatorName name) const
{
    return nullptr;
}

inline wearable::VectorOfElementPtr<const wearable::actuator::IActuator>
wearable::devices::IWearRemapper::getActuators(const actuator::ActuatorType type) const
{
    return {};
}

inline wearable::ElementPtr<const wearable::actuator::IHaptic>
wearable::devices::IWearRemapper::getHapticActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

inline wearable::ElementPtr<const wearable::actuator::IMotor>
wearable::devices::IWearRemapper::getMotorActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

inline wearable::ElementPtr<const wearable::actuator::IHeater>
wearable::devices::IWearRemapper::getHeaterActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

#endif // IWEARREMAPPER_H
