// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IFRAMETRANSFORMTOIWEAR
#define WEARABLE_IFRAMETRANSFORMTOIWEAR

#include "Wearable/IWear/IWear.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/dev/IMultipleWrapper.h>

#include <memory>

namespace wearable {
    namespace devices {
        class IFrameTransformToIWear;
    } // namespace devices
} // namespace wearable

class wearable::devices::IFrameTransformToIWear
    : public wearable::IWear
    , public yarp::dev::DeviceDriver
    , public yarp::dev::IPreciselyTimed
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    IFrameTransformToIWear();
    ~IFrameTransformToIWear() override;

    IFrameTransformToIWear(const IFrameTransformToIWear& other) = delete;
    IFrameTransformToIWear(IFrameTransformToIWear&& other) = delete;
    IFrameTransformToIWear& operator=(const IFrameTransformToIWear& other) = delete;
    IFrameTransformToIWear& operator=(IFrameTransformToIWear&& other) = delete;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IPreciselyTimed
    yarp::os::Stamp getLastInputStamp() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;

    // =====
    // IWEAR
    // =====

    // -------
    // GENERIC
    // -------

    SensorPtr<const wearable::sensor::ISensor>
    getSensor(const wearable::sensor::SensorName name) const override;

    VectorOfSensorPtr<const wearable::sensor::ISensor>
    getSensors(const wearable::sensor::SensorType type) const override;

    WearableName getWearableName() const override;
    WearStatus getStatus() const override;
    TimeStamp getTimeStamp() const override;

    // --------------
    // SINGLE SENSORS
    // --------------

    SensorPtr<const wearable::sensor::IAccelerometer>
    getAccelerometer(const wearable::sensor::SensorName name) const override;

    SensorPtr<const wearable::sensor::IForce3DSensor>
    getForce3DSensor(const wearable::sensor::SensorName name) const override;

    SensorPtr<const wearable::sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const wearable::sensor::SensorName name) const override;

    SensorPtr<const wearable::sensor::IGyroscope>
    getGyroscope(const wearable::sensor::SensorName name) const override;

    SensorPtr<const wearable::sensor::IMagnetometer>
    getMagnetometer(const wearable::sensor::SensorName name) const override;

    SensorPtr<const wearable::sensor::IOrientationSensor>
    getOrientationSensor(const wearable::sensor::SensorName name) const override;

    SensorPtr<const wearable::sensor::ITemperatureSensor>
    getTemperatureSensor(const wearable::sensor::SensorName name) const override;

    SensorPtr<const wearable::sensor::ITorque3DSensor>
    getTorque3DSensor(const wearable::sensor::SensorName name) const override;

    SensorPtr<const sensor::IEmgSensor> getEmgSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IPoseSensor>
    getPoseSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IPositionSensor>
    getPositionSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::ISkinSensor>
    getSkinSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IVirtualJointKinSensor>
    getVirtualJointKinSensor(const sensor::SensorName name) const override;

    SensorPtr<const sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const sensor::SensorName name) const override;

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

inline wearable::ElementPtr<const wearable::actuator::IActuator>
wearable::devices::IFrameTransformToIWear::getActuator(const actuator::ActuatorName name) const
{
    return nullptr;
}

inline wearable::VectorOfElementPtr<const wearable::actuator::IActuator>
wearable::devices::IFrameTransformToIWear::getActuators(const actuator::ActuatorType type) const
{
    return {};
}

inline wearable::ElementPtr<const wearable::actuator::IHaptic>
wearable::devices::IFrameTransformToIWear::getHapticActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

inline wearable::ElementPtr<const wearable::actuator::IMotor>
wearable::devices::IFrameTransformToIWear::getMotorActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

inline wearable::ElementPtr<const wearable::actuator::IHeater>
wearable::devices::IFrameTransformToIWear::getHeaterActuator(const actuator::ActuatorName) const
{
    return nullptr;
}

#endif // WEARABLE_IFRAMETRANSFORMTOIWEAR
