/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_FTSHOES
#define WEAR_FTSHOES

#include "IWear/IWear.h"

namespace wearable {
    namespace suit {
        class FTShoes;
    }
} // namespace wearable

class wearable::suit::FTShoes final : public wearable::IWear
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    FTShoes();
    ~FTShoes() override;

    FTShoes(const FTShoes& other) = delete;
    FTShoes(FTShoes&& other) = delete;
    FTShoes& operator=(const FTShoes& other) = delete;
    FTShoes& operator=(FTShoes&& other) = delete;

    // =======
    // GENERIC
    // =======

    wearable::SensorPtr<const wearable::sensor::ISensor>
    getSensor(const wearable::sensor::SensorName name) const override;

    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
    getSensors(const wearable::sensor::SensorType) const override;

    // ==============
    // SINGLE SENSORS
    // ==============

    virtual wearable::SensorPtr<const wearable::sensor::IAccelerometer>
    getAccelerometer(const wearable::sensor::SensorName name) const override;

    virtual wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
    getForce3DSensor(const wearable::sensor::SensorName name) const override;

    virtual wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const wearable::sensor::SensorName name) const override;

    virtual wearable::SensorPtr<const wearable::sensor::IGyroscope>
    getGyroscope(const wearable::sensor::SensorName name) const override;

    virtual wearable::SensorPtr<const wearable::sensor::IMagnetometer>
    getMagnetometer(const wearable::sensor::SensorName name) const override;

    virtual wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
    getOrientationSensor(const wearable::sensor::SensorName name) const override;

    virtual wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
    getTemperatureSensor(const wearable::sensor::SensorName name) const override;

    virtual wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
    getTorque3DSensor(const wearable::sensor::SensorName name) const override;
};

#endif // WEAR_FTSHOES
