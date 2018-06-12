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

namespace wear {
    namespace suit {
        class FTShoes;
    }
} // namespace wear

class wear::suit::FTShoes final : public wear::IWear
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

    wear::SensorPtr<const wear::sensor::ISensor>
    getSensor(const wear::sensor::SensorName name) const override;

    wear::VectorOfSensorPtr<const wear::sensor::ISensor>
    getSensors(const wear::sensor::SensorType) const override;

    // ==============
    // SINGLE SENSORS
    // ==============

    virtual wear::SensorPtr<const wear::sensor::IAccelerometer>
    getAccelerometer(const wear::sensor::SensorName name) const override;

    virtual wear::SensorPtr<const wear::sensor::IForce3DSensor>
    getForce3DSensor(const wear::sensor::SensorName name) const override;

    virtual wear::SensorPtr<const wear::sensor::IForceTorque6DSensor>
    getForceTorque6DSensor(const wear::sensor::SensorName name) const override;

    virtual wear::SensorPtr<const wear::sensor::IGyroscope>
    getGyroscope(const wear::sensor::SensorName name) const override;

    virtual wear::SensorPtr<const wear::sensor::IMagnetometer>
    getMagnetometer(const wear::sensor::SensorName name) const override;

    virtual wear::SensorPtr<const wear::sensor::IOrientationSensor>
    getOrientationSensor(const wear::sensor::SensorName name) const override;

    virtual wear::SensorPtr<const wear::sensor::ITemperatureSensor>
    getTemperatureSensor(const wear::sensor::SensorName name) const override;

    virtual wear::SensorPtr<const wear::sensor::ITorque3DSensor>
    getTorque3DSensor(const wear::sensor::SensorName name) const override;
};

#endif // WEAR_FTSHOES
