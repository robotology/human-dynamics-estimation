/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_ITEMPERATURE_SENSOR_H
#define WEARABLE_ITEMPERATURE_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class ITemperatureSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::ITemperatureSensor : public wearable::sensor::ISensor
{
public:
    ITemperatureSensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::TemperatureSensor;
    }

    virtual ~ITemperatureSensor() = 0;

    virtual bool getTemperature(double& temperature) const = 0;
};

#endif // WEARABLE_ITEMPERATURE_SENSOR_H
