/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IEMG_SENSOR_H
#define WEARABLE_IEMG_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IEmgSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IEmgSensor : public wearable::sensor::ISensor
{
public:
    IEmgSensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::EmgSensor;
    }

    virtual ~IEmgSensor() = default;

    virtual bool getEmgSignal(double& emgSignal) const = 0;
    virtual bool getNormalizationValue(double& normalizationValue) const = 0;
};

#endif // WEARABLE_IEMG_SENSOR_H
