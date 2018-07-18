/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_ISKIN_SENSOR_H
#define WEARABLE_ISKIN_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class ISkinSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::ISkinSensor : public wearable::sensor::ISensor
{
public:
    ISkinSensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::SkinSensor;
    }

    virtual ~ISkinSensor() = default;

    inline static const std::string& getPrefix();
    // TODO: to be implemented
};

inline const std::string& wearable::sensor::ISkinSensor::getPrefix()
{
    static std::string prefix{"skin_"};
    return prefix;
}

// const wearable::sensor::SensorPrefix wearable::sensor::ISkinSensor::namePrefix = "skin_";
#endif // WEARABLE_ISKIN_SENSOR_H
