/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IPOSITION_SENSOR_H
#define WEARABLE_IPOSITION_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IPositionSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IPositionSensor : public wearable::sensor::ISensor
{
public:
    IPositionSensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::PositionSensor;
    }

    virtual ~IPositionSensor() = default;

    virtual bool getPosition(Vector3& position) const = 0;

    inline static const std::string& getPrefix();
};

inline const std::string& wearable::sensor::IPositionSensor::getPrefix()
{
    static std::string prefix{"pos::"};
    return prefix;
}

#endif // WEARABLE_IPOSITION_SENSOR_H
