// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_ISKIN_SENSOR_H
#define WEARABLE_ISKIN_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"
#include <vector>

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

    virtual bool getPressure(std::vector<double>& pressure) const = 0;

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::ISkinSensor::getPrefix()
{
    return "skin" + wearable::Separator;
}

#endif // WEARABLE_ISKIN_SENSOR_H
