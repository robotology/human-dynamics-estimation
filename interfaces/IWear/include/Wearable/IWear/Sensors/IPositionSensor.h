// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::IPositionSensor::getPrefix()
{
    return "pos" + wearable::Separator;
}

#endif // WEARABLE_IPOSITION_SENSOR_H
