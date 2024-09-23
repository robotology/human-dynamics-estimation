// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

    virtual ~ITemperatureSensor() = default;

    virtual bool getTemperature(double& temperature) const = 0;

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::ITemperatureSensor::getPrefix()
{
    return "temp" + wearable::Separator;
}
#endif // WEARABLE_ITEMPERATURE_SENSOR_H
