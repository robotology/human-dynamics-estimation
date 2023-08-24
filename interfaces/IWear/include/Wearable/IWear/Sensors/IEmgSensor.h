// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::IEmgSensor::getPrefix()
{
    return "emg" + wearable::Separator;
}
#endif // WEARABLE_IEMG_SENSOR_H
