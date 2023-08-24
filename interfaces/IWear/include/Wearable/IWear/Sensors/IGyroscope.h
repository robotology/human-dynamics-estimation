// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IGYROSCOPE_H
#define WEARABLE_IGYROSCOPE_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IGyroscope;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IGyroscope : public wearable::sensor::ISensor
{
public:
    IGyroscope(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::Gyroscope;
    }

    virtual ~IGyroscope() = default;

    virtual bool getAngularRate(Vector3& angularRate) const = 0;

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::IGyroscope::getPrefix()
{
    return "gyro" + wearable::Separator;
}

#endif // WEARABLE_IGYROSCOPE_H
