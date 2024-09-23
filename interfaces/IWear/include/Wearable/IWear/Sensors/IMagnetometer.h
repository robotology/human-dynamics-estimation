// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IMAGNETOMETER_H
#define WEARABLE_IMAGNETOMETER_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IMagnetometer;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IMagnetometer : public wearable::sensor::ISensor
{
public:
    IMagnetometer(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::Magnetometer;
    }

    virtual ~IMagnetometer() = default;

    virtual bool getMagneticField(Vector3& magneticField) const = 0;

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::IMagnetometer::getPrefix()
{
    return "mag" + wearable::Separator;
}

#endif // WEARABLE_IMAGNETOMETER_H
