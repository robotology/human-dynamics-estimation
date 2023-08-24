// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IFORCE_3D_SENSOR_H
#define WEARABLE_IFORCE_3D_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IForce3DSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IForce3DSensor : public wearable::sensor::ISensor
{
public:
    IForce3DSensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::Force3DSensor;
    }

    virtual ~IForce3DSensor() = default;

    virtual bool getForce3D(Vector3& force) const = 0;

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::IForce3DSensor::getPrefix()
{
    return "f3D" + wearable::Separator;
}

#endif // WEARABLE_IFORCE_3D_SENSOR_H
