// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IORIENTATION_SENSOR_H
#define WEARABLE_IORIENTATION_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"
#include "Wearable/IWear/Utils.h"

namespace wearable {
    namespace sensor {
        class IOrientationSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IOrientationSensor : public wearable::sensor::ISensor
{
public:
    IOrientationSensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::OrientationSensor;
    }

    virtual ~IOrientationSensor() = default;
    virtual bool getOrientationAsQuaternion(Quaternion& orientation) const = 0;

    inline bool getOrientationAsRPY(Vector3& orientation) const;
    inline bool getOrientationAsRotationMatrix(Matrix3& orientation) const;

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::IOrientationSensor::getPrefix()
{
    return "orient" + wearable::Separator;
}

inline bool wearable::sensor::IOrientationSensor::getOrientationAsRPY(Vector3& orientation) const
{
    Quaternion quat;
    if (!getOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = utils::quaternionToRPY(quat);
    return true;
};

inline bool
wearable::sensor::IOrientationSensor::getOrientationAsRotationMatrix(Matrix3& orientation) const
{
    Quaternion quat;
    if (!getOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = utils::quaternionToRotationMatrix(quat);
    return true;
};

#endif // WEARABLE_IORIENTATION_SENSOR_H
