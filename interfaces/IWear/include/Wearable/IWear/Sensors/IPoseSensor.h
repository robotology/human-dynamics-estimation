/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IPOSE_SENSOR_H
#define WEARABLE_IPOSE_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"
#include "Wearable/IWear/Sensors/Utils.h"

namespace wearable {
    namespace sensor {
        class IPoseSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IPoseSensor : public wearable::sensor::ISensor
{
public:
    IPoseSensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::PoseSensor;
    }

    virtual ~IPoseSensor() = default;

    virtual bool getPose(Quaternion& orientation, Vector3& position) const = 0;

    inline bool getPose(Vector7& pose) const;
    inline bool getPoseOrientationAsQuaternion(Quaternion& orientation) const;
    inline bool getPosePosition(Vector3& position) const;
    inline bool getPoseOrientationAsRotationMatrix(Matrix3& orientation) const;
    inline bool getPoseOrientationAsRPY(Vector3& orientation) const;

    inline static const std::string& getPrefix();
};

inline const std::string& wearable::sensor::IPoseSensor::getPrefix()
{
    static std::string prefix{"pose::"};
    return prefix;
}

inline bool wearable::sensor::IPoseSensor::getPose(Vector7& pose) const
{
    Quaternion orientation;
    Vector3 position;
    if (!getPose(orientation, position)) {
        return false;
    }

    pose[0] = orientation[0];
    pose[1] = orientation[1];
    pose[2] = orientation[2];
    pose[3] = orientation[3];
    pose[4] = position[0];
    pose[5] = position[1];
    pose[6] = position[2];

    return true;
}

bool wearable::sensor::IPoseSensor::getPoseOrientationAsQuaternion(Quaternion& orientation) const
{
    Vector3 dummy;
    return getPose(orientation, dummy);
}

inline bool wearable::sensor::IPoseSensor::getPosePosition(Vector3& position) const
{
    Quaternion dummy;
    return getPose(dummy, position);
}

inline bool
wearable::sensor::IPoseSensor::getPoseOrientationAsRotationMatrix(Matrix3& orientation) const
{
    Quaternion quat;
    if (!getPoseOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = utils::quaternionToRotationMatrix(quat);
    return true;
};

inline bool wearable::sensor::IPoseSensor::getPoseOrientationAsRPY(Vector3& orientation) const
{
    Quaternion quat;
    if (!getPoseOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = utils::quaternionToRPY(quat);
    return true;
};

#endif // WEARABLE_IPOSE_SENSOR_H
