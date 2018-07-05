/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IPOSESENSOR
#define WEAR_IPOSESENSOR

#include "Wearable/IWear/Sensors/ISensor.h"
#include "Wearable/IWear/Sensors/Utils.h"

namespace wearable {
    namespace sensor {
        class IPoseSensor;
    }
} // namespace wearable

class wearable::sensor::IPoseSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IPoseSensor() = 0;

    virtual bool getPose(wearable::Quaternion& orientation, wearable::Vector3& position) const = 0;
    virtual bool getPoseOrientationAsQuaternion(wearable::Quaternion& orientation) const = 0;

    inline bool getPose(wearable::Vector7& pose) const;
    inline bool getPosePosition(wearable::Vector3& position) const;
    inline bool getPoseOrientationAsRotationMatrix(wearable::Matrix3& orientation) const;
    inline bool getPoseOrientationAsRPY(wearable::Vector3& orientation) const;
};

inline bool wearable::sensor::IPoseSensor::getPose(wearable::Vector7& pose) const
{
    wearable::Quaternion orientation;
    wearable::Vector3 position;
    if (!wearable::sensor::IPoseSensor::getPose(orientation, position)) {
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

// TODO
inline bool wearable::sensor::IPoseSensor::getPosePosition(wearable::Vector3& position) const {}

inline bool wearable::sensor::IPoseSensor::getPoseOrientationAsRotationMatrix(
    wearable::Matrix3& orientation) const
{
    wearable::Quaternion quat;
    if (!wearable::sensor::IPoseSensor::getPoseOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wearable::utils::quaternionToRotationMatrix(quat);
    return true;
};

inline bool
wearable::sensor::IPoseSensor::getPoseOrientationAsRPY(wearable::Vector3& orientation) const
{
    wearable::Quaternion quat;
    if (!wearable::sensor::IPoseSensor::getPoseOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wearable::utils::quaternionToRPY(quat);
    return true;
};

#endif // WEAR_IPOSITIONSENSOR
