/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IORIENTATIONSENSOR
#define WEAR_IORIENTATIONSENSOR

#include "Wearable/IWear/Sensors/ISensor.h"
#include "Wearable/IWear/Sensors/Utils.h"

namespace wearable {
    namespace sensor {
        class IOrientationSensor;
    }
} // namespace wearable

class wearable::sensor::IOrientationSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IOrientationSensor() = 0;
    virtual bool getOrientationAsQuaternion(wearable::Quaternion& orientation) const = 0;

    inline bool getOrientationAsRPY(wearable::Vector3& orientation) const;
    inline bool getOrientationAsRotationMatrix(wearable::Matrix3& orientation) const;
};

inline bool
wearable::sensor::IOrientationSensor::getOrientationAsRPY(wearable::Vector3& orientation) const
{
    wearable::Quaternion quat;
    if (!wearable::sensor::IOrientationSensor::getOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wearable::utils::quaternionToRPY(quat);
    return true;
};

inline bool wearable::sensor::IOrientationSensor::getOrientationAsRotationMatrix(
    wearable::Matrix3& orientation) const
{
    wearable::Quaternion quat;
    if (!wearable::sensor::IOrientationSensor::getOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wearable::utils::quaternionToRotationMatrix(quat);
    return true;
};

#endif // WEAR_IORIENTATIONSENSOR
