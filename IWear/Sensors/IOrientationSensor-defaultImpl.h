/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IORIENTATIONSENSOR_DEFAULT_IMPL
#define WEAR_IORIENTATIONSENSOR_DEFAULT_IMPL

#include "IWear/Sensors/IOrientationSensor.h"
#include "IWear/Sensors/ISensor.h"

#include "IWear/Utils.h"

bool wearable::sensor::IOrientationSensor::getOrientationAsRPY(wearable::Vector3& orientation) const
{
    wearable::Quaternion quat;
    if (!wearable::sensor::IOrientationSensor::getOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wearable::utils::quaternionToRPY(quat);
    return true;
};

bool wearable::sensor::IOrientationSensor::getOrientationAsRotationMatrix(
    wearable::Matrix3& orientation) const
{
    wearable::Quaternion quat;
    if (!wearable::sensor::IOrientationSensor::getOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wearable::utils::quaternionToRotationMatrix(quat);
    return true;
};

#endif // WEAR_IORIENTATIONSENSOR_DEFAULT_IMPL
