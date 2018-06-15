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

bool wear::sensor::IOrientationSensor::getOrientationAsRPY(wear::Vector3& orientation) const
{
    wear::Quaternion quat;
    if (!wear::sensor::IOrientationSensor::getOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wear::utils::quaternionToRPY(quat);
    return true;
};

bool wear::sensor::IOrientationSensor::getOrientationAsRotationMatrix(
    wear::Matrix3& orientation) const
{
    wear::Quaternion quat;
    if (!wear::sensor::IOrientationSensor::getOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wear::utils::quaterionToRotationMatrix(quat);
    return true;
};

#endif // WEAR_IORIENTATIONSENSOR_DEFAULT_IMPL
