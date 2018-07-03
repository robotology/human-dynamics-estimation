/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IPOSESENSOR_DEFAULT_IMPL
#define WEAR_IPOSESENSOR_DEFAULT_IMPL

#include "IWear/Sensors/IPoseSensor.h"
#include "IWear/Sensors/ISensor.h"

#include "IWear/Utils.h"

bool wear::sensor::IPoseSensor::getPose(wear::Vector7& pose) const
{

    wear::Quaternion orientation;
    wear::Vector3 position;
    if (!wear::sensor::IPoseSensor::getPose(orientation, position)) {
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

bool wear::sensor::IPoseSensor::getPoseOrientationAsRotationMatrix(wear::Matrix3& orientation) const
{
    wear::Quaternion quat;
    if (!wear::sensor::IPoseSensor::getPoseOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wear::utils::quaterionToRotationMatrix(quat);
    return true;
};

bool wear::sensor::IPoseSensor::getPoseOrientationAsRPY(wear::Vector3& orientation) const
{
    wear::Quaternion quat;
    if (!wear::sensor::IPoseSensor::getPoseOrientationAsQuaternion(quat)) {
        return false;
    }
    orientation = wear::utils::quaternionToRPY(quat);
    return true;
};

#endif // WEAR_IPOSESENSOR_DEFAULT_IMPL
