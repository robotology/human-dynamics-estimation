/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IVIRTUAL_LINK_KIN_SENSOR_H
#define WEARABLE_IVIRTUAL_LINK_KIN_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IVirtualLinkKinSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IVirtualLinkKinSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IVirtualLinkKinSensor() = 0;

    // 6D quantities
    virtual bool getLinkAcceleration(Vector3& linear, Vector3& angular) const = 0;
    virtual bool getLinkPose(Vector3& position, Quaternion& orientation) const = 0;
    virtual bool getLinkVelocity(Vector3& linear, Vector3& angular) const = 0;

    // 3D quantities
    virtual bool getLinkAngularAcceleration(Vector3& angularAcceleration) const = 0;
    virtual bool getLinkAngularVelocity(Vector3& angularVelocity) const = 0;
    virtual bool getLinkLinearAcceleration(Vector3& linearAcceleration) const = 0;
    virtual bool getLinkLinearVelocity(Vector3& linearVelocity) const = 0;
    virtual bool getLinkOrientation(Quaternion& orientation) const = 0;
    virtual bool getLinkPosition(Vector3& position) const = 0;
};

#endif // WEARABLE_IVIRTUAL_LINK_KIN_SENSOR_H
