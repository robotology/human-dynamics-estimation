/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IVIRTUALLINKKINSENSOR
#define WEAR_IVIRTUALLINKKINSENSOR

#include "IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IVirtualLinkKinSensor;
    }
} // namespace wearable

class wearable::sensor::IVirtualLinkKinSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IVirtualLinkKinSensor() = 0;

    // 6D quantities
    virtual bool getLinkAcceleration(wearable::Vector3& linear, wearable::Vector3& angular) const = 0;
    virtual bool getLinkPose(wearable::Vector3& position, wearable::Quaternion& orientation) const = 0;
    virtual bool getLinkVelocity(wearable::Vector3& linear, wearable::Vector3& angular) const = 0;

    // 3D quantities
    virtual bool getLinkAngularAcceleration(wearable::Vector3& linear) const = 0;
    virtual bool getLinkAngularVelocity(wearable::Vector3& linear) const = 0;
    virtual bool getLinkLinearAcceleration(wearable::Vector3& linear) const = 0;
    virtual bool getLinkLinearVelocity(wearable::Vector3& linear) const = 0;
    virtual bool getLinkOrientation(wearable::Quaternion& orientation) const = 0;
    virtual bool getLinkPosition(wearable::Vector3& position) const = 0;
};

#endif // WEAR_IVIRTUALLINKKINSENSOR
