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

namespace wear {
    namespace sensor {
        class IVirtualLinkKinSensor;
    }
} // namespace wear

class wear::sensor::IVirtualLinkKinSensor : public wear::sensor::ISensor
{
public:
    virtual ~IVirtualLinkKinSensor() = 0;

    // 6D quantities
    virtual bool getLinkAcceleration(wear::Vector3& linear, wear::Vector3& angular) const = 0;
    virtual bool getLinkPose(wear::Vector3& position, wear::Quaternion& orientation) const = 0;
    virtual bool getLinkVelocity(wear::Vector3& linear, wear::Vector3& angular) const = 0;

    // 3D quantities
    virtual bool getLinkAngularAcceleration(wear::Vector3& linear) const = 0;
    virtual bool getLinkAngularVelocity(wear::Vector3& linear) const = 0;
    virtual bool getLinkLinearAcceleration(wear::Vector3& linear) const = 0;
    virtual bool getLinkLinearVelocity(wear::Vector3& linear) const = 0;
    virtual bool getLinkOrientation(wear::Quaternion& orientation) const = 0;
    virtual bool getLinkPosition(wear::Vector3& position) const = 0;
};

#endif // WEAR_IVIRTUALLINKKINSENSOR
