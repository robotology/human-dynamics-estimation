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

class wear::sensor::IVirtualLinkKinSensor
{
public:
    virtual ~IVirtualLinkKinSensor() = 0;

    virtual bool getPose(wear::Vector3& position, wear::Quaternion& orientation) const = 0;
    virtual bool getVelocity(wear::Vector3& linear, wear::Vector3& angular) const = 0;
    virtual bool getAcceleration(wear::Vector3& linear, wear::Vector3& angular) const = 0;
    virtual bool getOrientation(wear::Quaternion& orientation) const = 0;
    virtual bool getPosition(wear::Vector3& position) const = 0;
    virtual bool getLinearVelocity(wear::Vector3& linear) const = 0;
    virtual bool getLinearAcceleration(wear::Vector3& linear) const = 0;
    virtual bool getAngularVelocity(wear::Vector3& linear) const = 0;
    virtual bool getAngularAcceleration(wear::Vector3& linear) const = 0;
};

#endif // WEAR_IVIRTUALLINKKINSENSOR
