/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IVIRTUALSPHERICALJOINTKINSENSOR
#define WEAR_IVIRTUALSPHERICALJOINTKINSENSOR

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class IVirtualSphericalJointKinSensor;
    }
} // namespace wear

class wear::sensor::IVirtualSphericalJointKinSensor
{
protected:
public:
    virtual ~IVirtualSphericalJointKinSensor() = 0;

    virtual bool getRoll(double& r) const = 0;
    virtual bool getPitch(double& p) const = 0;
    virtual bool getYaw(double& y) const = 0;
    virtual bool getRPY(wear::Vector3& rpy) const;
};

bool wear::sensor::IVirtualSphericalJointKinSensor::getRPY(wear::Vector3& rpy) const
{
    double r = 0, p = 0, y = 0;
    const bool ok = getRoll(r) && getPitch(p) && getYaw(y);
    rpy = {{r, p, y}};
    return ok;
}

#endif // WEAR_IVIRTUALSPHERICALJOINTKINSENSOR
