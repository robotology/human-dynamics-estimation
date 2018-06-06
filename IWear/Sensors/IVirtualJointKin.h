/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IVIRTUALSPHERICALJOINTKIN
#define WEAR_IVIRTUALSPHERICALJOINTKIN

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class IVirtualSphericalJointKin;
    }
} // namespace wear

class wear::sensor::IVirtualSphericalJointKin : public wear::sensor::ISensor
{
protected:
    using JointName = std::string;
    JointName m_name;
    // TODO: parent / child links?

public:
    virtual ~IVirtualSphericalJointKin() = default;

    virtual bool getRoll(double& r) const = 0;
    virtual bool getPitch(double& p) const = 0;
    virtual bool getYaw(double& y) const = 0;
    virtual bool getRPY(wear::Vector3& rpy) const;

    virtual JointName getJointName() const;
};

bool wear::sensor::IVirtualSphericalJointKin::getRPY(wear::Vector3& rpy) const
{
    double r = 0, p = 0, y = 0;
    const bool ok = getRoll(r) && getPitch(p) && getYaw(y);
    rpy = {{r, p, y}};
    return ok;
}

wear::sensor::IVirtualSphericalJointKin::JointName
wear::sensor::IVirtualSphericalJointKin::getJointName() const
{
    return m_name;
}

#endif // WEAR_IVIRTUALSPHERICALJOINTKIN
