/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_I3DFORCE
#define WEAR_I3DFORCE

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class I3DForce;
    }
} // namespace wear

class wear::sensor::I3DForce : public wear::sensor::ISensor
{
    virtual ~I3DForce() = default;

    virtual bool getFx(double& fx) const = 0;
    virtual bool getFy(double& fy) const = 0;
    virtual bool getFz(double& fz) const = 0;
    virtual bool get3DForce(wear::Vector3& force) const;
};

bool wear::sensor::I3DForce::get3DForce(wear::Vector3& force) const
{
    double fx = 0, fy = 0, fz = 0;
    const bool ok = getFx(fx) && getFy(fy) && getFz(fz);
    force = {{fx, fy, fz}};
    return ok;
}

#endif // WEAR_I3DFORCE
