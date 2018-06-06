/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_I3DTORQUE
#define WEAR_I3DTORQUE

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class I3DTorque;
    }
} // namespace wear

class wear::sensor::I3DTorque : public wear::sensor::ISensor
{
    virtual ~I3DTorque() = default;

    virtual bool getTx(double& tx) const = 0;
    virtual bool getTy(double& ty) const = 0;
    virtual bool getTz(double& tz) const = 0;
    virtual bool get3DTorque(wear::Vector3& torque) const;
};

bool wear::sensor::I3DTorque::get3DTorque(wear::Vector3& torque) const
{
    double tx = 0, ty = 0, tz = 0;
    const bool ok = getTx(tx) && getTy(ty) && getTz(tz);
    torque = {{tx, ty, tz}};
    return ok;
}

#endif // WEAR_I3DTORQUE
