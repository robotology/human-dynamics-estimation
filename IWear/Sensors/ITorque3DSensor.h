/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_I3DTORQUESENSOR
#define WEAR_I3DTORQUESENSOR

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class ITorque3DSensor;
    }
} // namespace wear

class wear::sensor::ITorque3DSensor : public wear::sensor::ISensor
{
    virtual ~ITorque3DSensor() = default;

    virtual bool getTx(double& tx) const = 0;
    virtual bool getTy(double& ty) const = 0;
    virtual bool getTz(double& tz) const = 0;
    virtual bool get3DTorque(wear::Vector3& torque) const;
};

bool wear::sensor::ITorque3DSensor::get3DTorque(wear::Vector3& torque) const
{
    double tx = 0, ty = 0, tz = 0;
    const bool ok = getTx(tx) && getTy(ty) && getTz(tz);
    torque = {{tx, ty, tz}};
    return ok;
}

#endif // WEAR_I3DTORQUESENSOR
