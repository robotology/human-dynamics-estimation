/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IFORCE3DSENSOR
#define WEAR_IFORCE3DSENSOR

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class IForce3DSensor;
    }
} // namespace wear

class wear::sensor::IForce3DSensor : public wear::sensor::ISensor
{
    virtual ~IForce3DSensor() = default;

    virtual bool getFx(double& fx) const = 0;
    virtual bool getFy(double& fy) const = 0;
    virtual bool getFz(double& fz) const = 0;
    virtual bool get3DForce(wear::Vector3& force) const;
};

bool wear::sensor::IForce3DSensor::get3DForce(wear::Vector3& force) const
{
    double fx = 0, fy = 0, fz = 0;
    const bool ok = getFx(fx) && getFy(fy) && getFz(fz);
    force = {{fx, fy, fz}};
    return ok;
}

#endif // WEAR_I3DFORCE3DSENSOR
