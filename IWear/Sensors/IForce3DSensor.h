/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IFORCE3DSENSOR
#define WEAR_IFORCE3DSENSOR

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class IForce3DSensor;
    }
} // namespace wear

class wear::sensor::IForce3DSensor
{
    virtual ~IForce3DSensor() = 0;

    virtual bool getForce3D(wear::Vector3& force) const = 0;
};

#endif // WEAR_I3DFORCE3DSENSOR
