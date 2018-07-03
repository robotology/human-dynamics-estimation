/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_ITORQUE3DSENSOR
#define WEAR_ITORQUE3DSENSOR

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class ITorque3DSensor;
    }
} // namespace wear

class wear::sensor::ITorque3DSensor : public wear::sensor::ISensor
{
    virtual ~ITorque3DSensor() = 0;

    virtual bool getTorque3D(wear::Vector3& torque) const = 0;
};

#endif // WEAR_ITORQUE3DSENSOR
