/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IFORCE3DSENSOR
#define WEAR_IFORCE3DSENSOR

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IForce3DSensor;
    }
} // namespace wearable

class wearable::sensor::IForce3DSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IForce3DSensor() = 0;

    virtual bool getForce3D(wearable::Vector3& force) const = 0;
};

#endif // WEAR_I3DFORCE3DSENSOR
