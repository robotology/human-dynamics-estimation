/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IFORCETORQUE6DSENSOR
#define WEAR_IFORCETORQUE6DSENSOR

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class IForceTorque6DSensor;
    }
} // namespace wear

class wear::sensor::IForceTorque6DSensor : public wear::sensor::ISensor
{
public:
    virtual ~IForceTorque6DSensor() = default;
};

#endif // WEAR_IFORCETORQUE6DSENSOR
