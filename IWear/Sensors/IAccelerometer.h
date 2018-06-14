/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IACCELEROMETER
#define WEAR_IACCELEROMETER

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class IAccelerometer;
    }
} // namespace wear

class wear::sensor::IAccelerometer
{
public:
    virtual ~IAccelerometer() = 0;
};

#endif // WEAR_IACCELEROMETER
