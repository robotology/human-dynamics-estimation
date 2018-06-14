/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_ISKINSENSOR
#define WEAR_ISKINSENSOR

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class ISkinSensor;
    }
} // namespace wear

class wear::sensor::ISkinSensor
{
public:
    virtual ~ISkinSensor() = 0;

};

#endif // WEAR_ISKINSENSOR
