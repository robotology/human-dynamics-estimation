/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IEMGSENSOR
#define WEAR_IEMGSENSOR

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class IEmgSensor;
    }
} // namespace wear

class wear::sensor::IEmgSensor
{
public:
    virtual ~IEmgSensor() = 0;

};

#endif // WEAR_IEMGSENSOR
