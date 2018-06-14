/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IPOSESENSOR
#define WEAR_IPOSESENSOR

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class IPoseSensor;
    }
} // namespace wear

class wear::sensor::IPoseSensor
{
public:
    virtual ~IPoseSensor() = 0;

};

#endif // WEAR_IPOSITIONSENSOR
