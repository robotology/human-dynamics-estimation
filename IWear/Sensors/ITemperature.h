/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_ITEMPERATURE
#define WEAR_ITEMPERATURE

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class ITemperature;
    }
} // namespace wear

class wear::sensor::ITemperature : public wear::sensor::ISensor
{
public:
    virtual ~ITemperature() = default;
};

#endif // WEAR_ITEMPERATURE
