/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IFREEBODYACCELERATION
#define WEAR_IFREEBODYACCELERATION

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class IFreeBodyAcceleration;
    }
} // namespace wear

class wear::sensor::IFreeBodyAcceleration : public wear::sensor::ISensor
{
public:
    virtual ~IFreeBodyAcceleration() = default;
};

#endif // WEAR_IFREEBODYACCELERATION
