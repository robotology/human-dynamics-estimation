/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_ISKIN
#define WEAR_ISKIN

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class ISkin;
    }
} // namespace wear

class wear::sensor::ISkin : public wear::sensor::ISensor
{
public:
    virtual ~ISkin() = default;
};

#endif // WEAR_ISKIN
