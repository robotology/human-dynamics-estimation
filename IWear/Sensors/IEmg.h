/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IEMG
#define WEAR_IEMG

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class IEmg;
    }
} // namespace wear

class wear::sensor::IEmg : public wear::sensor::ISensor
{
public:
    virtual ~IEmg() = default;
};

#endif // WEAR_IEMG
