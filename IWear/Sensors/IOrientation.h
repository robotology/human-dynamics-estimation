/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IORIENTATION
#define WEAR_IORIENTATION

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class IOrientation;
    }
} // namespace wear

class wear::sensor::IOrientation : public wear::sensor::ISensor
{
public:
    virtual ~IOrientation() = default;
};

#endif // WEAR_IORIENTATION
