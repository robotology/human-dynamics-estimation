/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_I6DFORCETORQUE
#define WEAR_I6DFORCETORQUE

#include "ISensor.h"

namespace wear {
    namespace sensor {
        class I6DForceTorque;
    }
} // namespace wear

class wear::sensor::I6DForceTorque : public wear::sensor::ISensor
{
public:
    virtual ~I6DForceTorque() = default;
};

#endif // WEAR_I6DFORCETORQUE
