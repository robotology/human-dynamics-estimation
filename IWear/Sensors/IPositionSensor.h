/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IPOSITIONSENSOR
#define WEAR_IPOSITIONSENSOR

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class IPositionSensor;
    }
} // namespace wear

class wear::sensor::IPositionSensor : public wear::sensor::ISensor
{
public:
    virtual ~IPositionSensor() = 0;

    virtual bool getPosition(wear::Vector3& position) const = 0;
};

#endif // WEAR_IPOSITIONSENSOR
