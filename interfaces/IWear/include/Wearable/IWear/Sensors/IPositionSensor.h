/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IPOSITIONSENSOR
#define WEAR_IPOSITIONSENSOR

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IPositionSensor;
    }
} // namespace wearable

class wearable::sensor::IPositionSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IPositionSensor() = 0;

    virtual bool getPosition(wearable::Vector3& position) const = 0;
};

#endif // WEAR_IPOSITIONSENSOR
