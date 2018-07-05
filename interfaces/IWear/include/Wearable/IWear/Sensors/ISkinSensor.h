/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_ISKINSENSOR
#define WEAR_ISKINSENSOR

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class ISkinSensor;
    }
} // namespace wearable

class wearable::sensor::ISkinSensor : public wearable::sensor::ISensor
{
public:
    virtual ~ISkinSensor() = 0;

    // TODO: to be implemented
};

#endif // WEAR_ISKINSENSOR
