/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IACCELEROMETER
#define WEAR_IACCELEROMETER

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IAccelerometer;
    }
} // namespace wearable

class wearable::sensor::IAccelerometer : public wearable::sensor::ISensor
{
public:
    virtual ~IAccelerometer() = 0;

    virtual bool getLinearAcceleration(wearable::Vector3& linearAcceleration) const = 0;
};

#endif // WEAR_IACCELEROMETER
