/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IFREEBODYACCELERATIONSENSOR
#define WEAR_IFREEBODYACCELERATIONSENSOR

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IFreeBodyAccelerationSensor;
    }
} // namespace wearable

class wearable::sensor::IFreeBodyAccelerationSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IFreeBodyAccelerationSensor() = 0;

    virtual bool getFreeBodyAcceleration(wearable::Vector3& freeBodyAcceleration) const = 0;
};

#endif // WEAR_IFREEBODYACCELERATIONSENSOR
