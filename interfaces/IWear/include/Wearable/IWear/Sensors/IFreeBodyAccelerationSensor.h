/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IFREE_BODY_ACCELERATION_SENSOR_H
#define WEARABLE_IFREE_BODY_ACCELERATION_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IFreeBodyAccelerationSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IFreeBodyAccelerationSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IFreeBodyAccelerationSensor() = 0;

    virtual bool getFreeBodyAcceleration(Vector3& freeBodyAcceleration) const = 0;
};

#endif // WEARABLE_IFREE_BODY_ACCELERATION_SENSOR_H
