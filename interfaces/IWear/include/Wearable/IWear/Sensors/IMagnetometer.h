/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IMAGNETOMETER_H
#define WEARABLE_IMAGNETOMETER_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IMagnetometer;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IMagnetometer : public wearable::sensor::ISensor
{
public:
    IMagnetometer(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::Magnetometer;
    }

    virtual ~IMagnetometer() = 0;

    virtual bool getMagneticField(Vector3& magneticField) const = 0;
};

#endif // WEARABLE_IMAGNETOMETER_H
