/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IGYROSCOPE_H
#define WEARABLE_IGYROSCOPE_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IGyroscope;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IGyroscope : public wearable::sensor::ISensor
{
public:
    IGyroscope(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::Gyroscope;
    }

    virtual ~IGyroscope() = default;

    virtual bool getAngularRate(Vector3& angularRate) const = 0;
};

#endif // WEARABLE_IGYROSCOPE_H
