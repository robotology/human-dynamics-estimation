/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IGYROSCOPE
#define WEAR_IGYROSCOPE

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IGyroscope;
    }
} // namespace wearable

class wearable::sensor::IGyroscope : public wearable::sensor::ISensor
{
public:
    virtual ~IGyroscope() = 0;

    virtual bool getAngularRate(wearable::Vector3& angularRate) const = 0;
};

#endif // WEAR_IGYROSCOPE
