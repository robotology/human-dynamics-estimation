/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IMAGNETOMETER
#define WEAR_IMAGNETOMETER

#include "IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IMagnetometer;
    }
} // namespace wearable

class wearable::sensor::IMagnetometer : public wearable::sensor::ISensor
{
public:
    virtual ~IMagnetometer() = 0;

    virtual bool getMagneticField(wearable::Vector3& magneticField) const = 0;
};

#endif // WEAR_IMAGNETOMETER
