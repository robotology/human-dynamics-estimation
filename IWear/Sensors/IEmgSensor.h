/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IEMGSENSOR
#define WEAR_IEMGSENSOR

#include "IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IEmgSensor;
    }
} // namespace wearable

class wearable::sensor::IEmgSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IEmgSensor() = 0;

    virtual bool getEmgSignal(double& emgSignal) const = 0;
    virtual bool getNormalizationValue(double& normalizationValue) const = 0;
};

#endif // WEAR_IEMGSENSOR
