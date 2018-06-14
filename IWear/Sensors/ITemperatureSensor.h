/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_ITEMPERATURESENSOR
#define WEAR_ITEMPERATURESENSOR

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class ITemperatureSensor;
    }
} // namespace wear

class wear::sensor::ITemperatureSensor
{
public:
    virtual ~ITemperatureSensor() = 0;

    virtual bool getTemperature(double& temperature) const = 0;
};

#endif // WEAR_ITEMPERATURESENSOR
