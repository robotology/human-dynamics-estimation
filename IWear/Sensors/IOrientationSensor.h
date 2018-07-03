/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IORIENTATIONSENSOR
#define WEAR_IORIENTATIONSENSOR

#include "IWear/Sensors/ISensor.h"

namespace wear {
    namespace sensor {
        class IOrientationSensor;
    }
} // namespace wear

class wear::sensor::IOrientationSensor : public wear::sensor::ISensor
{
public:
    virtual ~IOrientationSensor() = 0;

    virtual bool getOrientationAsQuaternion(wear::Quaternion& orientation) const = 0;
    virtual bool getOrientationAsRPY(wear::Vector3& orientation) const;
    virtual bool getOrientationAsRotationMatrix(wear::Matrix3& orientation) const;
};

#include "IWear/Sensors/IOrientationSensor-defaultImpl.h"
#endif // WEAR_IORIENTATIONSENSOR
