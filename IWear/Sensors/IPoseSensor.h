/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IPOSESENSOR
#define WEAR_IPOSESENSOR

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IPoseSensor;
    }
} // namespace wearable

class wearable::sensor::IPoseSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IPoseSensor() = 0;

    virtual bool getPose(wearable::Quaternion& orientation, wearable::Vector3& position) const = 0;
    virtual bool getPose(wearable::Vector7& pose) const;

    virtual bool getPoseOrientationAsQuaternion(wearable::Quaternion& orientation) const = 0;
    virtual bool getPosePosition(wearable::Vector3& position) const = 0;

    virtual bool getPoseOrientationAsRotationMatrix(wearable::Matrix3& orientation) const;
    virtual bool getPoseOrientationAsRPY(wearable::Vector3& orientation) const;
};

#include "Wearable/IWear/Sensors/IPoseSensor-defaultImpl.h"

#endif // WEAR_IPOSITIONSENSOR
