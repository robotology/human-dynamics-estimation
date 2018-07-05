/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IFORCETORQUE6DSENSOR
#define WEAR_IFORCETORQUE6DSENSOR

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IForceTorque6DSensor;
    }
} // namespace wearable

class wearable::sensor::IForceTorque6DSensor : public wearable::sensor::ISensor
{
public:
    virtual ~IForceTorque6DSensor() = 0;

    virtual bool gerForceTorque6D(wearable::Vector3& force3D,
                                  wearable::Vector3& torque3D) const = 0;

    // TODO: inline
    virtual bool getForceTorque6D(wearable::Vector6& forceTorque6D) const = 0;
    virtual bool getForceTorque3DForce(wearable::Vector3& force3D) const = 0;
    virtual bool getForceTorque3DTorque(wearable::Vector3& torque3D) const = 0;
};

#endif // WEAR_IFORCETORQUE6DSENSOR
