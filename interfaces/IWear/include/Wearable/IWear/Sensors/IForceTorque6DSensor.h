/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IFORCE_TORQUE_6D_SENSOR_H
#define WEARABLE_IFORCE_TORQUE_6D_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IForceTorque6DSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IForceTorque6DSensor : public wearable::sensor::ISensor
{
public:
    IForceTorque6DSensor() { m_type = SensorType::ForceTorque6DSensor; }
    virtual ~IForceTorque6DSensor() = 0;

    virtual bool gerForceTorque6D(Vector3& force3D, Vector3& torque3D) const = 0;

    // TODO: inline
    virtual bool getForceTorque6D(Vector6& forceTorque6D) const = 0;
    virtual bool getForceTorque3DForce(Vector3& force3D) const = 0;
    virtual bool getForceTorque3DTorque(Vector3& torque3D) const = 0;
};

#endif // WEARABLE_IFORCE_TORQUE_6D_SENSOR_H
