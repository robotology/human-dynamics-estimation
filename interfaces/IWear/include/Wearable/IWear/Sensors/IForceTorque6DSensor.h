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
    IForceTorque6DSensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::ForceTorque6DSensor;
    }

    virtual ~IForceTorque6DSensor() = default;

    virtual bool getForceTorque6D(Vector3& force3D, Vector3& torque3D) const = 0;

    inline bool getForceTorque6D(Vector6& forceTorque6D) const;
    inline bool getForceTorque3DForce(Vector3& force3D) const;
    inline bool getForceTorque3DTorque(Vector3& torque3D) const;

    inline static const std::string& getPrefix();
};

inline const std::string& wearable::sensor::IForceTorque6DSensor::getPrefix()
{
    static std::string prefix{"ft6D_"};
    return prefix;
}
bool wearable::sensor::IForceTorque6DSensor::getForceTorque6D(
    wearable::Vector6& forceTorque6D) const
{
    wearable::Vector3 force3D, torque3D;
    const bool ok = getForceTorque6D(force3D, torque3D);
    forceTorque6D = {force3D[0], force3D[1], force3D[2], torque3D[0], torque3D[1], torque3D[2]};
    return ok;
}

bool wearable::sensor::IForceTorque6DSensor::getForceTorque3DForce(wearable::Vector3& force3D) const
{
    wearable::Vector3 dummy;
    return getForceTorque6D(force3D, dummy);
}

bool wearable::sensor::IForceTorque6DSensor::getForceTorque3DTorque(
    wearable::Vector3& torque3D) const
{
    wearable::Vector3 dummy;
    return getForceTorque6D(dummy, torque3D);
}

#endif // WEARABLE_IFORCE_TORQUE_6D_SENSOR_H
