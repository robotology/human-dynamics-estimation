/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_IVIRTUAL_LINK_KIN_SENSOR_H
#define WEARABLE_IVIRTUAL_LINK_KIN_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IVirtualLinkKinSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IVirtualLinkKinSensor : public wearable::sensor::ISensor
{
public:
    IVirtualLinkKinSensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::VirtualLinkKinSensor;
    }

    virtual ~IVirtualLinkKinSensor() = default;

    // 6D quantities
    virtual bool getLinkAcceleration(Vector3& linear, Vector3& angular) const = 0;
    virtual bool getLinkPose(Vector3& position, Quaternion& orientation) const = 0;
    virtual bool getLinkVelocity(Vector3& linear, Vector3& angular) const = 0;

    // 3D quantities
    inline bool getLinkAngularAcceleration(Vector3& angularAcceleration) const;
    inline bool getLinkAngularVelocity(Vector3& angularVelocity) const;
    inline bool getLinkLinearAcceleration(Vector3& linearAcceleration) const;
    inline bool getLinkLinearVelocity(Vector3& linearVelocity) const;
    inline bool getLinkOrientation(Quaternion& orientation) const;
    inline bool getLinkPosition(Vector3& position) const;

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::IVirtualLinkKinSensor::getPrefix()
{
    return "vLink" + wearable::Separator;
}

inline bool wearable::sensor::IVirtualLinkKinSensor::getLinkAngularAcceleration(
    Vector3& angularAcceleration) const
{
    Vector3 dummy;
    return getLinkAcceleration(dummy, angularAcceleration);
}

inline bool wearable::sensor::IVirtualLinkKinSensor::getLinkLinearAcceleration(
    Vector3& linearAcceleration) const
{
    Vector3 dummy;
    return getLinkAcceleration(linearAcceleration, dummy);
}

inline bool
wearable::sensor::IVirtualLinkKinSensor::getLinkAngularVelocity(Vector3& angularVelocity) const
{
    Vector3 dummy;
    return getLinkVelocity(dummy, angularVelocity);
}

inline bool
wearable::sensor::IVirtualLinkKinSensor::getLinkLinearVelocity(Vector3& linearVelocity) const
{
    Vector3 dummy;
    return getLinkVelocity(linearVelocity, dummy);
}

inline bool
wearable::sensor::IVirtualLinkKinSensor::getLinkOrientation(Quaternion& orientation) const
{
    Vector3 dummy;
    return getLinkPose(dummy, orientation);
}

inline bool wearable::sensor::IVirtualLinkKinSensor::getLinkPosition(Vector3& position) const
{
    Quaternion dummy;
    return getLinkPose(position, dummy);
}

#endif // WEARABLE_IVIRTUAL_LINK_KIN_SENSOR_H
