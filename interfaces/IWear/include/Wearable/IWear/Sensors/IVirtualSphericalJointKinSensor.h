// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IVIRTUAL_SPHERICAL_JOINT_KIN_SENSOR_H
#define WEARABLE_IVIRTUAL_SPHERICAL_JOINT_KIN_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IVirtualSphericalJointKinSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IVirtualSphericalJointKinSensor : public wearable::sensor::ISensor
{
public:
    IVirtualSphericalJointKinSensor(SensorName aName = {},
                                    SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::VirtualSphericalJointKinSensor;
    }

    virtual ~IVirtualSphericalJointKinSensor() = default;

    // CONVENTION: RPY are defined as rotations about X-Y-Z wrt fixed frame
    virtual bool getJointAnglesAsRPY(Vector3& angleAsRPY) const = 0;
    virtual bool getJointVelocities(Vector3& velocities) const = 0;
    virtual bool getJointAccelerations(Vector3& accelerations) const = 0;

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::IVirtualSphericalJointKinSensor::getPrefix()
{
    return "vSJoint" + wearable::Separator;
}

#endif // WEARABLE_IVIRTUAL_SPHERICAL_JOINT_KIN_SENSOR_H
