// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IVIRTUAL_JOINT_KIN_SENSOR_H
#define WEARABLE_IVIRTUAL_JOINT_KIN_SENSOR_H

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace sensor {
        class IVirtualJointKinSensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::IVirtualJointKinSensor : public wearable::sensor::ISensor
{
public:
    IVirtualJointKinSensor(SensorName aName = {},
                           SensorStatus aStatus = SensorStatus::Unknown)
        : ISensor(aName, aStatus)
    {
        m_type = SensorType::VirtualJointKinSensor;
    }

    virtual bool getJointPosition(double& position) const = 0;
    virtual bool getJointVelocity(double& velocity) const = 0;
    virtual bool getJointAcceleration(double& acceleration) const = 0;

    inline static const std::string getPrefix();
};

inline const std::string wearable::sensor::IVirtualJointKinSensor::getPrefix()
{
    return "vJoint" + wearable::Separator;
}


#endif // WEARABLE_IVIRTUAL_JOINT_KIN_SENSOR_H
