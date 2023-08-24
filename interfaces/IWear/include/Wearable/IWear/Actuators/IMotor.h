// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IMOTOR_H
#define WEARABLE_IMOTOR_H

#include "Wearable/IWear/Actuators/IActuator.h"

namespace wearable {
    namespace actuator {
        class IMotor;
    } // namespace actuator
} // namespace wearable

class wearable::actuator::IMotor : public wearable::actuator::IActuator
{
public:
    IMotor(ActuatorName aName = {},
           ActuatorStatus aStatus = ActuatorStatus::Unknown)
        : IActuator(aName, aStatus)
    {
        m_type = ActuatorType::Motor;
    }

    virtual ~IMotor() = default;

    inline static const std::string getPrefix();

    virtual bool setMotorPosition(double& value) const = 0; //TODO: Double check if rads or deg
};

inline const std::string wearable::actuator::IMotor::getPrefix()
{
    return "motor" + wearable::Separator;
}

#endif // WEARABLE_IMOTOR_H
