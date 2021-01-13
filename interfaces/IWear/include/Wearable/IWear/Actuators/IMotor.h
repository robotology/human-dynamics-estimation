/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

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
