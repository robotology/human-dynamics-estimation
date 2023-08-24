// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IHAPTIC_H
#define WEARABLE_IHAPTIC_H

#include "Wearable/IWear/Actuators/IActuator.h"

namespace wearable {
    namespace actuator {
        class IHaptic;
    } // namespace actuator
} // namespace wearable

class wearable::actuator::IHaptic : public wearable::actuator::IActuator
{
public:
    IHaptic(ActuatorName aName = {},
            ActuatorStatus aStatus = ActuatorStatus::Unknown)
        : IActuator(aName, aStatus)
    {
        m_type = ActuatorType::Haptic;
    }

    virtual ~IHaptic() = default;

    inline static const std::string getPrefix();

    virtual bool setHapticCommand(double& value) const = 0;
};

inline const std::string wearable::actuator::IHaptic::getPrefix()
{
    return "haptic" + wearable::Separator;
}

#endif // WEARABLE_IHAPTIC_H
