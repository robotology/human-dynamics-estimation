// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_IHEATER_H
#define WEARABLE_IHEATER_H

#include "Wearable/IWear/Actuators/IActuator.h"

namespace wearable {
    namespace actuator {
        class IHeater;
    } // namespace actuator
} // namespace wearable

class wearable::actuator::IHeater : public wearable::actuator::IActuator
{
public:
    IHeater(ActuatorName aName = {},
            ActuatorStatus aStatus = ActuatorStatus::Unknown)
        : IActuator(aName, aStatus)
    {
        m_type = ActuatorType::Heater;
    }

    virtual ~IHeater() = default;

    inline static const std::string getPrefix();

    //TODO: Update set and get methods
};

inline const std::string wearable::actuator::IHeater::getPrefix()
{
    return "heater" + wearable::Separator;
}

#endif // WEARABLE_IHEATER_H
