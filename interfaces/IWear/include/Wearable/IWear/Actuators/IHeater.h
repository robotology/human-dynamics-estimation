/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

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
