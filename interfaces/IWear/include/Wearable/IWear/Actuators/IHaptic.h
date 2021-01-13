/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

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
    {
        m_type = ActuatorType::Haptic;
    }

    virtual ~IHaptic() = default;

    inline static const std::string getPrefix();

    //TODO: Update set and get methods
};

inline const std::string wearable::actuator::IHaptic::getPrefix()
{
    return "hap" + wearable::Separator;
}

#endif // WEARABLE_IHAPTIC_H
