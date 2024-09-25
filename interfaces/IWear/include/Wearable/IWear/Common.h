// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_COMMON_H
#define WEARABLE_COMMON_H

#ifndef wError
#include <iostream>
#define wError std::cerr
#endif

#ifndef wWarning
#include <iostream>
#define wWarning std::cout
#endif

#include <string>

namespace wearable {

    const std::string Separator = "::";

    enum class ElementType
    {
        WearableSensor = 0,
        WearableActuator,
    };

    class IWearableDevice;

} // namespace wearable

class wearable::IWearableDevice
{
protected:
    ElementType m_wearable_element_type;

    virtual ElementType getWearableElementType() const = 0;
};

#endif // WEARABLE_COMMON_H
