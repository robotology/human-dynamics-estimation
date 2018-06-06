/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_ISENSOR
#define WEAR_ISENSOR

#include <array>
#include <string>

namespace wear {
    using Type = std::string;
    using FrameName = std::string;
    using SensorName = std::string;

    using Vector3 = std::array<double, 3>;
    using Quaternion = std::array<double, 4>;
    using SensorName = std::string;

    namespace sensor {
        class ISensor;
    }
} // namespace wear

class wear::sensor::ISensor
{
protected:
    wear::Type m_type;
    wear::FrameName m_frameName;
    wear::SensorName m_sensorName;

public:
    virtual ~ISensor() = default;
};

#endif // WEAR_ISENSOR
