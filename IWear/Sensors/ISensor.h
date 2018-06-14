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
    using Vector3 = std::array<double, 3>;
    using Quaternion = std::array<double, 4>;

    namespace sensor {
        using SensorName = std::string;

        enum class SensorType
        {
            Accelerometer,
            EmgSensor,
            Force3DSensor,
            ForceTorque6DSensor,
            FreeBodyAccelerationSensor,
            Gyroscope,
            Magnetometer,
            OrientationSensor,
            PoseSensor,
            PositionSensor,
            SkinSensor,
            TemperatureSensor,
            Torque3DSensor,
            VirtualLinkKinSensor,
            VirtualSphericalJointKinSensor
        };

        class ISensor;
    } // namespace sensor
} // namespace wear

class wear::sensor::ISensor
{
protected:
    wear::sensor::SensorType m_sensorType;
    wear::sensor::SensorName m_sensorName;

public:
    virtual ~ISensor() = 0;

    virtual wear::sensor::SensorType getSensorType() const;
    virtual const wear::sensor::SensorName getSensorName() const;
};

const wear::sensor::SensorName wear::sensor::ISensor::getSensorName() const
{
    return m_sensorName;
}

wear::sensor::SensorType wear::sensor::ISensor::getSensorType() const
{
    return m_sensorType;
}

#endif // WEAR_ISENSOR
