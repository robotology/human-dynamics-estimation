/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARABLE_ISENSOR_H
#define WEARABLE_ISENSOR_H

#include <array>
#include <string>

namespace wearable {

    using Vector3 = std::array<double, 3>;
    using Vector6 = std::array<double, 6>;
    using Vector7 = std::array<double, 7>;
    using Matrix3 = std::array<Vector3, 3>; // TODO: think about it, this force RowMajor
    using Quaternion = std::array<double, 4>;

    namespace sensor {
        using SensorName = std::string;

        enum class SensorStatus
        {
            Error = 0,
            Ok,
            Calibrating,
            Overflow,
            Timeout,
            Unknown,
            WaitingForFirstRead,
        };

        enum class SensorType
        {
            Accelerometer = 0,
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
            VirtualSphericalJointKinSensor,
        };

        class ISensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::ISensor
{
protected:
    SensorName m_name;
    SensorType m_type;
    SensorStatus m_status;

public:
    ISensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : m_name{aName}
        , m_status{aStatus}
    {}

    virtual ~ISensor() = default;

    // TODO: timestamp? sequence number?
    // TODO: virtual?
    inline SensorName getSensorName() const;
    inline SensorStatus getSensorStatus() const;
    inline SensorType getSensorType() const;
};

inline wearable::sensor::SensorName wearable::sensor::ISensor::getSensorName() const
{
    return m_name;
}

inline wearable::sensor::SensorType wearable::sensor::ISensor::getSensorType() const
{
    return m_type;
}

inline wearable::sensor::SensorStatus wearable::sensor::ISensor::getSensorStatus() const
{
    return m_status;
}

#endif // WEARABLE_ISENSOR_H
