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
} // namespace wear

class wear::sensor::ISensor
{
public:
    virtual ~ISensor() = 0;

    // TODO: timestamp? sequence number?
    virtual wear::sensor::SensorName getSensorName() const = 0;
    virtual wear::sensor::SensorStatus getSensorStatus() const = 0;
    virtual wear::sensor::SensorType getSensorType() const = 0;
};

#endif // WEAR_ISENSOR
