// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_ISENSOR_H
#define WEARABLE_ISENSOR_H

#include "Wearable/IWear/Common.h"

#include <array>
#include <atomic>

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
            VirtualJointKinSensor,
            VirtualSphericalJointKinSensor,
            Invalid
        };

        inline SensorType sensorTypeFromString(std::string sensorTypeString)
        {
            if (sensorTypeString == "Accelerometer")
                return SensorType::Accelerometer;
            else if (sensorTypeString == "EmgSensor")
                return SensorType::EmgSensor;
            else if (sensorTypeString == "Force3DSensor")
                return SensorType::Force3DSensor;
            else if (sensorTypeString == "ForceTorque6DSensor")
                return SensorType::ForceTorque6DSensor;
            else if (sensorTypeString == "FreeBodyAccelerationSensor")
                return SensorType::FreeBodyAccelerationSensor;
            else if (sensorTypeString == "Gyroscope")
                return SensorType::Gyroscope;
            else if (sensorTypeString == "Magnetometer")
                return SensorType::Magnetometer;
            else if (sensorTypeString == "OrientationSensor")
                return SensorType::OrientationSensor;
            else if (sensorTypeString == "PoseSensor")
                return SensorType::PoseSensor;
            else if (sensorTypeString == "PositionSensor")
                return SensorType::PositionSensor;
            else if (sensorTypeString == "SkinSensor")
                return SensorType::SkinSensor;
            else if (sensorTypeString == "TemperatureSensor")
                return SensorType::TemperatureSensor;
            else if (sensorTypeString == "Torque3DSensor")
                return SensorType::Torque3DSensor;
            else if (sensorTypeString == "VirtualLinkKinSensor")
                return SensorType::VirtualLinkKinSensor;
            else if (sensorTypeString == "VirtualJointKinSensor")
                return SensorType::VirtualJointKinSensor;
            else if (sensorTypeString == "VirtualSphericalJointKinSensor")
                return SensorType::VirtualSphericalJointKinSensor;
            else {
                return SensorType::Invalid;
            }
        }

        class ISensor;
    } // namespace sensor
} // namespace wearable

class wearable::sensor::ISensor : public wearable::IWearableDevice
{
protected:
    SensorName m_name;
    SensorType m_type;
    std::atomic<SensorStatus> m_status;

public:
    ISensor(SensorName aName = {}, SensorStatus aStatus = SensorStatus::Unknown)
        : m_name{aName}
        , m_status{aStatus}
    {
        m_wearable_element_type = ElementType::WearableSensor;
    }

    virtual ~ISensor() = default;

    inline ElementType getWearableElementType() const;

    // TODO: timestamp? sequence number?
    inline SensorName getSensorName() const;
    inline SensorStatus getSensorStatus() const;
    inline SensorType getSensorType() const;
};

inline wearable::ElementType wearable::sensor::ISensor::getWearableElementType() const
{
    return m_wearable_element_type;
}

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
