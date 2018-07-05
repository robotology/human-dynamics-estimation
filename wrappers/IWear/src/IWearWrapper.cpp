/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "IWearWrapper.h"
#include "Wearable/IWear/IWear.h"
#include "thrift/WearData.h"

#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/LogStream.h>

#include <mutex>

const std::string WrapperName = "IWearWrapper";
const std::string logPrefix = WrapperName + " : ";
constexpr double DefaultPeriod = 0.01;

using namespace wearable;
using namespace wearable::wrappers;

class IWearWrapper::impl
{
public:
    yarp::os::BufferedPort<msg::WearData> portWearData;

    wearable::IWear* iWear = nullptr;
    yarp::dev::IPreciselyTimed* iPreciselyTimed = nullptr;
};

IWearWrapper::IWearWrapper()
    : yarp::os::PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

IWearWrapper::~IWearWrapper()
{
    detachAll();
}

// =======
// Helpers
// =======

const std::map<sensor::SensorType, msg::SensorType> mapSensorTypes = {
    {sensor::SensorType::Accelerometer, msg::SensorType::ACCELEROMETER},
    {sensor::SensorType::EmgSensor, msg::SensorType::EMG_SENSOR},
    {sensor::SensorType::Force3DSensor, msg::SensorType::FORCE_3D_SENSOR},
    {sensor::SensorType::ForceTorque6DSensor, msg::SensorType::FORCE_TORQUE_6D_SENSOR},
    {sensor::SensorType::FreeBodyAccelerationSensor,
     msg::SensorType::FREE_BODY_ACCELERATION_SENSOR},
    {sensor::SensorType::Gyroscope, msg::SensorType::GYROSCOPE},
    {sensor::SensorType::Magnetometer, msg::SensorType::MAGNETOMETER},
    {sensor::SensorType::OrientationSensor, msg::SensorType::ORIENTATION_SENSOR},
    {sensor::SensorType::PoseSensor, msg::SensorType::POSE_SENSOR},
    {sensor::SensorType::PositionSensor, msg::SensorType::POSITION_SENSOR},
    {sensor::SensorType::SkinSensor, msg::SensorType::SKIN_SENSOR},
    {sensor::SensorType::TemperatureSensor, msg::SensorType::TEMPERATURE_SENSOR},
    {sensor::SensorType::Torque3DSensor, msg::SensorType::TORQUE_3D_SENSOR},
    {sensor::SensorType::VirtualLinkKinSensor, msg::SensorType::VIRTUAL_LINK_KIN_SENSOR},
    {sensor::SensorType::VirtualSphericalJointKinSensor,
     msg::SensorType::VIRTUAL_SPHERICAL_JOINT_KIN_SENSOR},
};

const std::map<sensor::SensorStatus, msg::SensorStatus> mapSensorStatus = {
    {sensor::SensorStatus::Ok, msg::SensorStatus::OK},
    {sensor::SensorStatus::Error, msg::SensorStatus::ERROR},
    {sensor::SensorStatus::Overflow, msg::SensorStatus::DATA_OVERFLOW},
    {sensor::SensorStatus::Calibrating, msg::SensorStatus::CALIBRATING},
    {sensor::SensorStatus::Timeout, msg::SensorStatus::TIMEOUT},
    {sensor::SensorStatus::WaitingForFirstRead, msg::SensorStatus::WAITING_FOR_FIRST_READ},
    {sensor::SensorStatus::Unknown, msg::SensorStatus::UNKNOWN},
};

msg::SensorInfo generateSensorStatus(const wearable::sensor::ISensor* sensor)
{
    return {sensor->getSensorName(),
            mapSensorTypes.at(sensor->getSensorType()),
            mapSensorStatus.at(sensor->getSensorStatus())};
}

msg::VectorXYZ vector3ToVectorXYZ(const wearable::Vector3& input)
{
    return {input[0], input[1], input[2]};
}

msg::VectorRPY vector3ToVectorRPY(const wearable::Vector3& input)
{
    return {input[0], input[1], input[2]};
}

msg::Quaternion convertQuaternion(const wearable::Quaternion& input)
{
    return {input[0], {input[1], input[2], input[3]}};
}

// ========================
// PeriodicThread interface
// ========================

bool IWearWrapper::threadInit()
{
    if (pImpl->iWear->getStatus() != WearStatus::Ok) {
        yError() << logPrefix << "Status of IWear object is not ::Ok";
        return false;
    }

    return true;
}

void IWearWrapper::run()
{
    if (!pImpl->iWear) {
        yError() << logPrefix << "The IWear pointer is null in the driver loop.";
        return;
    }

    if (!pImpl->iPreciselyTimed) {
        yError() << logPrefix << "The IPreciselyTimed pointer is null in the driver loop.";
        return;
    }

    msg::WearData& data = pImpl->portWearData.prepare();

    yarp::os::Stamp timestamp = pImpl->iPreciselyTimed->getLastInputStamp();
    pImpl->portWearData.setEnvelope(timestamp);

    {
        const auto vectorOfSensors = pImpl->iWear->getAccelerometers();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getLinearAcceleration(vector3)) {
                askToStop();
                return;
            }

            data.accelerometers.push_back(
                {generateSensorStatus(sensor.get()), vector3ToVectorXYZ(vector3)});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getEmgSensors();
        for (const auto& sensor : vectorOfSensors) {
            double value;
            // double normalizationValue;
            if (!sensor->getEmgSignal(value)
                // TODO
                // || !sensor->getEmgSignal(normalizationValue)
            ) {
                askToStop();
                return;
            }

            data.emgSensors.push_back({generateSensorStatus(sensor.get()), value});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getForce3DSensors();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getForce3D(vector3)) {
                askToStop();
                return;
            }

            data.force3DSensors.push_back(
                {generateSensorStatus(sensor.get()), vector3ToVectorXYZ(vector3)});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getForceTorque6DSensors();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector6 vector6;
            if (!sensor->getForceTorque6D(vector6)) {
                askToStop();
                return;
            }

            data.forceTorque6DSensors.push_back(
                {generateSensorStatus(sensor.get()),
                 {{vector6[0], vector6[1], vector6[2]}, {vector6[3], vector6[4], vector6[5]}}});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getFreeBodyAccelerationSensors();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getFreeBodyAcceleration(vector3)) {
                askToStop();
                return;
            }

            data.freeBodyAccelerationSensors.push_back(
                {generateSensorStatus(sensor.get()), vector3ToVectorXYZ(vector3)});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getGyroscopes();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getAngularRate(vector3)) {
                askToStop();
                return;
            }

            data.gyroscopes.push_back(
                {generateSensorStatus(sensor.get()), vector3ToVectorXYZ(vector3)});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getMagnetometers();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getMagneticField(vector3)) {
                askToStop();
                return;
            }

            data.magnetometers.push_back(
                {generateSensorStatus(sensor.get()), vector3ToVectorXYZ(vector3)});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getOrientationSensors();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Quaternion quaternion;
            if (!sensor->getOrientationAsQuaternion(quaternion)) {
                askToStop();
                return;
            }

            data.orientationSensors.push_back(
                {generateSensorStatus(sensor.get()), convertQuaternion(quaternion)});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getPoseSensors();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 vector3;
            wearable::Quaternion quaternion;
            if (!sensor->getPose(quaternion, vector3)) {
                askToStop();
                return;
            }

            data.poseSensors.push_back(
                {generateSensorStatus(sensor.get()),
                 {convertQuaternion(quaternion), vector3ToVectorXYZ(vector3)}});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getPositionSensors();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getPosition(vector3)) {
                askToStop();
                return;
            }

            data.positionSensors.push_back(
                {generateSensorStatus(sensor.get()), vector3ToVectorXYZ(vector3)});
        }
    }
    // TODO: skin sensor
    {
        const auto vectorOfSensors = pImpl->iWear->getTemperatureSensors();
        for (const auto& sensor : vectorOfSensors) {
            double value;
            if (!sensor->getTemperature(value)) {
                askToStop();
                return;
            }

            data.temperatureSensors.push_back({generateSensorStatus(sensor.get()), value});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->get3DTorqueSensors();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getTorque3D(vector3)) {
                askToStop();
                return;
            }

            data.torque3DSensors.push_back(
                {generateSensorStatus(sensor.get()), vector3ToVectorXYZ(vector3)});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getVirtualLinkKinSensors();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 linearAcc;
            wearable::Vector3 angularAcc;
            wearable::Vector3 linearVel;
            wearable::Vector3 angularVel;
            wearable::Vector3 position;
            wearable::Quaternion orientation;
            if (!sensor->getLinkAcceleration(linearAcc, angularAcc)
                || !sensor->getLinkPose(position, orientation)
                || !sensor->getLinkVelocity(linearVel, angularVel)) {
                askToStop();
                return;
            }

            data.virtualLinkKinSensors.push_back({generateSensorStatus(sensor.get()),
                                                  {convertQuaternion(orientation),
                                                   vector3ToVectorXYZ(position),
                                                   vector3ToVectorXYZ(linearVel),
                                                   vector3ToVectorXYZ(angularVel),
                                                   vector3ToVectorXYZ(linearAcc),
                                                   vector3ToVectorXYZ(angularAcc)}});
        }
    }
    {
        const auto vectorOfSensors = pImpl->iWear->getVirtualSphericalJointKinSensors();
        for (const auto& sensor : vectorOfSensors) {
            wearable::Vector3 jointAngles;
            wearable::Vector3 jointVel;
            wearable::Vector3 jointAcc;
            if (!sensor->getJointAnglesAsRPY(jointAngles) || !sensor->getJointVelocities(jointVel)
                || !sensor->getJointAccelerations(jointAcc)) {
                askToStop();
                return;
            }

            data.virtualSphericalJointKinSensors.push_back({generateSensorStatus(sensor.get()),
                                                            {vector3ToVectorRPY(jointAngles),
                                                             vector3ToVectorXYZ(jointVel),
                                                             vector3ToVectorXYZ(jointAcc)}});
        }
    }

    // Stream the data though the port
    pImpl->portWearData.write();
}

// ======================
// DeviceDriver interface
// ======================

bool IWearWrapper::open(yarp::os::Searchable& config)
{
    if (!config.check("outputPortName")) {
        yError() << logPrefix << "Output port name not specified";
        return false;
    }

    if (!config.check("period")) {
        yInfo() << logPrefix << "Using default period: " << DefaultPeriod << "s";
    }

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    setPeriod(period);

    return true;
}

bool IWearWrapper::close()
{
    return true;
}

// ==================
// IWrapper interface
// ==================

bool IWearWrapper::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << logPrefix << "Passed PolyDriver is nullptr.";
        return false;
    }

    if (pImpl->iWear || !poly->view(pImpl->iWear) || !pImpl->iWear) {
        yError() << logPrefix << "Failed to view the IWear interface from the PolyDriver.";
        return false;
    }

    if (pImpl->iPreciselyTimed || !poly->view(pImpl->iPreciselyTimed) || !pImpl->iPreciselyTimed) {
        yError() << logPrefix
                 << "Failed to view the IPreciselyTimed interface from the PolyDriver.";
        return false;
    }

    // Start the PeriodicThread loop
    if (!this->start()) {
        yError() << logPrefix << "Failed to start the loop.";
        return false;
    }

    return true;
}

bool IWearWrapper::detach()
{
    pImpl->iWear = nullptr;
    pImpl->iPreciselyTimed = nullptr;

    return true;
}

// ==========================
// IMultipleWrapper interface
// ==========================

bool IWearWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << logPrefix << "This wrapper accepts only one attached PolyDriver.";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << logPrefix << "Passed PolyDriverDescriptor is nullptr.";
        return false;
    }

    return attach(driver->poly);
}

bool IWearWrapper::detachAll()
{
    return detach();
}
