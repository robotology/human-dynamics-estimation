/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "IWearWrapper.h"
#include "Wearable/IWear/IWear.h"
#include "thrift/WearableData.h"

#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>

const std::string WrapperName = "IWearWrapper";
const std::string logPrefix = WrapperName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace wearable;
using namespace wearable::wrappers;

class IWearWrapper::impl
{
public:
    yarp::os::BufferedPort<msg::WearableData> dataPort;

    std::string dataPortName;

    bool firstRun = true;
    size_t waitingFirstReadCounter = 1;

    wearable::IWear* iWear = nullptr;
    yarp::dev::IPreciselyTimed* iPreciselyTimed = nullptr;

    wearable::VectorOfSensorPtr<const wearable::sensor::IAccelerometer> accelerometers;
    wearable::VectorOfSensorPtr<const wearable::sensor::IEmgSensor> emgSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::IForce3DSensor> force3DSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::IForceTorque6DSensor> forceTorque6DSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
        freeBodyAccelerationSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::IGyroscope> gyroscopes;
    wearable::VectorOfSensorPtr<const wearable::sensor::IMagnetometer> magnetometers;
    wearable::VectorOfSensorPtr<const wearable::sensor::IOrientationSensor> orientationSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::IPoseSensor> poseSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::IPositionSensor> positionSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::ISkinSensor> skinSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::ITemperatureSensor> temperatureSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::ITorque3DSensor> torque3DSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
        virtualLinkKinSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualJointKinSensor>
        virtualJointKinSensors;
    wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
        virtualSphericalJointKinSensors;

    //    std::vector<std::string> help(const std::string& functionName = "--all") override; // TODO
};

IWearWrapper::IWearWrapper()
    : yarp::os::PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

IWearWrapper::~IWearWrapper()
{
    detachAll();
    close();
}

// =======
// Helpers
// =======

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
    return {sensor->getSensorName(), mapSensorStatus.at(sensor->getSensorStatus())};
}

msg::VectorXYZ vector3ToVectorXYZ(const wearable::Vector3& input)
{
    return {input[0], input[1], input[2]};
}

msg::VectorRPY vector3ToVectorRPY(const wearable::Vector3& input)
{
    return {input[0], input[1], input[2]};
}

msg::QuaternionWXYZ convertQuaternion(const wearable::Quaternion& input)
{
    return {input[0], input[1], input[2], input[3]};
}

// ========================
// PeriodicThread interface
// ========================

void IWearWrapper::run()
{
    if (!pImpl->iWear) {
        yError() << logPrefix << "The IWear pointer is null in the driver loop.";
        askToStop();
        return;
    }

    if (!pImpl->iPreciselyTimed) {
        yError() << logPrefix << "The IPreciselyTimed pointer is null in the driver loop.";
        askToStop();
        return;
    }

    while (pImpl->iWear->getStatus() == WearStatus::Calibrating
           || pImpl->iWear->getStatus() == WearStatus::WaitingForFirstRead) {
        if (pImpl->waitingFirstReadCounter++ % 1000 == 0) {
            pImpl->waitingFirstReadCounter = 1;
            yInfo() << logPrefix << "IWear interface waiting for first data. Waiting...";
        }
        return;
    }

    if (pImpl->iWear->getStatus() == WearStatus::Error
            || pImpl->iWear->getStatus() == WearStatus::Unknown) {
        yError() << logPrefix << "The status of the IWear interface is not Ok ("
                 << static_cast<int>(pImpl->iWear->getStatus()) << ")";
        askToStop();
        return;
    }

    // case status is TIMEOUT or DATA_OVERFLOW
    if (pImpl->iWear->getStatus() != WearStatus::Ok) {
        yWarning() << logPrefix << "The status of the IWear interface is not Ok ("
                 << static_cast<int>(pImpl->iWear->getStatus()) << ")";
    }

    if (pImpl->firstRun) {
        pImpl->firstRun = false;

        pImpl->accelerometers = pImpl->iWear->getAccelerometers();
        pImpl->emgSensors = pImpl->iWear->getEmgSensors();
        pImpl->force3DSensors = pImpl->iWear->getForce3DSensors();
        pImpl->forceTorque6DSensors = pImpl->iWear->getForceTorque6DSensors();
        pImpl->freeBodyAccelerationSensors = pImpl->iWear->getFreeBodyAccelerationSensors();
        pImpl->gyroscopes = pImpl->iWear->getGyroscopes();
        pImpl->magnetometers = pImpl->iWear->getMagnetometers();
        pImpl->orientationSensors = pImpl->iWear->getOrientationSensors();
        pImpl->poseSensors = pImpl->iWear->getPoseSensors();
        pImpl->positionSensors = pImpl->iWear->getPositionSensors();
        pImpl->skinSensors = pImpl->iWear->getSkinSensors();
        pImpl->temperatureSensors = pImpl->iWear->getTemperatureSensors();
        pImpl->torque3DSensors = pImpl->iWear->getTorque3DSensors();
        pImpl->virtualLinkKinSensors = pImpl->iWear->getVirtualLinkKinSensors();
        pImpl->virtualJointKinSensors = pImpl->iWear->getVirtualJointKinSensors();
        pImpl->virtualSphericalJointKinSensors = pImpl->iWear->getVirtualSphericalJointKinSensors();
    }

    msg::WearableData& data = pImpl->dataPort.prepare();
    data.producerName = pImpl->iWear->getWearableName();

    yarp::os::Stamp timestamp = pImpl->iPreciselyTimed->getLastInputStamp();
    pImpl->dataPort.setEnvelope(timestamp);

    {
        for (const auto& sensor : pImpl->accelerometers) {
            wearable::Vector3 vector3;
            if (!sensor->getLinearAcceleration(vector3)) {
                yWarning() << logPrefix << "[Accelerometers] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.accelerometers[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                            vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->emgSensors) {
            double value, normalization;
            // double normalizationValue;
            if (!sensor->getEmgSignal(value) || !sensor->getEmgSignal(normalization)) {
                yWarning() << logPrefix << "[EmgSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }
            data.emgSensors[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                        {value, normalization}};
        }
    }
    {
        for (const auto& sensor : pImpl->force3DSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getForce3D(vector3)) {
                yWarning() << logPrefix << "[Force3DSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.force3DSensors[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                            vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->forceTorque6DSensors) {
            wearable::Vector6 vector6;
            if (!sensor->getForceTorque6D(vector6)) {
                yWarning() << logPrefix << "[ForceTorque6DSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.forceTorque6DSensors[sensor->getSensorName()] = {
                generateSensorStatus(sensor.get()),
                {{vector6[0], vector6[1], vector6[2]}, {vector6[3], vector6[4], vector6[5]}}};
        }
    }
    {
        for (const auto& sensor : pImpl->freeBodyAccelerationSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getFreeBodyAcceleration(vector3)) {
                yWarning() << logPrefix << "[FreeBodyAccelerationSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.freeBodyAccelerationSensors[sensor->getSensorName()] = {
                generateSensorStatus(sensor.get()), vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->gyroscopes) {
            wearable::Vector3 vector3;
            if (!sensor->getAngularRate(vector3)) {
                yWarning() << logPrefix << "[Gyroscopes] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.gyroscopes[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                        vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->magnetometers) {
            wearable::Vector3 vector3;
            if (!sensor->getMagneticField(vector3)) {
                yWarning() << logPrefix << "[Magnetometers] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.magnetometers[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                           vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->orientationSensors) {
            wearable::Quaternion quaternion;
            if (!sensor->getOrientationAsQuaternion(quaternion)) {
                yWarning() << logPrefix << "[OrientationSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.orientationSensors[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                                convertQuaternion(quaternion)};
        }
    }
    {
        for (const auto& sensor : pImpl->poseSensors) {
            wearable::Vector3 vector3;
            wearable::Quaternion quaternion;
            if (!sensor->getPose(quaternion, vector3)) {
                yWarning() << logPrefix << "[PoseSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.poseSensors[sensor->getSensorName()] = {
                generateSensorStatus(sensor.get()),
                {convertQuaternion(quaternion), vector3ToVectorXYZ(vector3)}};
        }
    }
    {
        for (const auto& sensor : pImpl->positionSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getPosition(vector3)) {
                yWarning() << logPrefix << "[PositionSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.positionSensors[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                             vector3ToVectorXYZ(vector3)};
        }
    }
    {
        if (pImpl->skinSensors.size() > 0) {
            yWarning() << logPrefix << "SkinSensor not yet implemented.";
        }
    }
    {
        for (const auto& sensor : pImpl->temperatureSensors) {
            double value;
            if (!sensor->getTemperature(value)) {
                yWarning() << logPrefix << "[TemperatureSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.temperatureSensors[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                                value};
        }
    }
    {
        for (const auto& sensor : pImpl->torque3DSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getTorque3D(vector3)) {
                yWarning() << logPrefix << "[Torque3DSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.torque3DSensors[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                             vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->virtualLinkKinSensors) {
            wearable::Vector3 linearAcc;
            wearable::Vector3 angularAcc;
            wearable::Vector3 linearVel;
            wearable::Vector3 angularVel;
            wearable::Vector3 position;
            wearable::Quaternion orientation;
            if (!sensor->getLinkAcceleration(linearAcc, angularAcc)
                || !sensor->getLinkPose(position, orientation)
                || !sensor->getLinkVelocity(linearVel, angularVel)) {
                yWarning() << logPrefix << "[VirtualLinkKinSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }

            data.virtualLinkKinSensors[sensor->getSensorName()] = {
                generateSensorStatus(sensor.get()),
                {convertQuaternion(orientation),
                 vector3ToVectorXYZ(position),
                 vector3ToVectorXYZ(linearVel),
                 vector3ToVectorXYZ(angularVel),
                 vector3ToVectorXYZ(linearAcc),
                 vector3ToVectorXYZ(angularAcc)}};
        }
    }
    {
        for (const auto& sensor : pImpl->virtualJointKinSensors) {
            double jointPos;
            double jointVel;
            double jointAcc;
            if (!sensor->getJointPosition(jointPos) || !sensor->getJointVelocity(jointVel)
                || !sensor->getJointAcceleration(jointAcc)) {
                yError() << logPrefix << "[VirtualJointKinSensors] "
                         << "Failed to read data";
                askToStop();
                return;
            }
            data.virtualJointKinSensors[sensor->getSensorName()] = {
                generateSensorStatus(sensor.get()),
                {jointPos,
                jointVel,
                jointAcc}};
        }
    }
    {
        for (const auto& sensor : pImpl->virtualSphericalJointKinSensors) {
            wearable::Vector3 jointAngles;
            wearable::Vector3 jointVel;
            wearable::Vector3 jointAcc;
            if (!sensor->getJointAnglesAsRPY(jointAngles) || !sensor->getJointVelocities(jointVel)
                || !sensor->getJointAccelerations(jointAcc)) {
                yWarning() << logPrefix << "[VirtualSphericalJointKinSensors] "
                         << "Failed to read data, "
                         << "sensor status is "
                         << static_cast<int>(sensor->getSensorStatus());
            }
            data.virtualSphericalJointKinSensors[sensor->getSensorName()] = {
                generateSensorStatus(sensor.get()),
                {vector3ToVectorRPY(jointAngles),
                 vector3ToVectorXYZ(jointVel),
                 vector3ToVectorXYZ(jointAcc)}};
        }
    }

    // Stream the data though the port
    pImpl->dataPort.write(true);
}

// ======================
// DeviceDriver interface
// ======================

bool IWearWrapper::open(yarp::os::Searchable& config)
{
    if (!config.check("dataPortName") || !config.find("dataPortName").isString()) {
        yError() << logPrefix << "dataPortName parameter not found";
        return false;
    }

    if (!config.check("period")) {
        yInfo() << logPrefix << "Using default period: " << DefaultPeriod << "s";
    }

    pImpl->dataPortName = config.find("dataPortName").asString();

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    setPeriod(period);

    return true;
}

bool IWearWrapper::close()
{
    pImpl->dataPort.close();
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

    // Open the port for streaming data
    pImpl->dataPort.open(pImpl->dataPortName);

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << logPrefix << "Failed to start the loop.";
        return false;
    }

    yDebug() << logPrefix << "attach() successful";
    return true;
}

void IWearWrapper::threadRelease()
{

}

bool IWearWrapper::detach()
{
    while (isRunning()) {
        stop();
    }

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
