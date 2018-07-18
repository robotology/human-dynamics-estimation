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
#include "thrift/WearableMetadataService.h"

#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>

#include <mutex>

const std::string WrapperName = "IWearWrapper";
const std::string logPrefix = WrapperName + " : ";
constexpr double DefaultPeriod = 0.01;

using namespace wearable;
using namespace wearable::wrappers;

class IWearWrapper::impl : public wearable::msg::WearableMetadataService
{
public:
    yarp::os::Port rpcPort;
    yarp::os::BufferedPort<msg::WearableData> dataPort;

    std::map<msg::SensorType, std::vector<msg::WearableSensorMetadata>> wearableMetadata;

    std::string rpcPortName;
    std::string dataPortName;

    bool firstRun = true;

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
    wearable::VectorOfSensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
        virtualSphericalJointKinSensors;

    std::map<msg::SensorType, std::vector<msg::WearableSensorMetadata>> getMetadata() override
    {
        return wearableMetadata;
    }
    //    std::vector<std::string> help(const std::string& functionName = "--all") override; // TODO
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
        return;
    }

    if (!pImpl->iPreciselyTimed) {
        yError() << logPrefix << "The IPreciselyTimed pointer is null in the driver loop.";
        return;
    }

    if (pImpl->iWear->getStatus() != WearStatus::Ok) {
        return;
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
                yError() << logPrefix << "[Accelerometers] "
                         << "Failed to read data";
                askToStop();
                return;
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
                yError() << logPrefix << "[EmgSensors] "
                         << "Failed to read data";
                askToStop();
                return;
            }
            data.emgSensors[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                        {value, normalization}};
        }
    }
    {
        for (const auto& sensor : pImpl->force3DSensors) {
            yError() << logPrefix << "[Force3DSensors] "
                     << "Failed to read data";
            wearable::Vector3 vector3;
            if (!sensor->getForce3D(vector3)) {
                askToStop();
                return;
            }

            data.force3DSensors[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                            vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->forceTorque6DSensors) {
            wearable::Vector6 vector6;
            if (!sensor->getForceTorque6D(vector6)) {
                yError() << logPrefix << "[ForceTorque6DSensors] "
                         << "Failed to read data";
                askToStop();
                return;
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
                yError() << logPrefix << "[FreeBodyAccelerationSensors] "
                         << "Failed to read data";
                askToStop();
                return;
            }

            data.freeBodyAccelerationSensors[sensor->getSensorName()] = {
                generateSensorStatus(sensor.get()), vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->gyroscopes) {
            wearable::Vector3 vector3;
            if (!sensor->getAngularRate(vector3)) {
                yError() << logPrefix << "[Gyroscopes] "
                         << "Failed to read data";
                askToStop();
                return;
            }

            data.gyroscopes[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                        vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->magnetometers) {
            wearable::Vector3 vector3;
            if (!sensor->getMagneticField(vector3)) {
                yError() << logPrefix << "[Magnetometers] "
                         << "Failed to read data";
                askToStop();
                return;
            }

            data.magnetometers[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                           vector3ToVectorXYZ(vector3)};
        }
    }
    {
        for (const auto& sensor : pImpl->orientationSensors) {
            wearable::Quaternion quaternion;
            if (!sensor->getOrientationAsQuaternion(quaternion)) {
                yError() << logPrefix << "[OrientationSensors] "
                         << "Failed to read data";
                askToStop();
                return;
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
                yError() << logPrefix << "[PoseSensors] "
                         << "Failed to read data";
                askToStop();
                return;
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
                yError() << logPrefix << "[PositionSensors] "
                         << "Failed to read data";
                askToStop();
                return;
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
                yError() << logPrefix << "[TemperatureSensors] "
                         << "Failed to read data";
                askToStop();
                return;
            }

            data.temperatureSensors[sensor->getSensorName()] = {generateSensorStatus(sensor.get()),
                                                                value};
        }
    }
    {
        for (const auto& sensor : pImpl->torque3DSensors) {
            wearable::Vector3 vector3;
            if (!sensor->getTorque3D(vector3)) {
                yError() << logPrefix << "[Torque3DSensors] "
                         << "Failed to read data";
                askToStop();
                return;
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
                yError() << logPrefix << "[VirtualLinkKinSensors] "
                         << "Failed to read data";
                askToStop();
                return;
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
        for (const auto& sensor : pImpl->virtualSphericalJointKinSensors) {
            wearable::Vector3 jointAngles;
            wearable::Vector3 jointVel;
            wearable::Vector3 jointAcc;
            if (!sensor->getJointAnglesAsRPY(jointAngles) || !sensor->getJointVelocities(jointVel)
                || !sensor->getJointAccelerations(jointAcc)) {
                yError() << logPrefix << "[VirtualSphericalJointKinSensors] "
                         << "Failed to read data";
                askToStop();
                return;
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
    yInfo() << "Serialized";
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

    if (!config.check("rpcPortName") || !config.find("rpcPortName").isString()) {
        yError() << logPrefix << "rpcPortName parameter not found";
        return false;
    }

    if (!config.check("period")) {
        yInfo() << logPrefix << "Using default period: " << DefaultPeriod << "s";
    }

    pImpl->rpcPortName = config.find("rpcPortName").asString();
    pImpl->dataPortName = config.find("dataPortName").asString();

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    setPeriod(period);

    return true;
}

bool IWearWrapper::close()
{
    pImpl->rpcPort.close();
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

    // Start the PeriodicThread loop
    if (!this->start()) {
        yError() << logPrefix << "Failed to start the loop.";
        return false;
    }

    // Open the port for streaming data
    pImpl->dataPort.open(pImpl->dataPortName);

    // Prepare the metadata for the RPC port
    for (const auto& s : pImpl->iWear->getAccelerometers()) {
        pImpl->wearableMetadata[msg::SensorType::ACCELEROMETER].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getEmgSensors()) {
        pImpl->wearableMetadata[msg::SensorType::EMG_SENSOR].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getForce3DSensors()) {
        pImpl->wearableMetadata[msg::SensorType::FORCE_3D_SENSOR].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getForceTorque6DSensors()) {
        pImpl->wearableMetadata[msg::SensorType::FORCE_TORQUE_6D_SENSOR].push_back(
            s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getFreeBodyAccelerationSensors()) {
        pImpl->wearableMetadata[msg::SensorType::FREE_BODY_ACCELERATION_SENSOR].push_back(
            s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getGyroscopes()) {
        pImpl->wearableMetadata[msg::SensorType::GYROSCOPE].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getMagnetometers()) {
        pImpl->wearableMetadata[msg::SensorType::MAGNETOMETER].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getOrientationSensors()) {
        pImpl->wearableMetadata[msg::SensorType::ORIENTATION_SENSOR].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getPoseSensors()) {
        pImpl->wearableMetadata[msg::SensorType::POSE_SENSOR].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getPositionSensors()) {
        pImpl->wearableMetadata[msg::SensorType::POSITION_SENSOR].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getSkinSensors()) {
        pImpl->wearableMetadata[msg::SensorType::SKIN_SENSOR].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getTemperatureSensors()) {
        pImpl->wearableMetadata[msg::SensorType::TEMPERATURE_SENSOR].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getTorque3DSensors()) {
        pImpl->wearableMetadata[msg::SensorType::TORQUE_3D_SENSOR].push_back(s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getVirtualLinkKinSensors()) {
        pImpl->wearableMetadata[msg::SensorType::VIRTUAL_LINK_KIN_SENSOR].push_back(
            s->getSensorName());
    }
    for (const auto& s : pImpl->iWear->getVirtualSphericalJointKinSensors()) {
        pImpl->wearableMetadata[msg::SensorType::VIRTUAL_SPHERICAL_JOINT_KIN_SENSOR].push_back(
            s->getSensorName());
    }

    // Open the RPC port for providing metadata
    pImpl->yarp().attachAsServer(pImpl->rpcPort);
    if (!pImpl->rpcPort.open(pImpl->rpcPortName)) {
        yError() << logPrefix << "Failed to open " << pImpl->rpcPortName;
        return false;
    }

    return true;
}

bool IWearWrapper::detach()
{
    pImpl->iWear = nullptr;
    pImpl->iPreciselyTimed = nullptr;
    pImpl->wearableMetadata = {};

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
