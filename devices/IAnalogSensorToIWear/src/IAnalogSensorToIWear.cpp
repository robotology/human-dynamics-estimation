/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "IAnalogSensorToIWear.h"

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <mutex>
#include <string>
#include <vector>

const std::string DeviceName = "IAnalogSensorToIWear";
const std::string LogPrefix = DeviceName + " :";

using namespace wearable;
using namespace wearable::sensor;
using namespace wearable::devices;

// Class that stores the IAnalogSensor interface and provides utilites to
// map its data to containers compatible with IWear
static class IAnalogSensorHandler
{
public:
    yarp::sig::Vector buffer;
    yarp::dev::IAnalogSensor* interface = nullptr;

    bool readData()
    {
        if (!interface) {
            yError() << LogPrefix << "Failed to read data. Interface is nullptr.";
            return false;
        }

        if (interface->read(buffer) != yarp::dev::IAnalogSensor::AS_OK) {
            yError() << LogPrefix << "Failed to read data from the IAnalogSensor interface."
                     << "Sensor read status is" << interface->read(buffer);
            return false;
        }

        return true;
    }

    wearable::sensor::SensorStatus getStatus()
    {
        if (!interface) {
            yError() << LogPrefix << "Failed to get status of IAnalogSensor";
            return SensorStatus::Unknown;
        }

        SensorStatus status = SensorStatus::Ok;

        // Logic for combining the status of all channels.
        // The tricky part is deciding how to handle the mixed case of timeout and overflow.
        // TODO: For now, overflow is stronger.
        for (int i = 0; i < interface->getChannels(); ++i) {
            switch (interface->getState(i)) {
                case yarp::dev::IAnalogSensor::AS_ERROR:
                    // If even just one sensor is AS_ERROR, return Error
                    return SensorStatus::Error;
                case yarp::dev::IAnalogSensor::AS_OVF:
                    status = SensorStatus::Overflow;
                    break;
                case yarp::dev::IAnalogSensor::AS_TIMEOUT:
                    if (status == SensorStatus::Overflow) {
                        break;
                    }
                    status = SensorStatus::Timeout;
                    break;
                case yarp::dev::IAnalogSensor::AS_OK:
                    // Keep checking other channels
                    break;
            }
        }

        return status;
    }

    bool getData(double& value, const size_t offset = 0)
    {
        if (buffer.size() != 1) {
            yError() << LogPrefix << "Size mismatch of the data read from IAnalogSensor interface."
                     << "The buffer was initialized with a size of" << buffer.size()
                     << "but the wearable sensor exposes" << 1 << "value";
            return false;
        }

        value = buffer[offset];
        return true;
    }

    bool getData(std::array<double, 3>& array, const size_t offset = 0)
    {
        if (buffer.size() < 3 + offset) {
            yError() << LogPrefix << "Size mismatch of the data read from IAnalogSensor interface."
                     << "The buffer was initialized with a size of" << buffer.size()
                     << "but the wearable sensor exposes" << 3 << "values";
            return false;
        }

        std::copy(buffer.data() + offset, buffer.data() + offset + 3, array.data());
        return true;
    }

    bool getData(std::array<double, 4>& array, const size_t offset = 0)
    {
        if (buffer.size() < 4 + offset) {
            yError() << LogPrefix << "Size mismatch of the data read from IAnalogSensor interface."
                     << "The buffer was initialized with a size of" << buffer.size()
                     << "but the wearable sensor exposes" << 4 << "values";
            return false;
        }

        std::copy(buffer.data() + offset, buffer.data() + offset + 4, array.data());
        return true;
    }
} handler;

// ================================
// WEARABLE SENSORS IMPLEMENTATIONS
// ================================

class ForceTorque6DSensor : public IForceTorque6DSensor
{
public:
    unsigned offset = 0;

    ForceTorque6DSensor(SensorName name, SensorStatus status = SensorStatus::Unknown)
        : IForceTorque6DSensor(name, status)
    {}

    void setStatus(const SensorStatus status) { m_status = status; }

    bool getForceTorque6D(Vector3& force3D, Vector3& torque3D) const override
    {
        if (!handler.readData()) {
            return false;
        }

        // Dirty workaround to set the status from a const method
        auto nonConstThis = const_cast<ForceTorque6DSensor*>(this);
        nonConstThis->setStatus(handler.getStatus());

        // TODO: The positions of force and torques are hardcoded. Forces should be the first
        //       triplet of elements of the read vector and torques the second one.
        return handler.getData(force3D, offset) && handler.getData(torque3D, offset + 3);
    }
};

class Force3DSensor : public IForce3DSensor
{
public:
    unsigned offset = 0;

    Force3DSensor(SensorName name, SensorStatus status = SensorStatus::Unknown)
        : IForce3DSensor(name, status)
    {}

    void setStatus(const SensorStatus status) { m_status = status; }

    bool getForce3D(Vector3& force3D) const override
    {
        if (!handler.readData()) {
            return false;
        }

        // Dirty workaround to set the status from a const method
        auto nonConstThis = const_cast<Force3DSensor*>(this);
        nonConstThis->setStatus(handler.getStatus());

        return handler.getData(force3D, offset);
    }
};

class Torque3DSensor : public ITorque3DSensor
{
public:
    unsigned offset = 0;

    Torque3DSensor(SensorName name, SensorStatus status = SensorStatus::Unknown)
        : ITorque3DSensor(name, status)
    {}

    void setStatus(const SensorStatus status) { m_status = status; }

    bool getTorque3D(Vector3& torque3D) const override
    {
        if (!handler.readData()) {
            return false;
        }

        // Dirty workaround to set the status from a const method
        auto nonConstThis = const_cast<Torque3DSensor*>(this);
        nonConstThis->setStatus(handler.getStatus());

        return handler.getData(torque3D, offset);
    }
};

class TemperatureSensor : public ITemperatureSensor
{
public:
    unsigned offset = 0;

    TemperatureSensor(SensorName name, SensorStatus status = SensorStatus::Unknown)
        : ITemperatureSensor(name, status)
    {}

    void setStatus(const SensorStatus status) { m_status = status; }

    bool getTemperature(double& temperature) const override
    {
        if (!handler.readData()) {
            return false;
        }

        // Dirty workaround to set the status from a const method
        auto nonConstThis = const_cast<TemperatureSensor*>(this);
        nonConstThis->setStatus(handler.getStatus());

        return handler.getData(temperature, offset);
    }
};

// TODO: implement all the other sensors

// ====================
// IANALOGSENSORTOIWEAR
// ====================

wearable::sensor::SensorType sensorTypeFromString(std::string sensorTypeString)
{
    if (sensorTypeString == "Accelerometer")
        return wearable::sensor::SensorType::Accelerometer;
    else if (sensorTypeString == "EmgSensor")
        return wearable::sensor::SensorType::EmgSensor;
    else if (sensorTypeString == "Force3DSensor")
        return wearable::sensor::SensorType::Force3DSensor;
    else if (sensorTypeString == "ForceTorque6DSensor")
        return wearable::sensor::SensorType::ForceTorque6DSensor;
    else if (sensorTypeString == "FreeBodyAccelerationSensor")
        return wearable::sensor::SensorType::FreeBodyAccelerationSensor;
    else if (sensorTypeString == "Gyroscope")
        return wearable::sensor::SensorType::Gyroscope;
    else if (sensorTypeString == "Magnetometer")
        return wearable::sensor::SensorType::Magnetometer;
    else if (sensorTypeString == "OrientationSensor")
        return wearable::sensor::SensorType::OrientationSensor;
    else if (sensorTypeString == "PoseSensor")
        return wearable::sensor::SensorType::PoseSensor;
    else if (sensorTypeString == "PositionSensor")
        return wearable::sensor::SensorType::PositionSensor;
    else if (sensorTypeString == "SkinSensor")
        return wearable::sensor::SensorType::SkinSensor;
    else if (sensorTypeString == "TemperatureSensor")
        return wearable::sensor::SensorType::TemperatureSensor;
    else if (sensorTypeString == "Torque3DSensor")
        return wearable::sensor::SensorType::Torque3DSensor;
    else if (sensorTypeString == "VirtualLinkKinSensor")
        return wearable::sensor::SensorType::VirtualLinkKinSensor;
    else if (sensorTypeString == "VirtualSphericalJointKinSensor")
        return wearable::sensor::SensorType::VirtualSphericalJointKinSensor;
    else {
        yError() << LogPrefix << "Sensor type" << sensorTypeString << "not supported";
        return {};
    }
}

struct ParsedOptions
{
    wearable::WearableName wearableName;

    wearable::sensor::SensorName sensorName;
    wearable::sensor::SensorType wearableSensorType;

    size_t numberOfChannels;
    size_t channelOffset;
};

class IAnalogSensorToIWear::Impl
{
public:
    bool firstRun = true;
    mutable std::recursive_mutex mutex;

    TimeStamp timestamp;
    ParsedOptions options;

    wearable::SensorPtr<wearable::sensor::ISensor> iSensor;

    bool allocateSensor(const wearable::sensor::SensorType type,
                        const wearable::sensor::SensorName name);
};

IAnalogSensorToIWear::IAnalogSensorToIWear()
    : pImpl{new Impl()}
{}

// Without this destructor here, the linker complains for
// undefined reference to vtable
IAnalogSensorToIWear::~IAnalogSensorToIWear() = default;

bool IAnalogSensorToIWear::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("sensorName") && config.find("sensorName").isString())) {
        yError() << LogPrefix << "Parameter 'sensorName' missing or invalid";
        return false;
    }

    if (!(config.check("wearableName") && config.find("wearableName").isString())) {
        yError() << LogPrefix << "Parameter 'wearableName' missing or invalid";
        return false;
    }

    if (!(config.check("numberOfChannels") && config.find("numberOfChannels").isInt())) {
        yError() << LogPrefix << "Parameter 'numberOfChannels' missing or invalid";
        return false;
    }

    if (!(config.check("channelOffset") && config.find("channelOffset").isInt())) {
        yError() << LogPrefix << "Parameter 'channelOffset' missing or invalid";
        return false;
    }

    if (!(config.check("wearableSensorType") && config.find("wearableSensorType").isString())) {
        yError() << LogPrefix << "Parameter 'wearableSensorType' missing or invalid";
        return false;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    pImpl->options.sensorName = config.find("sensorName").asString();
    pImpl->options.wearableName = config.find("wearableName").asString();
    pImpl->options.numberOfChannels = config.find("numberOfChannels").asInt();
    pImpl->options.channelOffset = config.find("channelOffset").asInt();
    std::string sensorType = config.find("wearableSensorType").asString();
    pImpl->options.wearableSensorType = sensorTypeFromString(sensorType);

    yInfo() << LogPrefix << "*** ====================";
    yInfo() << LogPrefix << "*** Sensor name        :" << pImpl->options.sensorName;
    yInfo() << LogPrefix << "*** Sensor Type        :" << sensorType;
    yInfo() << LogPrefix << "*** Wearable name      :" << pImpl->options.wearableName;
    yInfo() << LogPrefix << "*** Number of channels :" << pImpl->options.numberOfChannels;
    yInfo() << LogPrefix << "*** Channel offset     :" << pImpl->options.channelOffset;
    yInfo() << LogPrefix << "*** ====================";

    // =====================
    // INITIALIZE THE DEVICE
    // =====================

    handler.buffer.resize(pImpl->options.numberOfChannels);

    return true;
}

bool IAnalogSensorToIWear::close()
{
    detach();
    return true;
}

yarp::os::Stamp IAnalogSensorToIWear::getLastInputStamp()
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return yarp::os::Stamp(pImpl->timestamp.sequenceNumber, yarp::os::Time::now());
}

bool IAnalogSensorToIWear::Impl::allocateSensor(const wearable::sensor::SensorType type,
                                                const wearable::sensor::SensorName name)
{
    // The sensors are initialized as Ok in order to trigger the first data read.
    // If there is any error during the first read, the sensor updates its own status
    // that is then propagated to the global IWear status.
    switch (type) {
        case wearable::sensor::SensorType::Force3DSensor: {
            auto sensor = std::make_shared<Force3DSensor>(name, SensorStatus::Ok);
            sensor->offset = options.channelOffset;
            iSensor = std::dynamic_pointer_cast<ISensor>(sensor);
            break;
        }
        case wearable::sensor::SensorType::ForceTorque6DSensor: {
            auto sensor = std::make_shared<ForceTorque6DSensor>(name, SensorStatus::Ok);
            sensor->offset = options.channelOffset;
            iSensor = std::dynamic_pointer_cast<ISensor>(sensor);
            break;
        }
        case wearable::sensor::SensorType::TemperatureSensor: {
            auto sensor = std::make_shared<TemperatureSensor>(name, SensorStatus::Ok);
            sensor->offset = options.channelOffset;
            iSensor = std::dynamic_pointer_cast<ISensor>(sensor);
            break;
        }
        case wearable::sensor::SensorType::Torque3DSensor: {
            auto sensor = std::make_shared<Torque3DSensor>(name, SensorStatus::Ok);
            sensor->offset = options.channelOffset;
            iSensor = std::dynamic_pointer_cast<ISensor>(sensor);
            break;
        }
        default:
            // TODO: implement the remaining sensors
            return false;
    }

    return true;
}

bool IAnalogSensorToIWear::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (handler.interface || !poly->view(handler.interface) || !handler.interface) {
        yError() << LogPrefix << "Failed to view the IAnalogSensor interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    if (handler.interface->getChannels() == 0) {
        yError() << LogPrefix << "The number of channels is 0";
        return false;
    }

    if (handler.interface->getChannels()
        != pImpl->options.numberOfChannels + pImpl->options.channelOffset) {
        yError() << LogPrefix << "The number of sensor channels ("
                 << handler.interface->getChannels()
                 << ") is different than the number specified in the options plus the offset ("
                 << pImpl->options.numberOfChannels + pImpl->options.channelOffset << ")";
        return false;
    }

    for (unsigned i = 0; i < handler.interface->getChannels(); ++i) {
        if (handler.interface->getState(i) != yarp::dev::IAnalogSensor::AS_OK) {
            yError() << LogPrefix << "The status of IAnalogSensor interface for channel" << i
                     << "is not AS_OK (" << handler.interface->getState(i) << ")";
            return false;
        }
    }

    if (!pImpl->allocateSensor(pImpl->options.wearableSensorType, pImpl->options.sensorName)) {
        yError() << LogPrefix << "Failed to allocate a new sensor of the specified type";
        return false;
    }

    // Notify that the sensor is ready to be used
    pImpl->firstRun = false;

    return true;
}

bool IAnalogSensorToIWear::detach()
{
    handler.interface = nullptr;
    return true;
}

bool IAnalogSensorToIWear::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool IAnalogSensorToIWear::detachAll()
{
    return detach();
}

wearable::SensorPtr<const ISensor> IAnalogSensorToIWear::getSensor(const SensorName name) const
{
    // This device can provide only one sensor. Check if the name matches.
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return pImpl->iSensor;
}

wearable::VectorOfSensorPtr<const ISensor>
IAnalogSensorToIWear::getSensors(const SensorType type) const
{
    wearable::VectorOfSensorPtr<const ISensor> vector;

    if (pImpl->options.wearableSensorType == type) {
        vector.push_back(getSensor(pImpl->options.sensorName));
    }

    return vector;
}

wearable::WearableName IAnalogSensorToIWear::getWearableName() const
{
    return pImpl->options.wearableName + "::";
}

wearable::WearStatus IAnalogSensorToIWear::getStatus() const
{
    // This is necessary if something that uses the exposed IWear interface asks the status
    // before IAnalogSensor is attached
    if (pImpl->firstRun) {
        return WearStatus::WaitingForFirstRead;
    }

    if (!pImpl->iSensor) {
        yError() << LogPrefix << "The stored ISensor has not been yet allocated";
        return WearStatus::Error;
    }

    return pImpl->iSensor->getSensorStatus();
}

wearable::TimeStamp IAnalogSensorToIWear::getTimeStamp() const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);

    pImpl->timestamp.sequenceNumber = 0; // Always zero
    pImpl->timestamp.time = yarp::os::Time::now();

    return pImpl->timestamp;
}

wearable::SensorPtr<const wearable::sensor::IAccelerometer>
IAnalogSensorToIWear::getAccelerometer(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
IAnalogSensorToIWear::getForce3DSensor(const wearable::sensor::SensorName name) const
{
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return std::dynamic_pointer_cast<const wearable::sensor::IForce3DSensor>(pImpl->iSensor);
}

wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
IAnalogSensorToIWear::getForceTorque6DSensor(const wearable::sensor::SensorName name) const
{
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return std::dynamic_pointer_cast<const wearable::sensor::IForceTorque6DSensor>(pImpl->iSensor);
}

wearable::SensorPtr<const wearable::sensor::IGyroscope>
IAnalogSensorToIWear::getGyroscope(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IMagnetometer>
IAnalogSensorToIWear::getMagnetometer(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
IAnalogSensorToIWear::getOrientationSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
IAnalogSensorToIWear::getTemperatureSensor(const wearable::sensor::SensorName name) const
{
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return std::dynamic_pointer_cast<const wearable::sensor::ITemperatureSensor>(pImpl->iSensor);
}

wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
IAnalogSensorToIWear::getTorque3DSensor(const wearable::sensor::SensorName name) const
{
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return std::dynamic_pointer_cast<const wearable::sensor::ITorque3DSensor>(pImpl->iSensor);
}

wearable::SensorPtr<const wearable::sensor::IEmgSensor>
IAnalogSensorToIWear::getEmgSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
IAnalogSensorToIWear::getFreeBodyAccelerationSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IPoseSensor>
IAnalogSensorToIWear::getPoseSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IPositionSensor>
IAnalogSensorToIWear::getPositionSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ISkinSensor>
IAnalogSensorToIWear::getSkinSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
IAnalogSensorToIWear::getVirtualLinkKinSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
IAnalogSensorToIWear::getVirtualSphericalJointKinSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}
