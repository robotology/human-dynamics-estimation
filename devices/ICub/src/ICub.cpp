/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "ICub.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

#include <assert.h>
#include <map>
#include <mutex>
#include <string>
#include <vector>

using namespace wearable::devices;

const std::string LogPrefix = "ICub :";

class WrenchPort : public yarp::os::BufferedPort<yarp::os::Bottle>
{
public:
    std::vector<double> wrenchValues;
    bool firstRun = true;
    mutable std::mutex mtx;

    using yarp::os::BufferedPort<yarp::os::Bottle>::onRead;
    virtual void onRead(yarp::os::Bottle& wrench)
    {
        if (!wrench.isNull()) {
            std::lock_guard<std::mutex> lock(mtx);

            this->wrenchValues.resize(wrench.size());

            for (size_t i = 0; i < wrench.size(); ++i) {
                this->wrenchValues.at(i) = wrench.get(i).asDouble();
            }

            if (this->firstRun) {
                this->firstRun = false;
            }
        }
        else {
            yWarning() << LogPrefix << "[WrenchPort] read an invalid wrench bottle";
        }
    }
};

class ICub::ICubImpl
{
public:
    class ICubForceTorque6DSensor;

    wearable::TimeStamp timeStamp;

    size_t nSensors;
    std::vector<std::string> sensorNames;

    bool leftHandFTDataReceived = false;
    bool rightHandFTDataReceived = false;

    std::map<std::string, std::shared_ptr<ICubForceTorque6DSensor>> ftSensorsMap;

    const WearableName wearableName = "ICub";

    WrenchPort leftHandFTPort;
    WrenchPort rightHandFTPort;
};

// ==========================================
// ICub implementation of ForceTorque6DSensor
// ==========================================
class ICub::ICubImpl::ICubForceTorque6DSensor : public wearable::sensor::IForceTorque6DSensor
{
public:
    ICub::ICubImpl* icubImpl = nullptr;

    // ------------------------
    // Constructor / Destructor
    // ------------------------
    ICubForceTorque6DSensor(
        ICub::ICubImpl* impl,
        const wearable::sensor::SensorName name = {},
        const wearable::sensor::SensorStatus status =
            wearable::sensor::SensorStatus::WaitingForFirstRead) // Default sensor status
        : IForceTorque6DSensor(name, status)
        , icubImpl(impl)
    {
        // Set the sensor status Ok after receiving the first data on the wrench ports
        if (!icubImpl->leftHandFTPort.firstRun || !icubImpl->rightHandFTPort.firstRun) {
            auto nonConstThis = const_cast<ICubForceTorque6DSensor*>(this);
            nonConstThis->setStatus(wearable::sensor::SensorStatus::Ok);
        }
    }

    ~ICubForceTorque6DSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    // ==============================
    // IForceTorque6DSensor interface
    // ==============================
    bool getForceTorque6D(Vector3& force3D, Vector3& torque3D) const override
    {
        assert(icubImpl != nullptr);

        std::vector<double> wrench;

        // Reading wrench from WBD ports
        if (this->m_name
            == icubImpl->wearableName + wearable::Separator
                   + sensor::IForceTorque6DSensor::getPrefix() + "leftWBDFTSensor") {

            std::lock_guard<std::mutex> lck(icubImpl->leftHandFTPort.mtx);
            wrench = icubImpl->leftHandFTPort.wrenchValues;

            icubImpl->timeStamp.time = yarp::os::Time::now();
        }
        else if (this->m_name
                 == icubImpl->wearableName + wearable::Separator
                        + sensor::IForceTorque6DSensor::getPrefix() + "rightWBDFTSensor") {

            std::lock_guard<std::mutex> lck(icubImpl->rightHandFTPort.mtx);
            wrench = icubImpl->rightHandFTPort.wrenchValues;

            icubImpl->timeStamp.time = yarp::os::Time::now();
        }

        // Check the size of wrench values
        if (wrench.size() == 6) {
            force3D[0] = wrench.at(0);
            force3D[1] = wrench.at(1);
            force3D[2] = wrench.at(2);

            torque3D[0] = wrench.at(3);
            torque3D[1] = wrench.at(4);
            torque3D[2] = wrench.at(5);
        }
        else {
            yWarning() << LogPrefix << "Size of wrench read is wrong. Setting zero values instead.";
            force3D.fill(0.0);
            torque3D.fill(0.0);

            // Set sensor status to error if wrong data is read
            // auto nonConstThis = const_cast<ICubForceTorque6DSensor*>(this);
            // nonConstThis->setStatus(wearable::sensor::SensorStatus::Error);
        }

        return true;
    }
};

// ==========================================
// ICub constructor/destructor implementation
// ==========================================
ICub::ICub()
    : pImpl{new ICubImpl()}
{}

ICub::~ICub() = default;

// ======================
// DeviceDriver interface
// ======================
bool ICub::open(yarp::os::Searchable& config)
{
    yInfo() << LogPrefix << "Starting to configure";

    // Configure clock
    if (!yarp::os::Time::isSystemClock()) {
        yarp::os::Time::useSystemClock();
    }

    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    yarp::os::Bottle wbdHandsFTSensorsGroup = config.findGroup("wbd-hand-ft-sensors");
    if (wbdHandsFTSensorsGroup.isNull()) {
        yError() << LogPrefix << "REQUIRED parameter <wbd-hand-ft-sensors> NOT found";
        return false;
    }

    if (!wbdHandsFTSensorsGroup.check("leftHand") || !wbdHandsFTSensorsGroup.check("rightHand")) {
        yError() << LogPrefix << "REQUIRED parameter <leftHand> or <rightHand> NOT found";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    pImpl->nSensors = wbdHandsFTSensorsGroup.size() - 1;
    std::string leftHandFTPortName =
        wbdHandsFTSensorsGroup.check("leftHand", yarp::os::Value(false)).asString();
    std::string rightHandFTPortName =
        wbdHandsFTSensorsGroup.check("rightHand", yarp::os::Value(false)).asString();

    // ===========================
    // OPEN AND CONNECT YARP PORTS
    // ===========================

    if (!(pImpl->leftHandFTPort.open("/ICub/leftHantFTSensor:i")
          && yarp::os::Network::connect(leftHandFTPortName,
                                        pImpl->leftHandFTPort.getName().c_str()))) {
        yError() << LogPrefix << "Failed to open or connect to "
                 << pImpl->leftHandFTPort.getName().c_str();
        return false;
    }

    while (pImpl->leftHandFTPort.firstRun) {
        pImpl->leftHandFTPort.useCallback();
    }

    pImpl->sensorNames.push_back("leftWBDFTSensor");

    if (!(pImpl->rightHandFTPort.open("/ICub/rightHantFTSensor:i")
          && yarp::os::Network::connect(rightHandFTPortName,
                                        pImpl->rightHandFTPort.getName().c_str()))) {
        yError() << LogPrefix << "Failed to open or connect to "
                 << pImpl->rightHandFTPort.getName().c_str();
        return false;
    }

    while (pImpl->rightHandFTPort.firstRun) {
        pImpl->rightHandFTPort.useCallback();
    }

    pImpl->sensorNames.push_back("rightWBDFTSensor");

    std::string ft6dPrefix =
        getWearableName() + wearable::Separator + sensor::IForceTorque6DSensor::getPrefix();

    for (size_t s = 0; s < pImpl->sensorNames.size(); ++s) {
        // Create the new sensors
        auto ft6d = std::make_shared<ICubImpl::ICubForceTorque6DSensor>(
            pImpl.get(), ft6dPrefix + pImpl->sensorNames[s]);

        pImpl->ftSensorsMap.emplace(ft6dPrefix + pImpl->sensorNames[s],
                                    std::shared_ptr<ICubImpl::ICubForceTorque6DSensor>{ft6d});
    }

    return true;
}

// ---------------
// Generic Methods
// ---------------

wearable::WearableName ICub::getWearableName() const
{
    return pImpl->wearableName;
}

wearable::WearStatus ICub::getStatus() const
{
    wearable::WearStatus status = wearable::WearStatus::Ok;

    for (const auto& s : getAllSensors()) {
        if (s->getSensorStatus() != sensor::SensorStatus::Ok) {
            status = wearable::WearStatus::Error;
        }
    }

    // Default return status is Ok
    return status;
}

wearable::TimeStamp ICub::getTimeStamp() const
{
    // Stamp count should be always zero
    return {pImpl->timeStamp.time, 0};
}

bool ICub::close()
{
    return true;
}

// =========================
// IPreciselyTimed interface
// =========================
yarp::os::Stamp ICub::getLastInputStamp()
{
    // Stamp count should be always zero
    return yarp::os::Stamp(0, pImpl->timeStamp.time);
}

// ---------------------------
// Implemented Sensors Methods
// ---------------------------

wearable::SensorPtr<const wearable::sensor::ISensor>
ICub::getSensor(const wearable::sensor::SensorName name) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> sensors = getAllSensors();
    for (const auto& s : sensors) {
        if (s->getSensorName() == name) {
            return s;
        }
    }
    yWarning() << LogPrefix << "User specified name <" << name << "> not found";
    return nullptr;
}

wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
ICub::getSensors(const wearable::sensor::SensorType type) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> outVec;
    switch (type) {
        case sensor::SensorType::ForceTorque6DSensor: {
            outVec.reserve(pImpl->ftSensorsMap.size());
            for (const auto& ft6d : pImpl->ftSensorsMap) {
                outVec.push_back(static_cast<std::shared_ptr<sensor::ISensor>>(ft6d.second));
            }

            break;
        }
        default: {
            // yWarning() << LogPrefix << "Selected sensor type (" << static_cast<int>(type)
            //           << ") is not supported by ICub";
            return {};
        }
    }

    return outVec;
}

wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
ICub::getForceTorque6DSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->ftSensorsMap.find(name) == pImpl->ftSensorsMap.end()) {
        yError() << LogPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared point to the required sensor
    return static_cast<std::shared_ptr<sensor::IForceTorque6DSensor>>(pImpl->ftSensorsMap.at(name));
}
