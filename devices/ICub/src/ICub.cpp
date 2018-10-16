/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "ICub.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

using namespace wearable::devices;

const std::string LogPrefix = "ICub :";

class ICub::ICubImpl
{
public:
    class ICubForceTorque6DSensor;

    wearable::TimeStamp timeStamp;

    size_t nSensors;
    std::vector<std::string> sensorNames;

    bool leftHandFTDataReceived  = false;
    bool rightHandFTDataReceived = false;

    template<typename T>
    struct icubDeviceSensors
    {
        std::shared_ptr<T> icubSensor;
        size_t index;
    };

    std::map<std::string, icubDeviceSensors<ICubForceTorque6DSensor>> ftSensorsMap;

    const WearableName wearableName = "ICub" + wearable::Separator;

    class wrenchPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
    public:
        std::vector<double> wrenchValues;
        bool firstDataReceived = false;
        std::mutex mtx;

        using yarp::os::BufferedPort<yarp::os::Bottle>::onRead;
        virtual void onRead(yarp::os::Bottle& wrench) {
            if (!wrench.isNull()) {
                std::lock_guard<std::mutex> lock (mtx);

                this->wrenchValues.clear(); // Clear the vector of previous values

                if (this->wrenchValues.empty()) {
                    this->wrenchValues.resize(wrench.size());
                    for (size_t i = 0; i < wrench.size(); ++i) {
                        this->wrenchValues.at(i) = wrench.get(i).asDouble();
                    }
                }
            }
            else {
                yWarning() << LogPrefix << "[wrenchPort] read an invalid wrench bottle";
            }

            while (this->firstDataReceived != true)
            {
                this->firstDataReceived = true;
            }
        }

        std::vector<double> getWrench() {
            std::lock_guard<std::mutex> lck (mtx);
            return this->wrenchValues;
        }

        bool getFirstDataFlag() {
            return this->firstDataReceived;
        }
    };

    wrenchPort leftHandFTPort;
    wrenchPort rightHandFTPort;
};

// ==========================================
// ICub implementation of ForceTorque6DSensor
// ==========================================
class ICub::ICubImpl::ICubForceTorque6DSensor
        : public wearable::sensor::IForceTorque6DSensor
{
public:
    ICub::ICubImpl* icubImpl = nullptr;

    // ------------------------
    // Constructor / Destructor
    // ------------------------
    ICubForceTorque6DSensor(
            ICub::ICubImpl* impl,
            const wearable::sensor::SensorName name = {},
            const wearable::sensor::SensorStatus status = wearable::sensor::SensorStatus::WaitingForFirstRead) //Default sensor status
            : IForceTorque6DSensor(name, status)
            , icubImpl(impl)
    {
        //Set the sensor status Ok after receiving the first data on the wrench ports
        if (icubImpl->leftHandFTPort.getFirstDataFlag() || icubImpl->rightHandFTPort.getFirstDataFlag()) {
            auto nonConstThis = const_cast<ICubForceTorque6DSensor*>(this);
            nonConstThis->setStatus(wearable::sensor::SensorStatus::Ok);
        }

    }

    ~ICubForceTorque6DSensor() override = default;

     void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    // ==============================
    // IForceTorque6DSensor interface
    // ==============================
    bool getForceTorque6D(Vector3& force3D, Vector3& torque3D) const override {
        if (!icubImpl) {
            return false;
        }

        std::vector<double> wrench;

        // Reading wrench from WBD ports
        if (this->m_name == icubImpl->wearableName + sensor::IForceTorque6DSensor::getPrefix() + "leftWBDFTSensor") {
            icubImpl->leftHandFTPort.useCallback();
            wrench = icubImpl->leftHandFTPort.getWrench();

            icubImpl->timeStamp.time = yarp::os::Time::now();
        }
        else if(this->m_name == icubImpl->wearableName + sensor::IForceTorque6DSensor::getPrefix() + "rightWBDFTSensor") {
            icubImpl->rightHandFTPort.useCallback();
            wrench = icubImpl->rightHandFTPort.getWrench();

            icubImpl->timeStamp.time = yarp::os::Time::now();
        }


        // Check the size of wrench values
        if(wrench.size() == 6) {
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
            //auto nonConstThis = const_cast<ICubForceTorque6DSensor*>(this);
            //nonConstThis->setStatus(wearable::sensor::SensorStatus::Error);
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
    if (!yarp::os::Time::isSystemClock())
    {
        yarp::os::Time::useSystemClock();
    }

    yarp::os::Bottle wbdHandsFTSensorsSet = config.findGroup("wbd-hand-ft-sensors");
    if(wbdHandsFTSensorsSet.isNull()) {
        yError() << LogPrefix << "REQUIRED parameter <wbd-hand-ft-sensors> NOT found";
        return false;
    }
    else {

        if (!wbdHandsFTSensorsSet.check("nSensors")) {
            yError() << LogPrefix << "REQUIRED parameter <nSensors> NOT found";
            return false;
        }
        else {

            pImpl->nSensors = wbdHandsFTSensorsSet.check("nSensors",yarp::os::Value(false)).asInt();

            if (!wbdHandsFTSensorsSet.check("leftHand") || !wbdHandsFTSensorsSet.check("rightHand")) {
                yError() << LogPrefix << "REQUIRED parameter <leftHand> or <rightHand> NOT found";
                return false;
            }
            else {

                // Check YARP network
                if(!yarp::os::Network::initialized())
                {
                    yarp::os::Network::init();
                }

                // Connect to wbd left hand ft port
                std::string leftHandFTPortName = wbdHandsFTSensorsSet.check("leftHand",yarp::os::Value(false)).asString();

                if(pImpl->leftHandFTPort.open("/ICub/leftHantFTSensor:i"))
                {
                    if(!yarp::os::Network::connect(leftHandFTPortName,pImpl->leftHandFTPort.getName().c_str()))
                    {
                        yError() << LogPrefix << "Failed to connect " << leftHandFTPortName << " and " << pImpl->leftHandFTPort.getName().c_str();
                        return false;
                    }
                    else {
                        while (!pImpl->leftHandFTPort.getFirstDataFlag()) {
                            pImpl->leftHandFTPort.useCallback();
                        }
                        pImpl->sensorNames.push_back("leftWBDFTSensor");
                    }
                }
                else
                {
                    yError() << LogPrefix << "Failed to open " << pImpl->leftHandFTPort.getName().c_str();
                    return false;
                }

                // Connect to wbd right hand ft port
                std::string rightHandFTPortName = wbdHandsFTSensorsSet.check("rightHand",yarp::os::Value(false)).asString();

                if(pImpl->rightHandFTPort.open("/ICub/rightHantFTSensor:i"))
                {
                    if(!yarp::os::Network::connect(rightHandFTPortName,pImpl->rightHandFTPort.getName().c_str()))
                    {
                        yError() << LogPrefix << "Failed to connect " << rightHandFTPortName << " and " << pImpl->rightHandFTPort.getName().c_str();
                        return false;
                    }
                    else {
                        while (!pImpl->rightHandFTPort.getFirstDataFlag()) {
                            pImpl->rightHandFTPort.useCallback();
                        }
                        pImpl->sensorNames.push_back("rightWBDFTSensor");
                    }
                }
                else
                {
                    yError() << LogPrefix << "Failed to open " << pImpl->rightHandFTPort.getName().c_str();
                    return false;
                }

            }

        }

    }

    std::string ft6dPrefix = getWearableName() + sensor::IForceTorque6DSensor::getPrefix();

    for (size_t s = 0; s < pImpl->sensorNames.size(); ++s) {
        // Create the new sensors
         auto ft6d = std::make_shared<ICubImpl::ICubForceTorque6DSensor>(
            pImpl.get(), ft6dPrefix + pImpl->sensorNames[s]);

         pImpl->ftSensorsMap.emplace(
                     ft6dPrefix + pImpl->sensorNames[s],
                     ICubImpl::icubDeviceSensors<ICubImpl::ICubForceTorque6DSensor>{ft6d,s});
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
            yError() << LogPrefix << "The status of" << s->getSensorName() << "is not Ok ("
                     << static_cast<int>(s->getSensorStatus()) << ")";
            status = wearable::WearStatus::Error;
        }
    }

    // Default return status is Ok
    return status;
}


wearable::TimeStamp ICub::getTimeStamp() const
{
    // Stamp count should be always zero
    return {pImpl->timeStamp.time,0};
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
            outVec.reserve(pImpl->nSensors);
            for (const auto& ft6d : pImpl->ftSensorsMap) {
                outVec.push_back(
                    static_cast<std::shared_ptr<sensor::ISensor>>(ft6d.second.icubSensor));
            }

            break;
        }
        default: {
            //yWarning() << LogPrefix << "Selected sensor type (" << static_cast<int>(type)
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
    if (pImpl->ftSensorsMap.find(static_cast<std::string>(name))
        == pImpl->ftSensorsMap.end()) {
        yError() << LogPrefix << "Invalid sensor name";
        return nullptr;
    }

    //return a shared point to the required sensor
    return static_cast<std::shared_ptr<sensor::IForceTorque6DSensor>>(
        pImpl->ftSensorsMap.at(static_cast<std::string>(name)).icubSensor);
}
