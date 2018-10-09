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
        yarp::os::Bottle wrenchValues;
        bool firstDataReceived = false;
        using yarp::os::BufferedPort<yarp::os::Bottle>::onRead;
        virtual void onRead(yarp::os::Bottle& wrench) {

            //std::cout << "Received wrench of size : " << wrench.size() << std::endl;
            //std::cout << "Wrench values : " << wrench.toString().c_str() << std::endl;
            if (wrench.size() == 6) {
                this->wrenchValues.copy(wrench);
            }

            while (this->firstDataReceived != true)
            {
                std::cout << "Set firstDataReceived flag : true" << std::endl;
                this->firstDataReceived = true;
            }
        }

        yarp::os::Bottle getWrench() {
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
        std::cout << "ICubForceTorque6DSensor constructor" << std::endl;
        if (icubImpl->leftHandFTPort.getFirstDataFlag() || icubImpl->rightHandFTPort.getFirstDataFlag()) {
            auto nonConstThis = const_cast<ICubForceTorque6DSensor*>(this);
            nonConstThis->setStatus(wearable::sensor::SensorStatus::Ok);
            std::cout << "ICubForceTorque6DSensor constructor setting sensor status flag" << std::endl;
        }

    }

    ~ICubForceTorque6DSensor() override = default;

     void setStatus(const wearable::sensor::SensorStatus status) {
         //std::cout << "Status : " << status << std::endl;
         m_status = status;
         switch(m_status)
         {
         case sensor::SensorStatus::Error:
             std::cout << "Sensor Status : Error" << std::endl;
             break;
         case sensor::SensorStatus::Ok:
             //std::cout << "Sensor Status : Ok" << std::endl;
             break;
         case sensor::SensorStatus::Calibrating:
             std::cout << "Sensor Status : Calibrating" << std::endl;
             break;
         case sensor::SensorStatus::Timeout:
             std::cout << "Sensor Status : Timeout" << std::endl;
             break;
         case sensor::SensorStatus::Unknown:
             std::cout << "Sensor Status : Unknown" << std::endl;
             break;
         case sensor::SensorStatus::WaitingForFirstRead:
             std::cout << "Sensor Status : WaitingForFirstRead" << std::endl;
             break;
         default:
             std::cout << "Sensor Status : Default case" << std::endl;
             break;
         }
     }

    // ==============================
    // IForceTorque6DSensor interface
    // ==============================
    bool getForceTorque6D(Vector3& force3D, Vector3& torque3D) const override {
        //std::cout << "Inside getForceTorque6D" << std::endl;
        if (!icubImpl) {
            return false;
        }

        yarp::os::Bottle wrench;

        // Reading wrench from WBD ports
        if (this->m_name == icubImpl->wearableName + sensor::IForceTorque6DSensor::getPrefix() + "leftWBDFTSensor") {

            //std::cout << "Inside leftWBDFTSensor" << std::endl;
            icubImpl->leftHandFTPort.useCallback();
            wrench = icubImpl->leftHandFTPort.getWrench();

            icubImpl->timeStamp.time = yarp::os::Time::now();
        }
        else if(this->m_name == icubImpl->wearableName + sensor::IForceTorque6DSensor::getPrefix() + "rightWBDFTSensor") {

            //std::cout << "Inside rightWBDFTSensor" << std::endl;
            icubImpl->rightHandFTPort.useCallback();
            wrench = icubImpl->rightHandFTPort.getWrench();

            icubImpl->timeStamp.time = yarp::os::Time::now();
        }

        //TODO: Check if the bottle is valid
        if(wrench.size() == 6) {
            //std::cout << "Correct wrench data received" << std::endl;
            force3D[0] = wrench.get(0).asDouble();
            force3D[1] = wrench.get(1).asDouble();
            force3D[2] = wrench.get(2).asDouble();

            torque3D[0] = wrench.get(3).asDouble();
            torque3D[1] = wrench.get(4).asDouble();
            torque3D[2] = wrench.get(5).asDouble();
        }
        else {
            //std::cout << "Wrong wrench data received" << std::endl;
            force3D.fill(0.0);
            torque3D.fill(0.0);

            // Set sensor status to error if wrong data is read
            auto nonConstThis = const_cast<ICubForceTorque6DSensor*>(this);
            nonConstThis->setStatus(wearable::sensor::SensorStatus::Error);
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
