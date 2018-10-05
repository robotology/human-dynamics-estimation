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
    yarp::os::BufferedPort<yarp::os::Bottle> leftHandFTPort;
    yarp::os::BufferedPort<yarp::os::Bottle> rightHandFTPort;

    template<typename T>
    struct icubDeviceSensors
    {
        std::shared_ptr<T> icubSensor;
        size_t index;
    };

    std::map<std::string, icubDeviceSensors<ICubForceTorque6DSensor>> ftSensorsMap;

    const std::map<std::string, wearable::sensor::SensorStatus> icubSensorStatutsMap{
        {"Ok", sensor::SensorStatus::Ok},
        {"Error", sensor::SensorStatus::Error}
    };

    std::string sensorStatus;

    const WearableName wearableName = "ICub" + wearable::Separator;
};

// ==========================================
// ICub implementation of ForceTorque6DSensor
// ==========================================
class ICub::ICubImpl::ICubForceTorque6DSensor
        : public wearable::sensor::IForceTorque6DSensor
{
private:
    yarp::os::Bottle* wrench;

public:
    ICub::ICubImpl* icubImpl = nullptr;

    // ------------------------
    // Constructor / Destructor
    // ------------------------
    ICubForceTorque6DSensor(
            ICub::ICubImpl* impl,
            const wearable::sensor::SensorName name = {},
            const wearable::sensor::SensorStatus status = wearable::sensor::SensorStatus::Unknown)
            : IForceTorque6DSensor(name, status)
            , icubImpl(impl)
    {}

    ~ICubForceTorque6DSensor() override = default;

    // ==============================
    // IForceTorque6DSensor interface
    // ==============================
    bool getForceTorque6D(Vector3& force3D, Vector3& torque3D) const override {
        if (!icubImpl) {
            return false;
        }

        // Reading wrench from WBD ports
        if (this->m_name == "leftWBDFTSensor") {
            icubImpl->leftHandFTPort.read(wrench);
            icubImpl->timeStamp.time = yarp::os::Time::now();
            icubImpl->sensorStatus = "Ok";
        }
        else if(this->m_name == "rightWBDFTSensor") {
            icubImpl->rightHandFTPort.read(wrench);
            icubImpl->timeStamp.time = yarp::os::Time::now();
            icubImpl->sensorStatus = "Ok";
        }

        if(!wrench->isNull() && wrench->size() != 6) {
            force3D[0] = wrench->get(0).asDouble();
            force3D[1] = wrench->get(1).asDouble();
            force3D[2] = wrench->get(2).asDouble();

            torque3D[0] = wrench->get(3).asDouble();
            torque3D[1] = wrench->get(4).asDouble();
            torque3D[2] = wrench->get(5).asDouble();
        }
        else {
            force3D.fill(0.0);
            torque3D.fill(0.0);
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

    // Default sensor status
    pImpl->sensorStatus = "Error";

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
    return pImpl->icubSensorStatutsMap.at(pImpl->sensorStatus);
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
            yWarning() << LogPrefix << "Selected sensor type (" << static_cast<int>(type)
                       << ") is not supported by ICub";
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
