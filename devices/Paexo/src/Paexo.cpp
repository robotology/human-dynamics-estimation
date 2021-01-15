/*
 * Copyright (C) 2020 iCub Facility
 * Authors: Yeshasvi Tirupachuri
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "Paexo.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>

#include <mutex>
#include <string>
#include <stdio.h>
#include <vector>
#include <assert.h>

const std::string DeviceName = "Paexo";
const std::string LogPrefix = DeviceName + wearable::Separator;
double period = 0.01;

using namespace wearable;
using namespace wearable::devices;

const std::string EOL = "\n"; //EOL character
const int MAX_LINE_LENGTH = 5000; // Maximum line length to read from serial port

struct PaexoData
{
    bool   updated;
    double angle;
    double force;
    double leverarm;
};

class Paexo::PaexoImpl
{
public:
    mutable std::mutex mutex;
    yarp::dev::ISerialDevice *iSerialDevice = nullptr;

    wearable::TimeStamp timeStamp;

    std::string portsPrefix;
    yarp::os::BufferedPort<yarp::os::Bottle> dataPort;

    // RPC related
    class CmdParser;
    std::unique_ptr<CmdParser> cmdPro;
    yarp::os::RpcServer rpcPort;

    std::string serialComPortName;

    // Paexo data buffer
    PaexoData paexoData;

    // Wearable variables
    wearable::WearableName wearableName;

    // Joint Sensor
    std::string jointSensorPrefix;
    const std::string jointSensorName = "Angle";
    class PaexoVirtualJointKinSensor;
    SensorPtr<PaexoVirtualJointKinSensor> paexoJointSensor;

    // 3D Force Sensor
    std::string forceSensorPrefix;
    const std::string forceSensorName = "SupportForce";
    class PaexoForce3DSensor;
    SensorPtr<PaexoForce3DSensor> paexoForceSensor;

    // 3D Torque Sensor
    std::string torqueSensorPrefix;
    const std::string torqueSensorName = "LeverArm";
    class PaexoTorque3DSensor;
    SensorPtr<PaexoTorque3DSensor> paexoTorqueSensor;

    // Motor Actuator
    std::string motorActuatorPrefix;
    const std::string motorActuatorName = "Actuator";
    class PaexoMotorActuator;
    DevicePtr<PaexoMotorActuator> paexoMotorActuator;

    // Number of sensors
    const int nSensors = 3; // Hardcoded for Paexo

    // Numbe of actuators
    const int nActuators = 1; // Hardcoded for Paexo

    // First data flag
    bool firstDataRead;

    // constructor
    PaexoImpl();
};

class Paexo::PaexoImpl::CmdParser : public yarp::os::PortReader
{

public:
    std::string cmdString;
    bool cmdUpdated = false;
    bool data_broadcast = false;
    bool measurement_status = false;

    bool read(yarp::os::ConnectionReader& connection) override
    {
        yarp::os::Bottle command, response;
        if(command.read(connection) && !cmdUpdated)
        {
            cmdString = command.toString();

            if (*cmdString.begin() == '"' && *(cmdString.end() - 1) == '"') {
                cmdString = cmdString.substr(1, cmdString.size()-2);
            }

            response.addString("Entered commands is " + cmdString);

            // TODO: This check can be better if status returns the measurement and broadcast information
            // Check for measurement related command
            if (cmdString == "start") {
                measurement_status = true;
            }
            else if (cmdString == "stop") {
                measurement_status = false;
            }

            // Check for data boardcast related command
            if (cmdString == "en_bc_data") {
                data_broadcast = true;
            }
            else if (cmdString == "di_bc_data") {
                data_broadcast = false;
            }

            cmdString.append(EOL);
            cmdUpdated = true;

            yarp::os::ConnectionWriter* reply = connection.getWriter();

            if (reply != NULL) {
                response.write(*reply);
            }
            else return false;
        }

        return true;
    }
};

Paexo::PaexoImpl::PaexoImpl()
    : cmdPro(new CmdParser())
{}

// Default constructor
Paexo::Paexo()
    : PeriodicThread(period)
    , pImpl{new PaexoImpl()}
{}

// Destructor
Paexo::~Paexo() = default;

bool Paexo::open(yarp::os::Searchable& config)
{
    // ==================================
    // Check the configuration parameters
    // ==================================

    // Period of the this device
    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period: " << period << "s";
    }
    else {
        period = config.find("period").asFloat64();
        yInfo() << LogPrefix << "Using the period : " << period << "s";
    }

    // Get port prefix name
    if (!(config.check("portsPrefixName") && config.find("portsPrefixName").isString())) {
        yInfo() << LogPrefix << "Using default port prefix /wearable/paexo";
    }
    else {
        pImpl->portsPrefix = config.find("portsPrefixName").asString();
        yInfo() << LogPrefix << "Using the ports prefix " << pImpl->portsPrefix;
    }

    if (!(config.check("wearableName") && config.find("wearableName").isString())) {
        yInfo() << LogPrefix << "Using default wearable name Paexo";
        pImpl->wearableName = DeviceName;
    }
    else {
        pImpl->wearableName = config.find("wearableName").asString();
        yInfo() << LogPrefix << "Using the wearable name " << pImpl->wearableName;
    }

    // ===================
    // Ports configuration
    // ===================
    if(!pImpl->dataPort.open(pImpl->portsPrefix + ":o")) {
        yError() << LogPrefix << "Failed to open data port " << pImpl->portsPrefix + ":o";
        return false;
    }

    if(!pImpl->rpcPort.open(pImpl->portsPrefix + "/rpc:i")) {
        yError() << LogPrefix << "Failed to open rpc port " << pImpl->portsPrefix + "/rpc:i";
        return false;
    }

    // Set rpc port reader
    pImpl->rpcPort.setReader(*pImpl->cmdPro);

    // Intialize wearable sensors
    pImpl->jointSensorPrefix = getWearableName() + sensor::IVirtualJointKinSensor::getPrefix();
    pImpl->paexoJointSensor = SensorPtr<PaexoImpl::PaexoVirtualJointKinSensor>{std::make_shared<PaexoImpl::PaexoVirtualJointKinSensor>(pImpl.get(),
                                                                                                                                       pImpl->jointSensorPrefix + pImpl->jointSensorName)};

    pImpl->forceSensorPrefix = getWearableName() + sensor::IForce3DSensor::getPrefix();
    pImpl->paexoForceSensor = SensorPtr<PaexoImpl::PaexoForce3DSensor>{std::make_shared<PaexoImpl::PaexoForce3DSensor>(pImpl.get(),
                                                                                                                      pImpl->forceSensorPrefix + pImpl->forceSensorName)};

    pImpl->torqueSensorPrefix = getWearableName() + sensor::ITorque3DSensor::getPrefix();
    pImpl->paexoTorqueSensor = SensorPtr<PaexoImpl::PaexoTorque3DSensor>{std::make_shared<PaexoImpl::PaexoTorque3DSensor>(pImpl.get(), pImpl->torqueSensorPrefix + pImpl->torqueSensorName)};

    // Initialize wearable actuators
    pImpl->motorActuatorPrefix = getWearableName() + actuator::IMotor::getPrefix();
    pImpl->paexoMotorActuator = DevicePtr<PaexoImpl::PaexoMotorActuator>{std::make_shared<PaexoImpl::PaexoMotorActuator>(pImpl.get(),
                                                                                                                         pImpl->motorActuatorPrefix + pImpl->motorActuatorName)};

    // Initialize paexo data buffer
    pImpl->paexoData.angle    = 0.0;
    pImpl->paexoData.force    = 0.0;
    pImpl->paexoData.leverarm = 0.0;
    pImpl->paexoData.updated  = false;

    // Initialize first data flag
    pImpl->firstDataRead = false;

    return true;

}

// =============================================
// Paexo implementation of VirtualJointKinSensor
// =============================================
class Paexo::PaexoImpl::PaexoVirtualJointKinSensor : public wearable::sensor::IVirtualJointKinSensor
{
public:
    Paexo::PaexoImpl* paexoImpl = nullptr;

    PaexoVirtualJointKinSensor(Paexo::PaexoImpl* impl,
                               const wearable::sensor::SensorName name = {},
                               const wearable::sensor::SensorStatus status = wearable::sensor::SensorStatus::Ok) // Default sensors status is set Ok
        : IVirtualJointKinSensor(name, status)
        , paexoImpl(impl)
    {
        // TODO: Initialization
    }

    ~PaexoVirtualJointKinSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    bool getJointPosition(double &position) const override
    {
        assert(paexoImpl != nullptr);

        std::lock_guard<std::mutex> lock(paexoImpl->mutex);
        position = paexoImpl->paexoData.angle;
        return true;
    }

    bool getJointVelocity(double &velocity) const override
    {
        assert(paexoImpl != nullptr);

        velocity = 0.0;

        return true;
    }

    bool getJointAcceleration(double &acceleration) const override
    {
        assert(paexoImpl != nullptr);

        acceleration = 0.0;

        return true;
    }
};

// =====================================
// Paexo implementation of Force3DSensor
// =====================================
class Paexo::PaexoImpl::PaexoForce3DSensor : public wearable::sensor::IForce3DSensor
{
public:
    Paexo::PaexoImpl* paexoImpl = nullptr;

    PaexoForce3DSensor(Paexo::PaexoImpl* impl,
                       const wearable::sensor::SensorName name = {},
                       const wearable::sensor::SensorStatus status = wearable::sensor::SensorStatus::Ok) // Default sensors status is set Ok
        : IForce3DSensor(name, status)
        , paexoImpl(impl)
    {
        // TODO: Initialization
    }

    ~PaexoForce3DSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    bool getForce3D(Vector3 &force) const override
    {
        assert(paexoImpl != nullptr);

        std::lock_guard<std::mutex> lock(paexoImpl->mutex);
        force[0] = paexoImpl->paexoData.force;
        force[1] = 0.0;
        force[2] = 0.0;
        return true;
    }
};

// ======================================
// Paexo implementation of Torque3DSensor
// ======================================
class Paexo::PaexoImpl::PaexoTorque3DSensor : public wearable::sensor::ITorque3DSensor
{
public:
    Paexo::PaexoImpl* paexoImpl = nullptr;

    PaexoTorque3DSensor(Paexo::PaexoImpl* impl,
                        const wearable::sensor::SensorName name = {},
                        const wearable::sensor::SensorStatus status = wearable::sensor::SensorStatus::Ok) // Default sensors status is set Ok
        : ITorque3DSensor(name, status)
        , paexoImpl(impl)
    {
        // TODO: Initialization
    }

    ~PaexoTorque3DSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    bool getTorque3D(Vector3 &torque) const override
    {
        assert(paexoImpl != nullptr);

        std::lock_guard<std::mutex> lock(paexoImpl->mutex);
        torque[0] = paexoImpl->paexoData.leverarm;
        torque[1] = 0.0;
        torque[2] = 0.0;
        return true;
    }
};

// =======================================
// Paexo implementation of Motor actutator
// =======================================
class Paexo::PaexoImpl::PaexoMotorActuator : public wearable::actuator::IMotor
{
public:
    Paexo::PaexoImpl* paexoImpl = nullptr;

    PaexoMotorActuator(Paexo::PaexoImpl* impl,
                       const wearable::actuator::ActuatorName name = {},
                       const wearable::actuator::ActuatorStatus status = wearable::actuator::ActuatorStatus::Ok) // Default actuator status is set ok
        : IMotor(name, status)
        , paexoImpl(impl)
    {
        //TODO: Initialization
    }

    bool setMotorPosition(double& value) const override
    {
        // TODO: Set the commanded value to the serial write
        yInfo() << LogPrefix << "Trying to set the Paexo actuation motor position to : " << value << " deg"; //NOTE: This is a dummy debug comment
        return true;
    }
};

void Paexo::run()
{
    // Send commands to BLE central serial port
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        if (pImpl->cmdPro->cmdUpdated) {

            int s = pImpl->cmdPro->cmdString.length();
            char c[s+1];
            std::strcpy(c, pImpl->cmdPro->cmdString.c_str());
            if (pImpl->iSerialDevice->send(c, s)) {
                pImpl->cmdPro->cmdUpdated = false;
            }
        }
    }


    char msg[MAX_LINE_LENGTH];
    int size = pImpl->iSerialDevice->receiveLine(msg, MAX_LINE_LENGTH);

    if (size > 1) {

        // Get timestamp
        pImpl->timeStamp.time = yarp::os::Time::now();

        // Check if the first char is a digit, if it is the received message is broadcast information
        if (isdigit(msg[0]) && (pImpl->cmdPro->measurement_status && pImpl->cmdPro->data_broadcast)) {

            // Prepare yarp bottle with serial message and write to yarp port
            yarp::os::Bottle& bc_data = pImpl->dataPort.prepare();
            bc_data.clear();
            bc_data.fromString(msg);

            pImpl->dataPort.write();

            // Add baroadcast data to buffer
            std::lock_guard<std::mutex> lock(pImpl->mutex);
            pImpl->paexoData.angle = bc_data.get(0).asDouble();
            pImpl->paexoData.force = bc_data.get(1).asDouble();
            pImpl->paexoData.leverarm = bc_data.get(2).asDouble();
            pImpl->paexoData.updated = true;

        }
        else if(!isdigit(msg[0])) {
            yInfo() << LogPrefix << msg;
        }
    }

    if (!pImpl->firstDataRead && pImpl->paexoData.updated) {

        // Set sensor status
        // TODO: Check if the sensor status can be modified from here
        pImpl->firstDataRead = true;
    }
}

bool Paexo::close()
{
    detach();
    return true;
}

bool Paexo::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is a nullptr";
        return false;
    }

    if (pImpl->iSerialDevice || !poly->view(pImpl->iSerialDevice) || !pImpl->iSerialDevice) {
        yError() << LogPrefix << "Failed to view the ISerialDevice interface from the attached polydriver device";
        return false;
    }
    else {
        yInfo() << LogPrefix << "ISerialDevice interface viewed correctly";
    }

    // Get the comport name of the serial device
    pImpl->serialComPortName = poly->getValue("comport").asString();

    // TODO: Check if the ISerialDevice interface is configured correctly
    // I do not see any method to check this

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the period thread.";
        return false;
    }

    yInfo() << LogPrefix << "attach() successful";
    return true;

}

bool Paexo::detach()
{
    while(yarp::os::PeriodicThread::isRunning()) {
        yarp::os::PeriodicThread::stop();
    }

    pImpl->iSerialDevice = nullptr;
    return true;
}

bool Paexo::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    // A single serial device will be streaming data from all the sensors from the FTShoes
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached yarp Serial device";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool Paexo::detachAll()
{
    return detach();
}

void Paexo::threadRelease()
{}

// =========================
// IPreciselyTimed interface
// =========================
yarp::os::Stamp Paexo::getLastInputStamp()
{
    // Stamp count should be always zero
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return yarp::os::Stamp(0, pImpl->timeStamp.time);
}

// ---------------------------
// Implement Sensors Methods
// ---------------------------

wearable::WearableName Paexo::getWearableName() const
{
    return pImpl->wearableName + wearable::Separator;
}

wearable::WearStatus Paexo::getStatus() const
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

wearable::TimeStamp Paexo::getTimeStamp() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return {pImpl->timeStamp.time, 0};
}


wearable::SensorPtr<const wearable::sensor::ISensor>
Paexo::getSensor(const wearable::sensor::SensorName name) const
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
Paexo::getSensors(const wearable::sensor::SensorType aType) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> outVec;
    outVec.reserve(pImpl->nSensors);
    switch (aType) {
        case sensor::SensorType::VirtualJointKinSensor: {
            outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(pImpl->paexoJointSensor));
            break;
        }
        case sensor::SensorType::Force3DSensor: {
            outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(pImpl->paexoForceSensor));
            break;
        }
        case sensor::SensorType::Torque3DSensor: {
            outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(pImpl->paexoTorqueSensor));
            break;
        }
        default: {
            return {};
        }
    }

    return outVec;
}

wearable::DevicePtr<const wearable::actuator::IActuator>
Paexo::getActuator(const wearable::actuator::ActuatorName name) const
{
    wearable::VectorOfDevicePtr<const wearable::actuator::IActuator> actuators = getAllActuators();

    for (const auto& a : actuators)
    {
        if (a->getActuatorName() == name)
        {
            return a;
        }
    }
    yWarning() << LogPrefix << "User specified actuator name <" << name << "> not found";
    return nullptr;
}

wearable::VectorOfDevicePtr<const wearable::actuator::IActuator>
Paexo::getActuators(const wearable::actuator::ActuatorType aType) const
{
    wearable::VectorOfDevicePtr<const wearable::actuator::IActuator> outVec;
    outVec.reserve(pImpl->nActuators);

    switch (aType) {
        case wearable::actuator::ActuatorType::Motor: {
            outVec.push_back(static_cast<DevicePtr<actuator::IActuator>>(pImpl->paexoMotorActuator));
            break;
        }
        default: {
            return {};
        }
    }

    return outVec;
}

// ------------
// JOINT Sensor
// ------------
wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
Paexo::getVirtualJointKinSensor(const wearable::sensor::SensorName name) const
{

    // Check if user-provided name corresponds to an available sensor
    if (name == pImpl->jointSensorPrefix + pImpl->jointSensorName) {
        yError() << LogPrefix << "Invalid sensor name " << name;
        return nullptr;
    }

    // Return a shared point to the required sensor
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>&>(
        *pImpl->paexoJointSensor);
}

// ------------
// FORCE Sensor
// ------------
wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
Paexo::getForce3DSensor(const wearable::sensor::SensorName name) const
{

    // Check if user-provided name corresponds to an available sensor
    if (name == pImpl->forceSensorPrefix + pImpl->forceSensorName) {
        yError() << LogPrefix << "Invalid sensor name " << name;
        return nullptr;
    }

    // Return a shared point to the required sensor
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::IForce3DSensor>&>(
        *pImpl->paexoForceSensor);
}

// -------------
// TORQUE Sensor
// -------------
wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
Paexo::getTorque3DSensor(const wearable::sensor::SensorName name) const
{

    // Check if user-provided name corresponds to an available sensor
    if (name == pImpl->torqueSensorPrefix + pImpl->torqueSensorName) {
        yError() << LogPrefix << "Invalid sensor name " << name;
        return nullptr;
    }

    // Return a shared point to the required sensor
    return dynamic_cast<wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>&>(
        *pImpl->paexoTorqueSensor);
}

// ---------------
// MOTORO Actuator
// ---------------
wearable::DevicePtr<const actuator::IMotor>
Paexo::getMotorActuator(const actuator::ActuatorName name) const
{
    // Check if user-provided name corresponds to an available actuator
    if (name == pImpl->motorActuatorPrefix + pImpl->motorActuatorName) {
        yError() << LogPrefix << "Invalid actuator name " << name;
        return nullptr;
    }

    // Return a shared point to the required sensor
    return dynamic_cast<wearable::DevicePtr<const wearable::actuator::IMotor>&>(
        *pImpl->paexoMotorActuator);
}
