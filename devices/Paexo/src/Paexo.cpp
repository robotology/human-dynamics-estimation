/*
 * Copyright (C) 2020 iCub Facility
 * Authors: Yeshasvi Tirupachuri
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "Paexo.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/TypedReaderCallback.h>

#include <assert.h>
#include <mutex>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <vector>

#ifdef ENABLE_PAEXO_USE_iFEELDriver
#include <iFeelDriver/iFeelDriver/iFeelDriver.h>
#endif

const std::string DeviceName = "Paexo";
const std::string LogPrefix = DeviceName + wearable::Separator;
double period = 0.01;

using namespace wearable;
using namespace wearable::devices;

using YarpBufferedPort = yarp::os::BufferedPort<yarp::os::Bottle>;

const std::string EOL = "\n"; // EOL character
const int MAX_LINE_LENGTH = 5000; // Maximum line length to read from serial port

struct PaexoData
{
    bool updated;
    double angle;
    double force;
    double leverarm;
};

class Paexo::PaexoImpl
{
public:
    std::unique_ptr<yarp::os::Network> network = nullptr;

    mutable std::mutex mutex;
    yarp::dev::ISerialDevice* iSerialDevice = nullptr;

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

    // 6D Force-Torque Sensors
    std::string ftSensorPrefix;
    const std::string ftSensorName = "ArmForceTorque";
    class PaexoForceTorque6DSensor;
    SensorPtr<PaexoForceTorque6DSensor> paexoLeftArmFTSensor;
    SensorPtr<PaexoForceTorque6DSensor> paexoRightArmFTSensor;

    // Motor Actuator
    std::string motorActuatorPrefix;
    const std::string motorActuatorName = "Motor";
    class PaexoMotorActuator;
    ElementPtr<PaexoMotorActuator> paexoLeftMotorActuator;
    ElementPtr<PaexoMotorActuator> paexoRightMotorActuator;

    // Motor actuator yarp port control
    class PaexoMotorControlPort;
    std::unique_ptr<PaexoMotorControlPort> paexoLeftMotorControlPort;
    std::unique_ptr<PaexoMotorControlPort> paexoRightMotorControlPort;

    // Helper function to split wearable element name
    // TODO: To be moved to wearables utilities
    inline std::vector<std::string> split(const std::string& s, const std::string& delimiter)
    {
        std::size_t pos_start = 0, pos_end, delim_len = delimiter.length();
        std::string token;
        std::vector<std::string> res;

        while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
            token = s.substr(pos_start, pos_end - pos_start);
            pos_start = pos_end + delim_len;
            res.push_back(token);
        }

        res.push_back(s.substr(pos_start));
        return res;
    }

    inline std::string getValidYarpName(const std::string& actuatorName)
    {
        std::string portName;

        // Get valid port name from actuator name
        const auto vecStr = split(actuatorName, wearable::Separator);

        for (const auto& str : vecStr) {
            if (portName.empty()) {
                portName = str;
            }
            else {
                portName = portName + '/' + str;
            }
        }

        portName = "/" + portName + ":i";

        return portName;
    }

    // Number of sensors
    const int nSensors = 3; // Hardcoded for Paexo

    // Numbe of actuators
    const int nActuators = 1; // Hardcoded for Paexo

    // First data flag
    bool firstDataRead;

    // constructor
    PaexoImpl();

#ifdef ENABLE_PAEXO_USE_iFEELDriver
    // iFeelDriver
    bool useiFeelDriver;
    iFeel::SerialConfig serial; // serial port configuraton
    iFeel::iFeelSystemConfig ifeelConfig; // iFeelDriver configuration
    iFeel::iFeelSystemData ifeelData; // iFeel sensors data
    std::unique_ptr<iFeel::iFeelDriver> ifeelDriver{nullptr};
#endif
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
        if (command.read(connection) && !cmdUpdated) {
            cmdString = command.toString();

            if (*cmdString.begin() == '"' && *(cmdString.end() - 1) == '"') {
                cmdString = cmdString.substr(1, cmdString.size() - 2);
            }

            response.addString("Entered commands is " + cmdString);

            // TODO: This check can be better if status returns the measurement and broadcast
            // information Check for measurement related command
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
            else
                return false;
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

class Paexo::PaexoImpl::PaexoMotorControlPort : public YarpBufferedPort
{
public:
    std::string portName;
    ElementPtr<Paexo::PaexoImpl::PaexoMotorActuator> paexoMotorActuator = nullptr;

    void onRead(yarp::os::Bottle& motorCommand) override;

    // Constructor
    PaexoMotorControlPort(const std::string& name,
                          ElementPtr<Paexo::PaexoImpl::PaexoMotorActuator>& actuator)
    {
        portName = name;
        // Set the actutor
        if (actuator == nullptr) {
            throw std::runtime_error(LogPrefix + "Actuator passing error for motor control port "
                                     + portName);
        }

        paexoMotorActuator = actuator;
    }
};

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

    // ==========================================
    // Check iFeelDriver configuration parameters
    // ==========================================

    yarp::os::Bottle& ifeelDriverGroup = config.findGroup("iFeelDriver");

#ifdef ENABLE_PAEXO_USE_iFEELDriver
    pImpl->useiFeelDriver = false;
    if (!ifeelDriverGroup.isNull()) {
        yarp::os::Bottle& ifeelDriverSerialGroup = ifeelDriverGroup.findGroup("serial-config");

        if (ifeelDriverSerialGroup.isNull()) {
            yError() << LogPrefix
                     << "iFeelDriver serial-config is not provided, cannot configure iFeelDriver "
                        "to be used with Paexo";
            return false;
        }

        // Configure iFeelDriver serial device
        if (!ifeelDriverSerialGroup.check("serial-port-name")) {
            yError() << LogPrefix << "REQUIRED parameter <serial-port-name> NOT found.";
            return false;
        }
        pImpl->serial.name = ifeelDriverSerialGroup.find("serial-port-name").asString();
        yInfo() << LogPrefix << "iFeelDriver configuring through the serial port "
                << pImpl->serial.name;

        // Serial timeout
        uint32_t timeoutms;
        if (!ifeelDriverSerialGroup.check("serial-timeout-ms")) {
            timeoutms = 100;
            yWarning() << LogPrefix
                       << "Optional parameter <serial-timeout-ms> NOT found. Setting default value:"
                       << timeoutms << " ms.";
        }
        else {
            timeoutms =
                static_cast<uint32_t>(ifeelDriverSerialGroup.find("serial-timeout-ms").asInt32());
        }
        pImpl->serial.tout = serial::Timeout::simpleTimeout(timeoutms);

        // TODO: Some of the serial ports can be passed through configuration file
        pImpl->serial.size = 256;
        pImpl->serial.sleep_us = 100;
        pImpl->serial.baudrate = 9600;
        pImpl->serial.eol = END_OF_LINE_STR;

        // iFeelDriver configuration parameters
        if (!ifeelDriverGroup.check("ifeeldriver-config-timeout-s")) {
            pImpl->ifeelConfig.ConfigurationDuration = std::chrono::seconds(1);
            yWarning() << LogPrefix
                       << "Optional parameter <ifeeldriver-config-timeout-s> NOT found. Using "
                          "default value of 1 second";
        }
        else {
            pImpl->ifeelConfig.ConfigurationDuration = static_cast<std::chrono::seconds>(
                ifeelDriverGroup.find("ifeeldriver-config-timeout-s").asInt32());
        }

        // iFeelDriver nodes configuration list
        if (!(ifeelDriverGroup.check("ifeeldriver-config-nodes")
              && ifeelDriverGroup.find("ifeeldriver-config-nodes").isList())) {
            yError() << LogPrefix
                     << "Required parameter <ifeeldriver-config-nodes> NOT Found or is not a valid "
                        "list of NodeIDs";
            return false;
        }

        yarp::os::Bottle* nodeList = ifeelDriverGroup.find("ifeeldriver-config-nodes").asList();

        // NOTE: Assuming only one FTShoeNode is configured through iFeelDriver
        assert(nodeList->size() == 1 && nodeList->get(i).asInt32() == 2);

        for (size_t i = 0; i < nodeList->size(); i++) {
            pImpl->ifeelConfig.ConfigurationNodes.push_back(nodeList->get(i).asInt32());
        }

        pImpl->ifeelDriver =
            std::make_unique<iFeel::iFeelDriver>(pImpl->serial, pImpl->ifeelConfig);

        // initialize iFeel driver
        if (!pImpl->ifeelDriver->init()) {
            yError() << LogPrefix << "Failed to initialize iFeelDriver.";
            return false;
        }

        yInfo() << LogPrefix << "iFeelDriver is configured to be used with Paexo ";
    }
    else {
        yWarning() << LogPrefix << "iFeelDriver is not configured to be used with Paexo ";
    }

#endif

    // ===================
    // Ports configuration
    // ===================
    if (!pImpl->dataPort.open(pImpl->portsPrefix + ":o")) {
        yError() << LogPrefix << "Failed to open data port " << pImpl->portsPrefix + ":o";
        return false;
    }

    if (!pImpl->rpcPort.open(pImpl->portsPrefix + "/rpc:i")) {
        yError() << LogPrefix << "Failed to open rpc port " << pImpl->portsPrefix + "/rpc:i";
        return false;
    }

    // Set rpc port reader
    pImpl->rpcPort.setReader(*pImpl->cmdPro);

    // Intialize wearable sensors
    pImpl->jointSensorPrefix = getWearableName() + sensor::IVirtualJointKinSensor::getPrefix();
    pImpl->paexoJointSensor = SensorPtr<PaexoImpl::PaexoVirtualJointKinSensor>{
        std::make_shared<PaexoImpl::PaexoVirtualJointKinSensor>(
            pImpl.get(), pImpl->jointSensorPrefix + pImpl->jointSensorName)};

    pImpl->forceSensorPrefix = getWearableName() + sensor::IForce3DSensor::getPrefix();
    pImpl->paexoForceSensor =
        SensorPtr<PaexoImpl::PaexoForce3DSensor>{std::make_shared<PaexoImpl::PaexoForce3DSensor>(
            pImpl.get(), pImpl->forceSensorPrefix + pImpl->forceSensorName)};

    pImpl->torqueSensorPrefix = getWearableName() + sensor::ITorque3DSensor::getPrefix();
    pImpl->paexoTorqueSensor =
        SensorPtr<PaexoImpl::PaexoTorque3DSensor>{std::make_shared<PaexoImpl::PaexoTorque3DSensor>(
            pImpl.get(), pImpl->torqueSensorPrefix + pImpl->torqueSensorName)};

#ifdef ENABLE_PAEXO_USE_iFEELDriver
    pImpl->ftSensorPrefix = getWearableName() + sensor::IForceTorque6DSensor::getPrefix();
    pImpl->paexoLeftArmFTSensor = SensorPtr<PaexoImpl::PaexoForceTorque6DSensor>{
        std::make_shared<PaexoImpl::PaexoForceTorque6DSensor>(
            pImpl.get(), pImpl->ftSensorPrefix + "Left" + pImpl->ftSensorName)};
    pImpl->paexoRightArmFTSensor = SensorPtr<PaexoImpl::PaexoForceTorque6DSensor>{
        std::make_shared<PaexoImpl::PaexoForceTorque6DSensor>(
            pImpl.get(), pImpl->ftSensorPrefix + "Right" + pImpl->ftSensorName)};
#endif

    // Initialize wearable actuators for left and right motor
    pImpl->motorActuatorPrefix = getWearableName() + actuator::IMotor::getPrefix();
    const std::string leftActuatorName =
        pImpl->motorActuatorPrefix + "Left" + pImpl->motorActuatorName;
    pImpl->paexoLeftMotorActuator =
        std::make_shared<PaexoImpl::PaexoMotorActuator>(pImpl.get(), leftActuatorName);

    const std::string rightActuatorName =
        pImpl->motorActuatorPrefix + "Right" + pImpl->motorActuatorName;
    pImpl->paexoRightMotorActuator =
        std::make_shared<PaexoImpl::PaexoMotorActuator>(pImpl.get(), rightActuatorName);

    // Initialize yarp control ports

    // Check yarp network initialization
    pImpl->network = std::make_unique<yarp::os::Network>();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << LogPrefix << "YARP server wasn't found active.";
        return false;
    }

    yInfo() << LogPrefix << "Initiailizing PaexoMotorControlPort for " << leftActuatorName;

    const std::string leftActuatorPortName = pImpl->getValidYarpName(leftActuatorName);
    pImpl->paexoLeftMotorControlPort = std::make_unique<PaexoImpl::PaexoMotorControlPort>(
        leftActuatorPortName, pImpl->paexoLeftMotorActuator);
    pImpl->paexoLeftMotorControlPort->useCallback();

    if (!pImpl->paexoLeftMotorControlPort->open(leftActuatorPortName)) {
        yError() << LogPrefix << "Failed to open paexo motor control port " << leftActuatorPortName;
        return false;
    }

    yInfo() << LogPrefix << "Initiailizing PaexoMotorControlPort for " << rightActuatorName;

    const std::string rightActuatorPortName = pImpl->getValidYarpName(rightActuatorName);
    pImpl->paexoRightMotorControlPort = std::make_unique<PaexoImpl::PaexoMotorControlPort>(
        rightActuatorPortName, pImpl->paexoRightMotorActuator);
    pImpl->paexoRightMotorControlPort->useCallback();

    if (!pImpl->paexoRightMotorControlPort->open(rightActuatorPortName)) {
        yError() << LogPrefix << "Failed to open paexo motor control port "
                 << rightActuatorPortName;
        return false;
    }

    // Initialize paexo data buffer
    pImpl->paexoData.angle = 0.0;
    pImpl->paexoData.force = 0.0;
    pImpl->paexoData.leverarm = 0.0;
    pImpl->paexoData.updated = false;

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

    PaexoVirtualJointKinSensor(
        Paexo::PaexoImpl* impl,
        const wearable::sensor::SensorName name = {},
        const wearable::sensor::SensorStatus status =
            wearable::sensor::SensorStatus::Ok) // Default sensors status is set Ok
        : IVirtualJointKinSensor(name, status)
        , paexoImpl(impl)
    {
        // TODO: Initialization
    }

    ~PaexoVirtualJointKinSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    bool getJointPosition(double& position) const override
    {
        assert(paexoImpl != nullptr);

        std::lock_guard<std::mutex> lock(paexoImpl->mutex);
        position = paexoImpl->paexoData.angle;
        return true;
    }

    bool getJointVelocity(double& velocity) const override
    {
        assert(paexoImpl != nullptr);

        velocity = 0.0;

        return true;
    }

    bool getJointAcceleration(double& acceleration) const override
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
                       const wearable::sensor::SensorStatus status =
                           wearable::sensor::SensorStatus::Ok) // Default sensors status is set Ok
        : IForce3DSensor(name, status)
        , paexoImpl(impl)
    {
        // TODO: Initialization
    }

    ~PaexoForce3DSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    bool getForce3D(Vector3& force) const override
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
                        const wearable::sensor::SensorStatus status =
                            wearable::sensor::SensorStatus::Ok) // Default sensors status is set Ok
        : ITorque3DSensor(name, status)
        , paexoImpl(impl)
    {
        // TODO: Initialization
    }

    ~PaexoTorque3DSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    bool getTorque3D(Vector3& torque) const override
    {
        assert(paexoImpl != nullptr);

        std::lock_guard<std::mutex> lock(paexoImpl->mutex);
        torque[0] = paexoImpl->paexoData.leverarm;
        torque[1] = 0.0;
        torque[2] = 0.0;
        return true;
    }
};

// ===========================================
// Paexo implementation of ForceTorque6DSensor
// ===========================================
#ifdef ENABLE_PAEXO_USE_iFEELDriver
class Paexo::PaexoImpl::PaexoForceTorque6DSensor : public wearable::sensor::IForceTorque6DSensor
{
public:
    Paexo::PaexoImpl* paexoImpl = nullptr;

    PaexoForceTorque6DSensor(
        Paexo::PaexoImpl* impl,
        const wearable::sensor::SensorName name = {},
        const wearable::sensor::SensorStatus status =
            wearable::sensor::SensorStatus::Ok) // Default sensor status is Ok, to be updated using
                                                // node status from iFeelDriver
        : IForceTorque6DSensor(name, status)
        , paexoImpl(impl)
    {
        // TODO: Initialization
    }

    ~PaexoForceTorque6DSensor() override = default;

    void setStatus(const wearable::sensor::SensorStatus status) { m_status = status; }

    bool getForceTorque6D(Vector3& force3D, Vector3& torque3D) const override
    {
        assert(paexoImpl != nullptr);

        std::lock_guard<std::mutex> lock(paexoImpl->mutex);

        // Get FT Data from front or back FT using iFeelDriver data
        // TODO: Cleanup ftData buffer handling
        paexoImpl->ifeelData = paexoImpl->ifeelDriver->getData();

        auto it = paexoImpl->ifeelData.FTShoeNodes.begin();
        iFeel::Utils::DataTypes::NodeID nodeID = it->first;

        if (this->getSensorName().find("Left") != std::string::npos) {
            iFeel::ForceTorque ftData = paexoImpl->ifeelData.FTShoeNodes[nodeID]->getFrontFT();

            force3D[0] = ftData.x;
            force3D[1] = ftData.y;
            force3D[2] = ftData.z;

            torque3D[0] = ftData.tx;
            torque3D[1] = ftData.ty;
            torque3D[2] = ftData.tz;
        }

        if (this->getSensorName().find("Right") != std::string::npos) {
            iFeel::ForceTorque ftData = paexoImpl->ifeelData.FTShoeNodes[nodeID]->getBackFT();

            force3D[0] = ftData.x;
            force3D[1] = ftData.y;
            force3D[2] = ftData.z;

            torque3D[0] = ftData.tx;
            torque3D[1] = ftData.ty;
            torque3D[2] = ftData.tz;
        }

        return true;
    }
};

#endif

// =======================================
// Paexo implementation of Motor actutator
// =======================================
class Paexo::PaexoImpl::PaexoMotorActuator : public wearable::actuator::IMotor
{
public:
    Paexo::PaexoImpl* paexoImpl = nullptr;

    PaexoMotorActuator(
        Paexo::PaexoImpl* impl,
        const wearable::actuator::ActuatorName name = {},
        const wearable::actuator::ActuatorStatus status =
            wearable::actuator::ActuatorStatus::Ok) // Default actuator status is set ok
        : IMotor(name, status)
        , paexoImpl(impl)
    {
        // TODO: Initialization
    }

    bool setMotorPosition(double& value) const override
    {
        // Prepare the move command
        std::string motorCommand = {};

        if (this->getActuatorName().find("Left") != std::string::npos) {
            motorCommand = "move:l:" + std::to_string(value);
        }

        if (this->getActuatorName().find("Right") != std::string::npos) {
            motorCommand = "move:r:" + std::to_string(value);
        }

        motorCommand += EOL;

        char c[motorCommand.length() + 1];
        std::strcpy(c, motorCommand.c_str());

        // Set the commanded value to the serial write
        // TODO: Check for serial write failure
        paexoImpl->iSerialDevice->flush();
        paexoImpl->iSerialDevice->send(c, motorCommand.length());

        return true;
    }
};

void Paexo::PaexoImpl::PaexoMotorControlPort::onRead(yarp::os::Bottle& motorCommand)
{
    // TODO: Check if mutex is needed
    // NOTE: Assuming the associated port received a vector of one double as motor command
    assert(paexoMotorActuator != nullptr);
    // yInfo() << LogPrefix << "Data received on " << portName << motorCommand.toString().c_str();

    double cmd = motorCommand.get(0).asFloat64();
    paexoMotorActuator.get()->setMotorPosition(cmd);
}

void Paexo::run()
{
    // Send commands to BLE central serial port
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        if (pImpl->cmdPro->cmdUpdated) {

            int s = pImpl->cmdPro->cmdString.length();
            char c[s + 1];
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

        // Check if the first char is a digit, if it is the received message is broadcast
        // information
        if (isdigit(msg[0])
            && (pImpl->cmdPro->measurement_status && pImpl->cmdPro->data_broadcast)) {

            // Prepare yarp bottle with serial message and write to yarp port
            yarp::os::Bottle& bc_data = pImpl->dataPort.prepare();
            bc_data.clear();
            bc_data.fromString(msg);

            pImpl->dataPort.write();

            // Add baroadcast data to buffer
            std::lock_guard<std::mutex> lock(pImpl->mutex);
            pImpl->paexoData.angle = bc_data.get(0).asFloat64();
            pImpl->paexoData.force = bc_data.get(1).asFloat64();
            pImpl->paexoData.leverarm = bc_data.get(2).asFloat64();
            pImpl->paexoData.updated = true;
        }
        else if (!isdigit(msg[0])) {
            // yInfo() << LogPrefix << msg;
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
        yError()
            << LogPrefix
            << "Failed to view the ISerialDevice interface from the attached polydriver device";
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
    while (yarp::os::PeriodicThread::isRunning()) {
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

void Paexo::threadRelease() {}

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
        case sensor::SensorType::ForceTorque6DSensor: {
#ifdef ENABLE_PAEXO_USE_iFEELDriver
            outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(pImpl->paexoLeftArmFTSensor));
            outVec.push_back(static_cast<SensorPtr<sensor::ISensor>>(pImpl->paexoRightArmFTSensor));
#endif
            break;
        }
        default: {
            return {};
        }
    }

    return outVec;
}

wearable::ElementPtr<const wearable::actuator::IActuator>
Paexo::getActuator(const wearable::actuator::ActuatorName name) const
{
    wearable::VectorOfElementPtr<const wearable::actuator::IActuator> actuators = getAllActuators();

    for (const auto& a : actuators) {
        if (a->getActuatorName() == name) {
            return a;
        }
    }
    yWarning() << LogPrefix << "User specified actuator name <" << name << "> not found";
    return nullptr;
}

wearable::VectorOfElementPtr<const wearable::actuator::IActuator>
Paexo::getActuators(const wearable::actuator::ActuatorType aType) const
{
    wearable::VectorOfElementPtr<const wearable::actuator::IActuator> outVec;
    outVec.reserve(pImpl->nActuators);

    switch (aType) {
        case wearable::actuator::ActuatorType::Motor: {
            outVec.push_back(
                static_cast<ElementPtr<actuator::IActuator>>(pImpl->paexoLeftMotorActuator));
            outVec.push_back(
                static_cast<ElementPtr<actuator::IActuator>>(pImpl->paexoRightMotorActuator));
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

// -------------------
// FORCE TORQUE Sensor
// -------------------
#ifdef ENABLE_PAEXO_USE_iFEELDriver
wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
wearable::devices::Paexo::getForceTorque6DSensor(const sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (name == pImpl->ftSensorPrefix + "Left" + pImpl->ftSensorName
        || name == pImpl->ftSensorPrefix + "Right" + pImpl->ftSensorName) {
        yError() << LogPrefix << "Invalid sensor name " << name;
        return nullptr;
    }

    // Return a shared point to the required sensor
    // TODO: Return corresponding left and right sensors
    wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor> ftSensor;
    if (name.find("Left")) {
        ftSensor = dynamic_cast<wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>&>(
            *pImpl->paexoLeftArmFTSensor);
    }
    else if (name.find("Right")) {
        ftSensor = dynamic_cast<wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>&>(
            *pImpl->paexoRightArmFTSensor);
    }
    return ftSensor;
}
#endif

// ---------------
// MOTORO Actuator
// ---------------
wearable::ElementPtr<const actuator::IMotor>
Paexo::getMotorActuator(const actuator::ActuatorName name) const
{
    // Check if user-provided name corresponds to an available actuator
    if (name == pImpl->motorActuatorPrefix + pImpl->motorActuatorName) {
        yError() << LogPrefix << "Invalid actuator name " << name;
        return nullptr;
    }

    // Return a shared point to the required actuator
    return dynamic_cast<wearable::ElementPtr<const wearable::actuator::IMotor>&>(
        *pImpl->paexoLeftMotorActuator);
    return dynamic_cast<wearable::ElementPtr<const wearable::actuator::IMotor>&>(
        *pImpl->paexoRightMotorActuator);
}
