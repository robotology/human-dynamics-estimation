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

const std::string DeviceName = "Paexo";
const std::string LogPrefix = DeviceName + ":";
double period = 0.01;

using namespace wearable;
using namespace wearable::devices;

const std::string EOL = "\n"; //EOL character
const int MAX_LINE_LENGTH = 5000; // Maximum line length to read from serial port

class Paexo::Impl
{
public:
    mutable std::mutex mutex;
    yarp::dev::ISerialDevice *iSerialDevice = nullptr;

    std::string portsPrefix;
    yarp::os::BufferedPort<yarp::os::Bottle> dataPort;

    // RPC related
    class CmdParser;
    std::unique_ptr<CmdParser> cmdPro;
    yarp::os::RpcServer rpcPort;

    std::string serialComPortName;

    // constructor
    Impl();
};

class Paexo::Impl::CmdParser : public yarp::os::PortReader
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

Paexo::Impl::Impl()
    : cmdPro(new CmdParser())
{}

// Default constructor
Paexo::Paexo()
    : PeriodicThread(period)
    , pImpl{new Impl()}
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

    //TODO: Open rpc port
    // Get port prefix name
    if (!(config.check("portsPrefixName") && config.find("portsPrefixName").isString())) {
        yInfo() << LogPrefix << "Using default port prefix /wearable/paexo";
    }
    else {
        pImpl->portsPrefix = config.find("portsPrefixName").asString();
        yInfo() << LogPrefix << "Using the ports prefix " << pImpl->portsPrefix;
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

    return true;

}

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

        // Check if the first char is a digit, if it is the received message is broadcast information
        if (isdigit(msg[0]) && (pImpl->cmdPro->measurement_status && pImpl->cmdPro->data_broadcast)) {

            // Prepare yarp bottle with serial message and write to yarp port
            yarp::os::Bottle& bc_data = pImpl->dataPort.prepare();
            bc_data.clear();
            bc_data.fromString(msg);

            pImpl->dataPort.write();

            // Add baroadcast data to Wearable interface

        }
        else if(!isdigit(msg[0])) {
            yInfo() << LogPrefix << msg;
        }
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
