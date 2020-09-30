/*
 * Copyright (C) 2020 iCub Facility
 * Authors: Yeshasvi Tirupachuri
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "Paexo.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <mutex>
#include <string>
#include <stdio.h>
#include <vector>

const std::string DeviceName = "Paexo";
const std::string LogPrefix = DeviceName + ":";
double period = 0.01;

using namespace wearable;
using namespace wearable::devices;


class Paexo::Impl
{
public:
    mutable std::mutex mutex;
    yarp::dev::ISerialDevice *iSerialDevice = nullptr;

    std::string serialComPortName;
};


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

    return true;

}

void Paexo::run()
{
    char charMsg[5000];
    int size = pImpl->iSerialDevice->receiveLine(charMsg, 5000);

    //TODO: Check how the data needs to be handled
    yInfo() << "Received Message : " << charMsg;
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
