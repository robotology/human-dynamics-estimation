/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "IWearActuatorsWrapper.h"

#include "Wearable/IWear/IWear.h"

#include "thrift/WearableActuatorCommand.h"

#include <yarp/os/LogStream.h>

using namespace wearable;
using namespace wearable::wrappers;

const std::string WrapperName = "IWearActuatorsWrapper";
const std::string LogPrefix = WrapperName + wearable::Separator;
constexpr double DefaultPeriod = 0.01;

class IWearActuatorsWrapper::impl : public wearable::msg::WearableActuatorCommand
{
public:
    std::string actuatorCommandInputPortName;
    //TODO: Add yarp port to read actuator command

    msg::WearableActuatorCommand wearableActuatorCommand;

    wearable::IWear* iWear = nullptr;
};

IWearActuatorsWrapper::IWearActuatorsWrapper()
    : yarp::os::PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

IWearActuatorsWrapper::~IWearActuatorsWrapper()
{
    detachAll();
    close();
}

// ======================
// DeviceDriver interface
// ======================

bool IWearActuatorsWrapper::open(yarp::os::Searchable& config)
{
    if (!config.check("actuatorCommandInputPortName") || !config.find("actuatorCommandInputPortName").isString()) {
        yError() << LogPrefix << "actuatorCommandInputPortName parameter not found";
        return false;
    }


    if (!config.check("period")) {
        yInfo() << LogPrefix << "Using default period: " << DefaultPeriod << "s";
    }

    // Parse configuration parameters

    pImpl->actuatorCommandInputPortName = config.find("actuatorCommandInputPortName").asString();

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    setPeriod(period);

    //TODO: Open yarp port to read actuator command

    return true;
}

bool IWearActuatorsWrapper::close()
{
    //TODO: Close the yarp port
    return true;
}

// ==================
// IWrapper interface
// ==================

bool IWearActuatorsWrapper::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr.";
        return false;
    }

    if (pImpl->iWear || !poly->view(pImpl->iWear) || !pImpl->iWear) {
        yError() << LogPrefix << "Failed to view the IWear interface from the PolyDriver.";
        return false;
    }

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the periodic thread.";
        return false;
    }

    yDebug() << LogPrefix << "attach() successful";
    return true;
}

void IWearActuatorsWrapper::threadRelease()
{

}

bool IWearActuatorsWrapper::detach()
{
    while (isRunning()) {
        stop();
    }

    pImpl->iWear = nullptr;
    pImpl->wearableActuatorCommand = {};

    return true;
}

// ==========================
// IMultipleWrapper interface
// ==========================

bool IWearActuatorsWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver.";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr.";
        return false;
    }

    return attach(driver->poly);
}

bool IWearActuatorsWrapper::detachAll()
{
    return detach();
}
