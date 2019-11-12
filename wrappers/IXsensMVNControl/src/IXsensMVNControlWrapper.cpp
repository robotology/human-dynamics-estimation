/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "IXsensMVNControlWrapper.h"
#include "IXsensMVNControl.h"

#include "XsensSuitControlService.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>

const std::string WrapperName = "IXsensMVNControlWrapper";
const std::string logPrefix = WrapperName + " : ";

// using namespace xsensmvn;
using namespace wearable::wrappers;

class IXsensMVNControlWrapper::impl : public wearable::msg::XsensSuitControlService
{
public:
    ~impl() override = default;

    yarp::os::Port rpcPort;
    xsensmvn::IXsensMVNControl* xsControl = nullptr;

    bool calibrate() override;
    bool calibrateWithType(const std::string& calibrationType) override;
    bool abortCalibration() override;
    bool startAcquisition() override;
    bool stopAcquisition() override;
    std::vector<std::string> help(const std::string& functionName = "--all") override
    {
        return {}; // TODO}
    }
};

wearable::wrappers::IXsensMVNControlWrapper::IXsensMVNControlWrapper()
    : PeriodicThread(0.5)
    , pImpl{new impl()}
{}

wearable::wrappers::IXsensMVNControlWrapper::~IXsensMVNControlWrapper() = default;

bool IXsensMVNControlWrapper::open(yarp::os::Searchable& config)
{
    std::string defaultRpcPortName = "/" + WrapperName + "/rpc:i";

    if (!config.check("rpcPortName") || !config.find("rpcPortName").isString()) {
        yInfo() << logPrefix << "rpcPortName parameter not found, using default defaultRpcPortName";
    }
    else {
        defaultRpcPortName = config.find("rpcPortName").asString();
    }

    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << logPrefix << "YARP server wasn't found active.";
        return false;
    }

    if (!pImpl->rpcPort.open(defaultRpcPortName)) {
        yError() << "Failed to open local port " << defaultRpcPortName;
        return false;
    }

    if (!pImpl->yarp().attachAsServer(pImpl->rpcPort)) {
        yError() << "Failed to attach " << defaultRpcPortName << " to the RPC service";
        return false;
    }

    start();
    return true;
}

bool IXsensMVNControlWrapper::close()
{
    pImpl->rpcPort.close();
    return true;
}

bool IXsensMVNControlWrapper::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << logPrefix << "Passed PolyDriver is nullptr.";
        return false;
    }

    if (pImpl->xsControl || !poly->view(pImpl->xsControl) || !pImpl->xsControl) {
        yError() << logPrefix
                 << "Failed to view the IXsensMVNControl interface from the PolyDriver.";
        return false;
    }

    return true;
}

bool IXsensMVNControlWrapper::detach()
{
    pImpl->xsControl = nullptr;
    return true;
}

void IXsensMVNControlWrapper::run()
{
    return;
}

bool IXsensMVNControlWrapper::impl::calibrate()
{
    if (!xsControl) {
        return false;
    }

    return xsControl->calibrate();
}

bool IXsensMVNControlWrapper::impl::calibrateWithType(const std::string& calibrationType)
{
    if (!xsControl) {
        return false;
    }

    return xsControl->calibrate(calibrationType);
}

bool IXsensMVNControlWrapper::impl::abortCalibration()
{
    if (!xsControl) {
        return false;
    }

    return xsControl->abortCalibration();
}

bool IXsensMVNControlWrapper::impl::startAcquisition()
{
    if (!xsControl) {
        return false;
    }

    return xsControl->startAcquisition();
}

bool IXsensMVNControlWrapper::impl::stopAcquisition()
{
    if (!xsControl) {
        return false;
    }

    return xsControl->stopAcquisition();
}

// ==========================
// IMultipleWrapper interface
// ==========================

bool IXsensMVNControlWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << logPrefix << "This wrapper accepts only one attached PolyDriver.";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << logPrefix << "Passed PolyDriverDescriptor is nullptr.";
        return false;
    }

    return attach(driver->poly);
}

bool IXsensMVNControlWrapper::detachAll()
{
    return detach();
}
