/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "XsensSuitWrapper.h"
#include <yarp/os/LogStream.h>

#include <mutex>

using namespace iwear::wrappers;

const std::string logPrefix = "XsensSuitWrapper : ";

class XsensSuitWrapper::impl : public xsens::IXsensMNVControlService
{
public:
    class IXsensMVNControl;
    IXsensMVNControl* xsensControl;
    std::mutex mutex;
};

XsensSuitWrapper::XsensSuitWrapper()
    : pImpl{new impl()}
{}

XsensSuitWrapper::~XsensSuitWrapper() = default;

bool XsensSuitWrapper::open(yarp::os::Searchable& /*config*/)
{
    return true;
}

bool XsensSuitWrapper::close()
{
    return true;
}

bool XsensSuitWrapper::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << logPrefix << " Passed PolyDriver is nullptr.";
        return false;
    }

    if (pImpl->xsensControl || !poly->view(pImpl->xsensControl) || !pImpl->xsensControl) {
        yError() << logPrefix << "Failed to attach the XsensDriver to the XsensWrapper.";
        return false;
    }

    // TODO: return true if the driver is up and running
    return true;
}

bool XsensSuitWrapper::detach()
{
    //    std::lock_guard<std::mutex>(pImpl->mutex);
    pImpl->xsensControl = nullptr;
    return true;
}

bool XsensSuitWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool XsensSuitWrapper::detachAll()
{
    return detach();
}
