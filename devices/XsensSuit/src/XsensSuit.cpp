/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "XsensSuit.h"
#include <yarp/os/LogStream.h>

#include <mutex>

using namespace wearable::devices;

const std::string logPrefix = "XsensSuit : ";

class XsensSuit::impl //: public xsens::IXsensMNVControlService
{
public:
    class IXsensMVNControl;
    IXsensMVNControl* xsensControl;
    std::mutex mutex;
};

XsensSuit::XsensSuit()
    : pImpl{new impl()}
{}

XsensSuit::~XsensSuit() = default;

// ======================
// DeviceDriver interface
// ======================

bool XsensSuit::open(yarp::os::Searchable& /*config*/)
{
    return true;
}

bool XsensSuit::close()
{
    return true;
}

yarp::os::Stamp XsensSuit::getLastInputStamp() {}

wearable::WearStatus XsensSuit::getStatus() const {}

wearable::TimeStamp XsensSuit::getTimeStamp() const {}
