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

yarp::os::Stamp XsensSuit::getLastInputStamp()
{
    return yarp::os::Stamp();
}

wearable::WearStatus XsensSuit::getStatus() const
{
    return {};
}

wearable::TimeStamp XsensSuit::getTimeStamp() const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::ISensor>
XsensSuit::getSensor(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
XsensSuit::getSensors(const wearable::sensor::SensorType) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
XsensSuit::getFreeBodyAccelerationSensor(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IMagnetometer>
XsensSuit::getMagnetometer(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
XsensSuit::getOrientationSensor(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IPoseSensor>
XsensSuit::getPoseSensor(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IPositionSensor>
XsensSuit::getPositionSensor(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
XsensSuit::getVirtualLinkKinSensor(const wearable::sensor::SensorName name) const
{
    return {};
}

wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
XsensSuit::getVirtualSphericalJointKinSensor(const wearable::sensor::SensorName name) const
{
    return {};
}
