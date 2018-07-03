/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENSSUITWRAPPER_H
#define XSENSSUITWRAPPER_H

#include <memory>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/dev/Wrapper.h>

namespace iwear {
    namespace wrappers {
        class XsensSuitWrapper;
    }
} // namespace iwear

class iwear::wrappers::XsensSuitWrapper
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::dev::IPreciselyTimed
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    XsensSuitWrapper();
    ~XsensSuitWrapper() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;

    // IPreciselyTimed interface
    yarp::os::Stamp getLastInputStamp() override;
};

#endif
