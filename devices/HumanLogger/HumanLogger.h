/*
* Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HUMANLOGGER_H
#define HUMANLOGGER_H

#include <memory>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/os/PeriodicThread.h>

namespace hde {
    namespace devices {
        class HumanLogger;
    } // namespace devices
} // namespace hde

class hde::devices::HumanLogger
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanLogger();
    ~HumanLogger() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;
};

#endif // HUMANLOGGER_H
