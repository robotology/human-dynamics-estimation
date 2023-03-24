// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HUMANLOGGER_H
#define HUMANLOGGER_H

#include <memory>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/PeriodicThread.h>

namespace hde {
    namespace devices {
        class HumanLogger;
    } // namespace devices
} // namespace hde

class hde::devices::HumanLogger
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IMultipleWrapper
    , private yarp::os::PeriodicThread
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

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;
};

#endif // HUMANLOGGER_H
