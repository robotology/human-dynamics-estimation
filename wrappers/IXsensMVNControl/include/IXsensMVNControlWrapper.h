// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IXSENS_MVN_CONTROL_WRAPPER_H
#define IXSENS_MVN_CONTROL_WRAPPER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/PeriodicThread.h>

#include <memory>

//#include "IXsensMVNControl.h"

namespace wearable {
    namespace wrappers {
        class IXsensMVNControlWrapper;
    }
} // namespace wearable

// using namespace xsensmvn;

class wearable::wrappers::IXsensMVNControlWrapper
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    IXsensMVNControlWrapper();
    ~IXsensMVNControlWrapper() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // PeriodicThread
    void run() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;
};

#endif // IXSENS_MVN_CONTROL_WRAPPER_H
