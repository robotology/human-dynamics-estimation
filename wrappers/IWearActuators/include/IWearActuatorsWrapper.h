/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef IWEACTUATORSWRAPPERS_H
#define IWEACTUATORSWRAPPERS_H

#include <memory>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include "thrift/WearableActuatorCommand.h"

namespace wearable {
    namespace wrappers {
        class IWearActuatorsWrapper;
    }
} // namespace wearable

class wearable::wrappers::IWearActuatorsWrapper
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::os::PeriodicThread
    , public yarp::os::TypedReaderCallback<msg::WearableActuatorCommand>
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    IWearActuatorsWrapper();
    ~IWearActuatorsWrapper() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // TypedReaderCallback
    void onRead(msg::WearableActuatorCommand& wearableActuatorCommand) override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;
};

#endif // IWEACTUATORSWRAPPERS_H
