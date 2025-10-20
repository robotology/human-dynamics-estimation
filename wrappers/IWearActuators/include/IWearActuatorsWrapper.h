// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IWEACTUATORSWRAPPERS_H
#define IWEACTUATORSWRAPPERS_H

#include <memory>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include "trintrin/msgs/WearableActuatorCommand.h"
#include "trintrin/msgs/GloveActuatorCommand.h"

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
    , public yarp::os::TypedReaderCallback<trintrin::msgs::WearableActuatorCommand>
    , public yarp::os::TypedReaderCallback<trintrin::msgs::GloveActuatorCommand>
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
    void onRead(trintrin::msgs::WearableActuatorCommand& wearableActuatorCommand) override;
    void onRead(trintrin::msgs::GloveActuatorCommand& gloveActuatorCommand) override;

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
