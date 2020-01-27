/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_ROBOTPOSITIONCONTROLLER
#define HDE_DEVICES_ROBOTPOSITIONCONTROLLER

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/Wrapper.h>

#include <memory>

namespace hde {
    namespace devices {
        class RobotPositionController;
    } // namespace devices
} // namespace hde

class hde::devices::RobotPositionController final
        : public yarp::dev::DeviceDriver
        , public yarp::dev::IWrapper
        , public yarp::dev::IMultipleWrapper
        , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    RobotPositionController();
    ~RobotPositionController();

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

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

#endif // HDE_DEVICES_ROBOTPOSITIONCONTROLLER
