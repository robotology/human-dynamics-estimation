// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_ROBOTPOSITIONCONTROLLER
#define HDE_DEVICES_ROBOTPOSITIONCONTROLLER

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IWrapper.h>

#include <memory>

namespace hde {
    namespace devices {
        class RobotPositionController;
    } // namespace devices
} // namespace hde

class hde::devices::RobotPositionController final
        : public yarp::dev::DeviceDriver
        , public yarp::dev::IWrapper
        , private yarp::os::PeriodicThread
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

};

#endif // HDE_DEVICES_ROBOTPOSITIONCONTROLLER
