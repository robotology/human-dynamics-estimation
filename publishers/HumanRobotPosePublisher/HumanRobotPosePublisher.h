// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANROBOTPOSEPUBLISHER
#define HDE_DEVICES_HUMANROBOTPOSEPUBLISHER

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <memory>

namespace hde {
    namespace publishers {
        class HumanRobotPosePublisher;
    } // namespace publishers
} // namespace hde

class hde::publishers::HumanRobotPosePublisher final
    : public yarp::dev::DeviceDriver
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanRobotPosePublisher();
    ~HumanRobotPosePublisher() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
};

#endif // HDE_DEVICES_HUMANROBOTPOSEPUBLISHER

