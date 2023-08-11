// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANSTATEREMAPPER
#define HDE_DEVICES_HUMANSTATEREMAPPER

#include <hde/interfaces/IHumanState.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace hde::msgs {
    class HumanState;
} // namespace hde::msgs
namespace hde::devices {
    class HumanStateRemapper;
} // namespace hde::devices

class hde::devices::HumanStateRemapper final
    : public yarp::dev::DeviceDriver
    , public hde::interfaces::IHumanState
    , public yarp::os::TypedReaderCallback<hde::msgs::HumanState>
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanStateRemapper();
    ~HumanStateRemapper() override;

    std::mutex mtx;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(hde::msgs::HumanState& humanState) override;

    // IHumanState interface
    std::vector<std::string> getJointNames() const override;
    std::string getBaseName() const override;
    size_t getNumberOfJoints() const override;

    std::vector<double> getJointPositions() const override;
    std::vector<double> getJointVelocities() const override;

    std::array<double, 3> getBasePosition() const override;
    std::array<double, 4> getBaseOrientation() const override;

    std::array<double, 6> getBaseVelocity() const override;

    std::array<double, 3> getCoMPosition() const override;
    std::array<double, 3> getCoMVelocity() const override;
};

#endif // HDE_DEVICES_HUMANSTATEREMAPPER
