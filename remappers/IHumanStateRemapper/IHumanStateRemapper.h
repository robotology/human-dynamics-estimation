/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_IHUMANSTATEREMAPPER
#define HDE_DEVICES_IHUMANSTATEREMAPPER

#include "IHumanState.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

namespace human {
    class HumanState;
}
namespace hde {
    namespace devices {
        class IHumanStateRemapper;
    } // namespace devices
} // namespace hde

class hde::devices::IHumanStateRemapper final
    : public yarp::dev::DeviceDriver
    , public hde::interfaces::IHumanState
    , public yarp::os::TypedReaderCallback<human::HumanState>
    , public yarp::os::PeriodicThread
    //, public yarp::dev::IPreciselyTimed
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    IHumanStateRemapper();
    ~IHumanStateRemapper() override;

	// DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(human::HumanState& humanState) override;

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

#endif // HDE_DEVICES_IHUMANSTATEREMAPPER
