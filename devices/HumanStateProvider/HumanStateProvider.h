/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_HUMANSTATEPROVIDER
#define HDE_DEVICES_HUMANSTATEPROVIDER

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>

#include "IHumanState.h"

#include <memory>

namespace hde {
    namespace devices {
        class HumanStateProvider;
    } // namespace devices
} // namespace hde

class hde::devices::HumanStateProvider final
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::os::PeriodicThread
    , public hde::interfaces::IHumanState
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanStateProvider();
    ~HumanStateProvider() override;

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

    // IHumanState
    std::vector<std::string> getJointNames() const override;
    size_t getNumberOfJoints() const override;
    std::string getBaseName() const override;
    std::vector<double> getJointPositions() const override;
    std::vector<double> getJointVelocities() const override;
    std::array<double, 3> getBasePosition() const override;
    std::array<double, 4> getBaseOrientation() const override;
    std::array<double, 6> getBaseVelocity() const override;
};

#endif // HDE_DEVICES_HUMANSTATEPROVIDER
