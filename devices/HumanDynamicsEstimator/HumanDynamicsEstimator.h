/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_HUMANDYNAMICSESTIMATOR
#define HDE_DEVICES_HUMANDYNAMICSESTIMATOR

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>

#include "IHumanDynamics.h"

#include <memory>

namespace hde {
    namespace devices {
        class HumanDynamicsEstimator;
    } // namespace devices
} // namespace hde

class hde::devices::HumanDynamicsEstimator final
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::os::PeriodicThread
    , public hde::interfaces::IHumanDynamics
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    HumanDynamicsEstimator();
    ~HumanDynamicsEstimator() override;

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

    // IHumanDynamics
    std::vector<std::string> getJointNames() const override;
    size_t getNumberOfJoints() const override;
    std::vector<double> getJointTorques() const override;
};

#endif // HDE_DEVICES_HUMANDYNAMICSESTIMATOR
