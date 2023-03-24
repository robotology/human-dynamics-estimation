// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANDYNAMICSESTIMATOR
#define HDE_DEVICES_HUMANDYNAMICSESTIMATOR

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/PeriodicThread.h>

#include <hde/interfaces/IHumanDynamics.h>

#include <memory>

namespace hde {
    namespace devices {
        class HumanDynamicsEstimator;
    } // namespace devices
} // namespace hde

class hde::devices::HumanDynamicsEstimator final
    : public yarp::dev::DeviceDriver
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

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;

    // IHumanDynamics
    std::vector<std::string> getJointNames() const override;
    size_t getNumberOfJoints() const override;
    std::vector<double> getJointTorques() const override;
};

#endif // HDE_DEVICES_HUMANDYNAMICSESTIMATOR
