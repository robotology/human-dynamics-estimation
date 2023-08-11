// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANSTATEPROVIDER
#define HDE_DEVICES_HUMANSTATEPROVIDER

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IWrapper.h>
#include <hde/interfaces/IHumanState.h>
#include <hde/interfaces/IWearableTargets.h>

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
    , public hde::interfaces::IWearableTargets
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
    std::array<double, 3> getCoMPosition() const override;
    std::array<double, 3> getCoMVelocity() const override;

    // IWearableTargets
    std::vector<hde::TargetName> getAllTargetsName() const override;
    std::shared_ptr<hde::WearableSensorTarget> getTarget(const hde::TargetName name) const override;
};

#endif // HDE_DEVICES_HUMANSTATEPROVIDER
