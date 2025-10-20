// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_WEARABLETARGETS_NWC_YARP
#define HDE_DEVICES_WEARABLETARGETS_NWC_YARP

#include <hde/interfaces/IWearableTargets.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace trintrin::msgs {
    class WearableTargets;
} // namespace trintrin::msgs
namespace hde::devices {
    class WearableTargets_nwc_yarp;
} // namespace hde::devices

class hde::devices::WearableTargets_nwc_yarp final
    : public yarp::dev::DeviceDriver
    , public hde::interfaces::IWearableTargets
    , public yarp::os::TypedReaderCallback<trintrin::msgs::WearableTargets>
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    WearableTargets_nwc_yarp();
    ~WearableTargets_nwc_yarp() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(trintrin::msgs::WearableTargets& wearableTargetsMgs) override;

    // IWearableTargets interface
    std::vector<hde::TargetName> getAllTargetsName() const override;
    std::shared_ptr<hde::WearableSensorTarget> getTarget(const TargetName name) const override;
};

#endif // HDE_DEVICES_WEARABLETARGETS_NWC_YARP
