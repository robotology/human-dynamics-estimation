/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_WEARABLETARGETSREMAPPER
#define HDE_DEVICES_WEARABLETARGETSREMAPPER

#include <hde/interfaces/IWearableTargets.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace hde::msgs {
    class WearableTargets;
} // namespace hde::msgs
namespace hde::devices {
    class WearableTargetsRemapper;
} // namespace hde::devices

class hde::devices::WearableTargetsRemapper final
    : public yarp::dev::DeviceDriver
    , public hde::interfaces::IWearableTargets
    , public yarp::os::TypedReaderCallback<hde::msgs::WearableTargets>
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    WearableTargetsRemapper();
    ~WearableTargetsRemapper() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(hde::msgs::WearableTargets& wearableTargetsMgs) override;

    // IWearableTargets interface
    std::vector<hde::TargetName> getAllTargetsName() const override;
    std::shared_ptr<hde::WearableSensorTarget> getTarget(const TargetName name) const override;
};

#endif // HDE_DEVICES_WEARABLETARGETSREMAPPER
