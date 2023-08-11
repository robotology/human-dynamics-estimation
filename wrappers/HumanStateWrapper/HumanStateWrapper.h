// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANSTATEWRAPPER
#define HDE_DEVICES_HUMANSTATEWRAPPER

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/os/PeriodicThread.h>

#include <memory>

namespace hde {
    namespace wrappers {
        class HumanStateWrapper;
    } // namespace wrappers
} // namespace hde

class hde::wrappers::HumanStateWrapper final
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanStateWrapper();
    ~HumanStateWrapper() override;

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
};

#endif // HDE_DEVICES_HUMANSTATEWRAPPER
