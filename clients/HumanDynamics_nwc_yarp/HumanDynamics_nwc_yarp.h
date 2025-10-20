// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANDYNAMICS_NEC_YARP
#define HDE_DEVICES_HUMANDYNAMICS_NEC_YARP

#include <hde/interfaces/IHumanDynamics.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace trintrin::msgs {
    class HumanDynamics;
} // namespace trintrin::msgs
namespace hde::devices {
    class HumanDynamics_nwc_yarp;
} // namespace hde::devices

class hde::devices::HumanDynamics_nwc_yarp final
    : public yarp::dev::DeviceDriver
    // inherite from the interface to be exposed
    , public hde::interfaces::IHumanDynamics
    // implement the callback to read the thrifted message
    , public yarp::os::TypedReaderCallback<trintrin::msgs::HumanDynamics>
    // implement the periodic thread
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanDynamics_nwc_yarp();
    ~HumanDynamics_nwc_yarp() override;



    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(trintrin::msgs::HumanDynamics& humanDynamics) override;

    // IHumanDynamics interface
    std::vector<std::string> getJointNames() const override;

    size_t getNumberOfJoints() const override;

    std::vector<double> getJointTorques() const override;
};

#endif // HDE_DEVICES_HUMANDYNAMICS_NEC_YARP

