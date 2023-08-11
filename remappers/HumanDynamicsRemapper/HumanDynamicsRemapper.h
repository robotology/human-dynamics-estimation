// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANDYNAMICSREMAPPER
#define HDE_DEVICES_HUMANDYNAMICSREMAPPER

#include <hde/interfaces/IHumanDynamics.h> 

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace hde::msgs {
    class HumanDynamics;
} // namespace hde::msgs
namespace hde::devices {
    class HumanDynamicsRemapper;
} // namespace hde::devices

class hde::devices::HumanDynamicsRemapper final
    : public yarp::dev::DeviceDriver
    // inherite from the interface to be exposed
    , public hde::interfaces::IHumanDynamics
    // implement the callback to read the thrifted message
    , public yarp::os::TypedReaderCallback<hde::msgs::HumanDynamics>
    // implement the periodic thread
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanDynamicsRemapper();
    ~HumanDynamicsRemapper() override;
    


    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(hde::msgs::HumanDynamics& humanDynamics) override;

    // IHumanDynamics interface
    std::vector<std::string> getJointNames() const override;
 
    size_t getNumberOfJoints() const override;

    std::vector<double> getJointTorques() const override;
};

#endif // HDE_DEVICES_HUMANDYNAMICSREMAPPER

