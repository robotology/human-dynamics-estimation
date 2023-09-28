// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANWRENCHREMAPPER
#define HDE_DEVICES_HUMANWRENCHREMAPPER

#include <hde/interfaces/IHumanWrench.h> 

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace hde::msgs {
    class HumanWrench;
} // namespace hde::msgs
namespace hde::devices {
    class HumanWrenchRemapper;
} // namespace hde::devices

class hde::devices::HumanWrenchRemapper final
    : public yarp::dev::DeviceDriver
    // inherite from the interface to be exposed
    , public hde::interfaces::IHumanWrench
    // implement the callback to read the thrifted message
    , public yarp::os::TypedReaderCallback<hde::msgs::HumanWrench>
    // implement the periodic thread
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanWrenchRemapper();
    ~HumanWrenchRemapper() override;
    


    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(hde::msgs::HumanWrench& humanWrench) override;

    // IHumanWrench interface
    std::vector<std::string> getWrenchSourceNames() const override;

    size_t getNumberOfWrenchSources() const override;

    std::vector<double> getWrenches() const override;
};

#endif // HDE_DEVICES_HUMANWRENCHREMAPPER

