/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_HUMANWRENCHPROVIDER
#define HDE_DEVICES_HUMANWRENCHPROVIDER

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>

#include <hde/interfaces/IHumanWrench.h>

#include <memory>

namespace hde {
    namespace devices {
        class HumanWrenchProvider;
    } // namespace devices
} // namespace hde

class hde::devices::HumanWrenchProvider final
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::os::PeriodicThread
    , public yarp::dev::IAnalogSensor
    , public hde::interfaces::IHumanWrench
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    HumanWrenchProvider();
    ~HumanWrenchProvider() override;

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

    // IAnalogSensor interface
    int read(yarp::sig::Vector& out) override;
    int getState(int ch) override;
    int getChannels() override;
    int calibrateSensor() override;
    int calibrateSensor(const yarp::sig::Vector& value) override;
    int calibrateChannel(int ch) override;
    int calibrateChannel(int ch, double value) override;

    // IHumanWrench
    std::vector<std::string> getWrenchSourceNames() const override;
    size_t getNumberOfWrenchSources() const override;
    std::vector<double> getWrenches() const override;
};

#endif // HDE_DEVICES_HUMANWRENCHPROVIDER
