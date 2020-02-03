/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanDataCollector.h"
#include "IHumanState.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

const std::string DeviceName = "HumanDataCollector";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

class HumanDataCollector::impl
{
public:
    hde::interfaces::IHumanState* iHumanState = nullptr;

    //TODO: Decide the names and the number of ports needed for IHumanState interface data
};

HumanDataCollector::HumanDataCollector()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanDataCollector::~HumanDataCollector() {}

bool HumanDataCollector::open(yarp::os::Searchable &config) {

    //TODO

    return true;
}

bool HumanDataCollector::close() {
    return true;
}

void HumanDataCollector::threadRelease() {}

void HumanDataCollector::run()
{

}

bool HumanDataCollector::attach(yarp::dev::PolyDriver *poly)
{
    //TODO

    return true;
}

bool HumanDataCollector::detach()
{
    while (isRunning()) {
        stop();
    }

    //TODO: Close all the yarp ports

    pImpl->iHumanState = nullptr;

    return false;
}

bool HumanDataCollector::attachAll(const yarp::dev::PolyDriverList &driverList)
{
    //TODO: Check how many devices this will attach to

    return true;
}

bool HumanDataCollector::detachAll()
{
    return detach();
}
