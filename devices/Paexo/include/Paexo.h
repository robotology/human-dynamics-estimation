/*
 * Copyright (C) 2020 iCub Facility
 * Authors: Yeshasvi Tirupachuri
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef WEARABLE_PAEXO_H
#define WEARABLE_PAEXO_H

#include "Wearable/IWear/IWear.h"

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/SerialInterfaces.h>

#include <yarp/os/PeriodicThread.h>

namespace wearable {
    namespace devices {
        class Paexo;
    } // namespace devices
} // namespace wearable

class wearable::devices::Paexo :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread,
        public yarp::dev::IWrapper,
        public yarp::dev::IMultipleWrapper
{

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
      Paexo();
      ~Paexo() override;

      // DeviceDriver
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

#endif // WEARABLE_PAEXO_H
