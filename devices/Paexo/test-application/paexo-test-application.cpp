/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>

#include "thrift/WearableActuatorCommand.h"

using namespace yarp::os;
//using namespace wearable;

int main() {

    BufferedPort<wearable::msg::WearableActuatorCommand> port;
    port.open("/Paexo/WearableActuatorsCommand/output:o");

    yarp::os::Network::init(); // is this needed ?
    yarp::os::Network::connect("/Paexo/WearableActuatorsCommand/output:o",
                               "/Paexo/WearableActuatorsCommand/input:i");

    while (true) {

        wearable::msg::WearableActuatorCommand& wearableActuatorCommand = port.prepare();

        // Add wearable actuator command
        wearableActuatorCommand.info.name = "Paexo::motor::Actuator";
        wearableActuatorCommand.info.type = wearable::msg::ActuatorType::MOTOR;
        wearableActuatorCommand.info.status = wearable::msg::ActuatorStatus::OK;

        wearableActuatorCommand.duration = 10;
        wearableActuatorCommand.value = 30;

        // Send the actuator command to the output port
        port.write();

        Time::delay(1);
    }

    return 0;
}
