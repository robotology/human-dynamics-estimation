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
#include <yarp/os/LogStream.h>

#include <thrift/WearableActuatorCommand.h>

using namespace yarp::os;
using namespace wearable;

int main() {

    yarp::os::Network::init();

    std::string inPort = "/Paexo/WearableActuatorsCommand/input:i";
    std::string outPort = "/Paexo/WearableActuatorsCommand/output:o";

    if (!yarp::os::Network::exists(inPort))
    {
        yError() << "Port " << inPort << " does not exists";
        return -1;
    }

    BufferedPort<wearable::msg::WearableActuatorCommand> port;
    if(!port.open(outPort))
    {
        yError() << "Failed to open port " << outPort;
        return -1;
    }

    if (!yarp::os::Network::connect(outPort,inPort))
    {
        yError() << "Failed to connect " << outPort << " to " << inPort;
        return -1;
    }

    while (true) {

        wearable::msg::WearableActuatorCommand& wearableActuatorCommand = port.prepare();

        // Add wearable actuator command
        wearableActuatorCommand.info.name = "Paexo::motor::Actuator";
        wearableActuatorCommand.info.type = wearable::msg::ActuatorType::MOTOR;
        wearableActuatorCommand.info.status = wearable::msg::ActuatorStatus::OK;

        wearableActuatorCommand.duration = 10;
        wearableActuatorCommand.value = 40;

        yInfo() << "Command " << wearableActuatorCommand.info.name << " to position "
                <<  wearableActuatorCommand.value << " deg";

        // Send the actuator command to the output port
        port.write();

        Time::delay(2);
    }

    return 0;
}
