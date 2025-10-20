// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "IWearActuatorsWrapper.h"

#include "Wearable/IWear/IWear.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>

#include <unordered_map>

using namespace wearable;
using namespace wearable::wrappers;

const std::string WrapperName = "IWearActuatorsWrapper";
const std::string LogPrefix = WrapperName + wearable::Separator;
constexpr double DefaultPeriod = 0.01;

class IWearActuatorsWrapper::impl : public trintrin::msgs::WearableActuatorCommand
{
public:
    std::string attachedWearableDeviceKey = "defaultIWearActuatorsWrapperDevice";

    std::string actuatorCommandInputPortName;
    std::string gloveActuatorCommandInputPortName;
    yarp::os::BufferedPort<trintrin::msgs::WearableActuatorCommand> actuatorCommandInputPort;
    yarp::os::BufferedPort<trintrin::msgs::GloveActuatorCommand> gloveActuatorCommandInputPort;

    std::unordered_map<std::string, wearable::ElementPtr<const actuator::IActuator>> actuatorsMap;

    trintrin::msgs::WearableActuatorCommand wearableActuatorCommand;
    trintrin::msgs::GloveActuatorCommand gloveActuatorCommand;

    wearable::IWear* iWear = nullptr;
};

IWearActuatorsWrapper::IWearActuatorsWrapper()
    : yarp::os::PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

IWearActuatorsWrapper::~IWearActuatorsWrapper()
{
    detachAll();
    close();
}

void IWearActuatorsWrapper::run()
{
    //TODO: Send the values of the actuators state ?
}

// ======================
// DeviceDriver interface
// ======================

bool IWearActuatorsWrapper::open(yarp::os::Searchable& config)
{
    if (!config.check("actuatorCommandInputPortName") && !config.check("gloveActuatorCommandInputPortName"))
    {
        yError() << LogPrefix << "No actuator command input ports found.";
        return false;
    }

    // Find and configure yarp port related to the actuator commands
    if (!config.find("actuatorCommandInputPortName").isString()) {
        yWarning() << LogPrefix << "actuatorCommandInputPortName parameter not found. Haptic feedback will not be available.";
    }
    else
    {
        pImpl->actuatorCommandInputPortName = config.find("actuatorCommandInputPortName").asString();
        if(!pImpl->actuatorCommandInputPort.open(pImpl->actuatorCommandInputPortName))
        {
            yError() << "Failed to open " << pImpl->actuatorCommandInputPortName << " yarp port";
            return false;
        }

        // Set the callback to use onRead() method of this device
        pImpl->actuatorCommandInputPort.useCallback(*this);

    }
    if (!config.check("gloveActuatorCommandInputPortName") || !config.find("gloveActuatorCommandInputPortName").isString()) {
        yWarning() << LogPrefix << "gloveActuatorCommandInputPortName parameter not found! The port will not be opened.";
    }
    else
    {
        pImpl->gloveActuatorCommandInputPortName = config.find("gloveActuatorCommandInputPortName").asString();
        if(!pImpl->gloveActuatorCommandInputPort.open(pImpl->gloveActuatorCommandInputPortName))
        {
            yError() << "Failed to open " << pImpl->gloveActuatorCommandInputPortName << " yarp port";
            return false;
        }
        // Set the callback to use onRead() method of this device
        pImpl->gloveActuatorCommandInputPort.useCallback(*this);
    }

    if (!config.check("period")) {
        yInfo() << LogPrefix << "Using default period: " << DefaultPeriod << "s";
    }

    // Parse configuration parameters

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    setPeriod(period);

    return true;
}

void IWearActuatorsWrapper::onRead(trintrin::msgs::WearableActuatorCommand& wearableActuatorCommand)
{
   // Unpack the actuator in from incoming command
    trintrin::msgs::ActuatorInfo info = wearableActuatorCommand.info;

   // Check if the commanded actuator name is available
   if (pImpl->actuatorsMap.find(info.name) == pImpl->actuatorsMap.end())
   {
       yWarning() << "Requested actuator with name " << info.name << " is not available in " << pImpl->attachedWearableDeviceKey << " wearable device \n \t Ignoring wearable actuation command.";
   }
   else // process the wearable actuator command
   {
       wearable::actuator::ActuatorType aType = pImpl->actuatorsMap[info.name]->getActuatorType();

       switch (aType) {
            case wearable::actuator::ActuatorType::Haptic: {

               // Check if the actuator type in the wearable command is correct
                if (info.type == trintrin::msgs::ActuatorType::HAPTIC)
               {
                   // Get haptic actuator
                   wearable::ElementPtr<const wearable::actuator::IHaptic> castActuator = std::static_pointer_cast<const wearable::actuator::IHaptic>(pImpl->actuatorsMap[info.name]);

                   // Send haptic command
                   castActuator->setHapticCommand(wearableActuatorCommand.value);
               }

               break;
            }
            case wearable::actuator::ActuatorType::Motor: {

               // Check if the actuator type in the wearable command is correct
                if (info.type == trintrin::msgs::ActuatorType::MOTOR)
               {
                   // Get motor actuator
                   wearable::ElementPtr<const wearable::actuator::IMotor> castActuator = std::static_pointer_cast<const wearable::actuator::IMotor>(pImpl->actuatorsMap[info.name]);

                   // Send motor command
                   castActuator->setMotorPosition(wearableActuatorCommand.value);
               }

               break;
            }
            default: {
               return;
            }
       }
   }
}

void IWearActuatorsWrapper::onRead(trintrin::msgs::GloveActuatorCommand& gloveActuatorCommand)
{
   // Unpack the actuator in from incoming command
    trintrin::msgs::ActuatorInfo info = gloveActuatorCommand.info;

   // Check if the commanded actuator name is available
   if (pImpl->actuatorsMap.find(info.name) == pImpl->actuatorsMap.end())
   {
       yWarning() << "Requested actuator with name " << info.name << " is not available in " << pImpl->attachedWearableDeviceKey << " wearable device \n \t Ignoring wearable actuation command.";
   }
   else // process the wearable actuator command
   {
        wearable::actuator::ActuatorType aType = pImpl->actuatorsMap[info.name]->getActuatorType();
        if (aType == wearable::actuator::ActuatorType::Haptic)
        {
            // Check if the actuator type in the wearable command is correct
            if (info.type == trintrin::msgs::ActuatorType::HAPTIC)
            {
                // Get haptic actuator
                wearable::ElementPtr<const wearable::actuator::IHaptic> castActuator = std::static_pointer_cast<const wearable::actuator::IHaptic>(pImpl->actuatorsMap[info.name]);

                // Send haptic command
                castActuator->setHapticCommands(gloveActuatorCommand.forceValue, gloveActuatorCommand.vibroTactileValue);
            }
        }
        else
        {
            yError() << "Actuator type is not correct!";
            return;
        }
   }
}

bool IWearActuatorsWrapper::close()
{
    pImpl->actuatorCommandInputPort.close();
    return true;
}

// ==================
// IWrapper interface
// ==================

bool IWearActuatorsWrapper::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr.";
        return false;
    }

    if (pImpl->iWear || !poly->view(pImpl->iWear) || !pImpl->iWear) {
        yError() << LogPrefix << "Failed to view the IWear interface from the PolyDriver.";
        return false;
    }

    // Check and add all the available actuators

    yInfo() << LogPrefix << "Finding available actuators from " << pImpl->attachedWearableDeviceKey << " wearable deive ...";

    for (const auto& a : pImpl->iWear->getHapticActuators())
    {
        pImpl->actuatorsMap[a->getActuatorName()] = a;
        yInfo() << LogPrefix << "Adding actuator" << a->getActuatorName();
    }

    for (const auto& a : pImpl->iWear->getMotorActuators())
    {
        pImpl->actuatorsMap[a->getActuatorName()] = a;
        yInfo() << LogPrefix << "Adding actuator" << a->getActuatorName();
    }

    for (const auto& a : pImpl->iWear->getHeaterActuators())
    {
        pImpl->actuatorsMap[a->getActuatorName()] = a;
        yInfo() << LogPrefix << "Adding actuator" << a->getActuatorName();
    }

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the periodic thread.";
        return false;
    }

    yDebug() << LogPrefix << "attach() successful";
    return true;
}

void IWearActuatorsWrapper::threadRelease()
{

}

bool IWearActuatorsWrapper::detach()
{
    while (isRunning()) {
        stop();
    }

    pImpl->iWear = nullptr;
    pImpl->wearableActuatorCommand = {};

    return true;
}

// ==========================
// IMultipleWrapper interface
// ==========================

bool IWearActuatorsWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver.";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr.";
        return false;
    }

    // Save key that identifies the driver
    pImpl->attachedWearableDeviceKey = driver->key;

    return attach(driver->poly);
}

bool IWearActuatorsWrapper::detachAll()
{
    return detach();
}
