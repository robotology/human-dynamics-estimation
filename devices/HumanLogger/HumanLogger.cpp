/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanLogger.h"
#include <hde/interfaces/IHumanState.h>
#include <hde/interfaces/IHumanDynamics.h>

#include <algorithm>
#include <functional>
#include <mutex>
#include <vector>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

#include <robometry/BufferManager.h>

const std::string LoggerName = "HumanLogger";
const std::string logPrefix = LoggerName + " :";
constexpr double DefaultPeriod = 0.01;

namespace hde {
    namespace devices {
        struct HumanLoggerSettings;

        enum class LoggerType
        {
            NONE = 0,
            MATLAB = 1
            // TODO: add YARP logging option
        };
    } // namespace devices
} // namespace hde

struct hde::devices::HumanLoggerSettings
{
    bool saveBufferManagerConfiguration{false};
    bool logHumanState{false};
    bool logHumanDynamics{false};
};

using namespace hde::devices;

class HumanLogger::impl
{
public:
    LoggerType loggerType;
    bool setLoggerType(const std::string& str);

    bool loadSettingsFromConfig(yarp::os::Searchable& config);
    void checkAndLoadBooleanOption(yarp::os::Searchable& config,
                                   const std::string& optionName,
                                   bool& option);
    bool configureBufferManager();

    size_t waitingFirstReadCounter = 1;

    interfaces::IHumanState* iHumanState = nullptr;
    interfaces::IHumanDynamics* iHumanDynamics = nullptr;
    HumanLoggerSettings settings;
    robometry::BufferConfig bufferConfig;
    robometry::BufferManager bufferManager;

    // buffer variables
    // iHumanState
    std::array<double, 3> basePositionInterface;
    std::array<double, 4> baseOrientationInterface;
    std::array<double, 6> baseVelocityInterface;
    std::vector<double> jointPositionsInterface;
    std::vector<double> jointVelocitiesInterface;
    std::vector<std::string> jointNamesStateInterface;
    // iHumanDynamics
    std::vector<double> jointTorquesInterface;
    std::vector<std::string> jointNamesDynamicsInterface;

};

HumanLogger::HumanLogger()
    : yarp::os::PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanLogger::~HumanLogger()
{
    detachAll();
    close();
}

// ========================
// PeriodicThread interface
// ========================

void HumanLogger::run()
{
    auto timeNow = yarp::os::Time::now();

    if (pImpl->settings.logHumanState)
    {
        if(!pImpl->iHumanState) {
            yError() << logPrefix << "The IHumanState pointer is null in the driver loop.";
            askToStop();
            return;
        }

        pImpl->basePositionInterface = pImpl->iHumanState->getBasePosition();
        pImpl->baseOrientationInterface = pImpl->iHumanState->getBaseOrientation();
        pImpl->baseVelocityInterface = pImpl->iHumanState->getBaseVelocity();
        pImpl->jointPositionsInterface = pImpl->iHumanState->getJointPositions();
        pImpl->jointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();

        if (pImpl->loggerType == LoggerType::MATLAB) {
            pImpl->bufferManager.push_back(pImpl->basePositionInterface,
                                           timeNow,
                                           "human_state::base_position");
            pImpl->bufferManager.push_back(pImpl->baseOrientationInterface,
                                           timeNow,
                                           "human_state::base_orientation");
            pImpl->bufferManager.push_back(pImpl->baseVelocityInterface,
                                           timeNow,
                                           "human_state::base_velocity");
            pImpl->bufferManager.push_back(pImpl->jointPositionsInterface,
                                           timeNow,
                                           "human_state::joint_positions");
            pImpl->bufferManager.push_back(pImpl->jointVelocitiesInterface,
                                           timeNow,
                                           "human_state::joint_velocities");
        }
    }

    if (pImpl->settings.logHumanDynamics)
    {
        if(!pImpl->iHumanDynamics) {
            yError() << logPrefix << "The IHumanDynamics pointer is null in the driver loop.";
            askToStop();
            return;
        }

        pImpl->jointTorquesInterface = pImpl->iHumanDynamics->getJointTorques();

        if (pImpl->loggerType == LoggerType::MATLAB) {
            pImpl->bufferManager.push_back(pImpl->jointTorquesInterface,
                                           timeNow,
                                           "human_dynamics::joint_torques");
        }
    }
}

// ======================
// DeviceDriver interface
// ======================

bool HumanLogger::open(yarp::os::Searchable& config)
{
    if (!config.check("period")) {
        yInfo() << logPrefix << "Using default period: " << DefaultPeriod << "s";
    }

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    setPeriod(period);

    // Load settings in the class
    bool ok = pImpl->loadSettingsFromConfig(config);
    if (!ok) {
        yError() << logPrefix << "Problem in loading settings from config.";
        return false;
    }

    return true;
}

bool HumanLogger::impl::setLoggerType(const std::string& str)
{
    if (!std::strcmp(str.c_str(), "matlab")) {
        this->loggerType = LoggerType::MATLAB;
    }
    else {
        this->loggerType = LoggerType::NONE;
        return false;
    }

    return true;
}

bool HumanLogger::impl::loadSettingsFromConfig(yarp::os::Searchable& config)
{
    // Check for logger type parameter
    this->loggerType = LoggerType::NONE;
    if (!(config.check("LoggerType")
          && config.find("LoggerType").isString()
          && setLoggerType(config.find("LoggerType").asString()) ) ) {
 
        yError() << logPrefix << "LoggerType not found or not valid";
        return false;
    }

    // Display the current logger level
    switch (this->loggerType) {
        case LoggerType::MATLAB: {
            yInfo() << logPrefix << "LoggerType set to MATLAB";
            break;
        }
        default:
            break;
    }

    // load logger flag settings
    checkAndLoadBooleanOption(config, "logHumanState", settings.logHumanState);
    checkAndLoadBooleanOption(config, "logHumanDynamics", settings.logHumanDynamics);

    // load buffer manager configuration settings
    checkAndLoadBooleanOption(
        config, "saveBufferManagerConfiguration", settings.saveBufferManagerConfiguration);

    std::string experimentName = "experimentName";
    if (config.check(experimentName.c_str()) && config.find(experimentName.c_str()).isString()) {
        bufferConfig.filename = config.find(experimentName.c_str()).asString();
    }
    else {
        yError() << logPrefix << " missing parameter: " << experimentName;
        return false;
    }

    std::string path = "path";
    if (config.check(path.c_str()) && config.find(path.c_str()).isString()) {
        bufferConfig.path = config.find(path.c_str()).asString();
    }

    std::string n_samples = "n_samples";
    if (config.check(n_samples.c_str()) && config.find(n_samples.c_str()).isInt32()) {
        bufferConfig.n_samples = config.find(n_samples.c_str()).asInt32();
    }
    else {
        yError() << logPrefix << " missing parameter: " << n_samples;
        return false;
    }

    std::string save_periodically = "save_periodically";
    if (config.check(save_periodically.c_str()) && config.find(save_periodically.c_str()).isBool()) {
        bufferConfig.save_periodically = config.find(save_periodically.c_str()).asBool();
    }

    if (bufferConfig.save_periodically) {
        std::string save_period = "save_period";
        if (config.check(save_period.c_str()) && config.find(save_period.c_str()).isFloat64()) {
            bufferConfig.save_period = config.find(save_period.c_str()).asFloat64();
        }
        else {
            yError() << logPrefix << " missing parameter: " << save_period;
            return false;
        }

        std::string data_threshold = "data_threshold";
        if (config.check(data_threshold.c_str()) && config.find(data_threshold.c_str()).isInt32()) {
            bufferConfig.data_threshold = config.find(data_threshold.c_str()).asInt32();
        }
    }

    std::string auto_save = "auto_save";
    if (config.check(auto_save.c_str()) && config.find(auto_save.c_str()).isBool()) {
        bufferConfig.auto_save = config.find(auto_save.c_str()).asBool();
    }

    if (!(bufferConfig.auto_save || bufferConfig.save_periodically)) {
        yError()
            << logPrefix
            << " both auto_save and save_periodically are set to false, nothing will be saved.";
        return false;
    }

    return true;
}

void HumanLogger::impl::checkAndLoadBooleanOption(yarp::os::Searchable& config,
                                                  const std::string& optionName,
                                                  bool& option)
{
    if (config.check(optionName.c_str())) {
        option = config.find(optionName.c_str()).asBool();
    }
}

bool HumanLogger::close()
{
    if (!pImpl->bufferConfig.auto_save) {
        pImpl->bufferManager.saveToFile();
    }
    bool ok{true};
    if (pImpl->settings.saveBufferManagerConfiguration) {
        auto buffConfToSave = pImpl->bufferManager.getBufferConfig();
        ok = bufferConfigToJson(buffConfToSave,
                                buffConfToSave.path + "bufferConfig" + buffConfToSave.filename
                                    + ".json");
    }
    return ok;
}


bool HumanLogger::impl::configureBufferManager()
{
    bool ok{true};

    if(settings.logHumanState) {
        if (loggerType == LoggerType::MATLAB)
        {
            jointNamesStateInterface =  iHumanState->getJointNames();
            ok = ok && bufferManager.addChannel({"human_state::base_position", {3, 1}});
            ok = ok && bufferManager.addChannel({"human_state::base_orientation", {4, 1}});
            ok = ok && bufferManager.addChannel({"human_state::base_velocity", {6, 1}});
            ok = ok && bufferManager.addChannel({"human_state::joint_positions", {jointNamesStateInterface.size(), 1}});
            ok = ok && bufferManager.addChannel({"human_state::joint_velocities", {jointNamesStateInterface.size(), 1}});
        }
    }
    if (settings.logHumanDynamics)
    {
        if (loggerType == LoggerType::MATLAB)
        {
            jointNamesDynamicsInterface =  iHumanDynamics->getJointNames();
            ok = ok && bufferManager.addChannel({"human_dynamics::joint_torques", {jointNamesDynamicsInterface.size(), 1}});
        }
    }
    

    ok = ok && bufferManager.configure(bufferConfig);
    if (ok) {
        yDebug() << logPrefix << " buffer manager configured successfully.";
    }

    return ok;
}

void HumanLogger::threadRelease() {}

// ==========================
// IMultipleWrapper interface
// ==========================

bool HumanLogger::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    bool attachStatus = true;

    if (driverList.size() > 2) {
        yError() << logPrefix << "This wrapper accepts maximum two attached PolyDriver.";
        return false;
    }

    for (int i = 0; i < driverList.size(); i++) {
        yarp::dev::PolyDriver* poly = driverList[i]->poly;

        if (!poly) {
            yError() << logPrefix << "Passed PolyDriver is nullptr.";
            return false;
        }

        // Get the device name from the driver
        const std::string deviceName = poly->getValue("device").asString();
        interfaces::IHumanState* tmpIHumanState = nullptr;
        interfaces::IHumanDynamics* tmpIHumanDynamics = nullptr;

        // View IHumanState
        if(pImpl->settings.logHumanState && !pImpl->iHumanState && poly->view(tmpIHumanState))
        {
            // Check the interface
            if (tmpIHumanState->getNumberOfJoints() == 0
                    || tmpIHumanState->getNumberOfJoints() != tmpIHumanState->getJointNames().size()) {
                yError() << logPrefix <<"The IHumanState interface"<<deviceName<<"might not be ready";
                return false;
            }

            pImpl->iHumanState = tmpIHumanState;
        }

        // View IHumanDynamics
        if(pImpl->settings.logHumanDynamics && !pImpl->iHumanDynamics && poly->view(tmpIHumanDynamics))
        {
            // Check the interface
            if (tmpIHumanDynamics->getNumberOfJoints() == 0
                    || tmpIHumanDynamics->getNumberOfJoints() != tmpIHumanDynamics->getJointNames().size()) {
                yError() << logPrefix << "The IHumanDynamics interface"<<deviceName<<"might not be ready";
                return false;
            }

            pImpl->iHumanDynamics = tmpIHumanDynamics;
        }

        if(!tmpIHumanState && !tmpIHumanDynamics)
        {
            yWarning()<<logPrefix<<"The device"<<deviceName<<"does not implement any supported device, so it cannot be attached";
        }
        else
        {
            yInfo()<<logPrefix<<"Device"<<deviceName<<"successfully attached";
        }
    }

    // Check IHumanState interface
    if (pImpl->settings.logHumanState && !pImpl->iHumanState)
    {
        yError()<<logPrefix<<"No IHumanState interface attached, stopping";
        return false;
    }

    // Check IHumanDynamics interface
    if (pImpl->settings.logHumanDynamics && !pImpl->iHumanDynamics)
    {
        yError()<<logPrefix<<"No IHumanDynamics interface attached, stopping";
        return false;
    }

    if (!pImpl->configureBufferManager()) {
        yError() << logPrefix << "Failed to configure buffer manager for the logger.";
        return false;
    }

    // Start the periodic thread
    if(!start()) {
        yError() << logPrefix << "Failed to start the loop.";
        return false;
    }
    
    yInfo() << logPrefix << "Successfully attached all the required devices";

    return true;
}

bool HumanLogger::detachAll()
{
    while (isRunning()) {
        stop();
    }

    pImpl->iHumanState = nullptr;
    pImpl->iHumanDynamics = nullptr;

    return true;
}
