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

#include <yarp/telemetry/experimental/BufferManager.h>

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

using MatlabChannelName = std::string;

class HumanLogger::impl
{
public:
    LoggerType loggerType;
    void setLoggerType(std::string& str);

    bool loadSettingsFromConfig(yarp::os::Searchable& config);
    void checkAndLoadBooleanOption(yarp::os::Property& prop,
                                   const std::string& optionName,
                                   bool& option);
    bool configureBufferManager();

    bool firstRun = true;
    size_t waitingFirstReadCounter = 1;

    interfaces::IHumanState* iHumanState = nullptr;
    interfaces::IHumanDynamics* iHumanDynamics = nullptr;
    std::mutex loggerMutex;
    HumanLoggerSettings settings;
    yarp::telemetry::experimental::BufferConfig bufferConfig;
    yarp::telemetry::experimental::BufferManager<double> bufferManager;

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
    std::lock_guard<std::mutex> guard(pImpl->loggerMutex);
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

void HumanLogger::impl::setLoggerType(std::string& str)
{
    if (!std::strcmp(str.c_str(), "matlab")) {
        this->loggerType = LoggerType::MATLAB;
    }
}

bool HumanLogger::impl::loadSettingsFromConfig(yarp::os::Searchable& config)
{
    // Check for logLevel parameter
    this->loggerType = LoggerType::NONE;
    if (!(config.check("LoggerType")
          && (config.find("LoggerType").isString() || config.find("LoggerType").isList()))) {
        yInfo() << logPrefix << "Using default LoggerType : MATLAB";
        this->loggerType = LoggerType::MATLAB;
    }
    else if (config.check("LoggerType") && config.find("LoggerType").isList()) {
        yarp::os::Bottle* loggerTypeList = config.find("LoggerType").asList();

        for (size_t i = 0; i < loggerTypeList->size(); i++) {
            std::string option = loggerTypeList->get(i).asString();

            setLoggerType(option);
        }
    }
    else if (config.check("LoggerType") && config.find("LoggerType").isString()) {
        std::string option = config.find("LoggerType").asString();
        setLoggerType(option);
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

    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    // load logger flag settings
    checkAndLoadBooleanOption(prop, "logHumanState", settings.logHumanState);
    checkAndLoadBooleanOption(prop, "logHumanDynamics", settings.logHumanDynamics);

    // load buffer manager configuration settings
    checkAndLoadBooleanOption(
        prop, "saveBufferManagerConfiguration", settings.saveBufferManagerConfiguration);

    std::string experimentName = "experimentName";
    if (prop.check(experimentName.c_str()) && prop.find(experimentName.c_str()).isString()) {
        bufferConfig.filename = prop.find(experimentName.c_str()).asString();
    }
    else {
        yError() << logPrefix << " missing parameter: " << experimentName;
        return false;
    }

    std::string path = "path";
    if (prop.check(path.c_str()) && prop.find(path.c_str()).isString()) {
        bufferConfig.path = prop.find(path.c_str()).asString();
    }

    std::string n_samples = "n_samples";
    if (prop.check(n_samples.c_str()) && prop.find(n_samples.c_str()).isInt32()) {
        bufferConfig.n_samples = prop.find(n_samples.c_str()).asInt32();
    }
    else {
        yError() << logPrefix << " missing parameter: " << n_samples;
        return false;
    }

    std::string save_periodically = "save_periodically";
    if (prop.check(save_periodically.c_str()) && prop.find(save_periodically.c_str()).isBool()) {
        bufferConfig.save_periodically = prop.find(save_periodically.c_str()).asBool();
    }

    if (bufferConfig.save_periodically) {
        std::string save_period = "save_period";
        if (prop.check(save_period.c_str()) && prop.find(save_period.c_str()).isFloat64()) {
            bufferConfig.save_period = prop.find(save_period.c_str()).asFloat64();
        }
        else {
            yError() << logPrefix << " missing parameter: " << save_period;
            return false;
        }

        std::string data_threshold = "data_threshold";
        if (prop.check(data_threshold.c_str()) && prop.find(data_threshold.c_str()).isInt32()) {
            bufferConfig.data_threshold = prop.find(data_threshold.c_str()).asInt32();
        }
    }

    std::string auto_save = "auto_save";
    if (prop.check(auto_save.c_str()) && prop.find(auto_save.c_str()).isBool()) {
        bufferConfig.auto_save = prop.find(auto_save.c_str()).asBool();
    }

    if (!(bufferConfig.auto_save || bufferConfig.save_periodically)) {
        yError()
            << logPrefix
            << " both auto_save and save_periodically are set to false, nothing will be saved.";
        return false;
    }

    return true;
}

void HumanLogger::impl::checkAndLoadBooleanOption(yarp::os::Property& prop,
                                                  const std::string& optionName,
                                                  bool& option)
{
    if (prop.check(optionName.c_str())) {
        option = prop.find(optionName.c_str()).asBool();
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

// ==================
// IWrapper interface
// ==================

bool HumanLogger::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << logPrefix << "Passed PolyDriver is nullptr.";
        return false;
    }

    // Get the device name from the driver
    const std::string deviceName = poly->getValue("device").asString();
    std::cerr << "attaching " << deviceName << std::endl;
    if (deviceName == "human_state_provider" || deviceName == "human_state_remapper") {
        // Attach IHumanState interface
        if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
            yError() << logPrefix << "Failed to view IHumanState interface from the polydriver";
            return false;
        }

        // Check the interface
        if (pImpl->iHumanState->getNumberOfJoints() == 0
                || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
            yError() << "The IHumanState interface might not be ready";
            return false;
        }

        yInfo() << logPrefix << deviceName << "attach() successful";
    }

    if (deviceName == "human_dynamics_estimator" || deviceName == "human_dynamics_remapper") {
        // Attach IHumanDynamics interface
        if (pImpl->iHumanDynamics || !poly->view(pImpl->iHumanDynamics) || !pImpl->iHumanDynamics) {
            yError() << logPrefix << "Failed to view IHumanDynamics interface from the polydriver";
            return false;
        }

        // Check the interface
        if (pImpl->iHumanDynamics->getNumberOfJoints() == 0
                || pImpl->iHumanDynamics->getNumberOfJoints() != pImpl->iHumanDynamics->getJointNames().size()) {
            yError() << "The IHumanDynamics interface might not be ready";
            return false;
        }

        yInfo() << logPrefix << deviceName << "attach() successful";
    }

    return true;
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

bool HumanLogger::detach()
{
    std::lock_guard<std::mutex> guard(pImpl->loggerMutex);
    while (isRunning()) {
        stop();
    }

    pImpl->iHumanState = nullptr;
    pImpl->iHumanDynamics = nullptr;

    return true;
}

// ==========================
// IMultipleWrapper interface
// ==========================

bool HumanLogger::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    bool attachStatus = true;

    if (driverList.size() > 2) {
        yError() << logPrefix << "This wrapper accepts maximum three attached PolyDriver.";
        return false;
    }

    for (size_t i = 0; i < driverList.size(); i++) {
        const yarp::dev::PolyDriverDescriptor* driver = driverList[i];

        if (!driver) {
            yError() << logPrefix << "Passed PolyDriverDescriptor is nullptr";
            return false;
        }

        attachStatus = attachStatus && attach(driver->poly);
    }

    if (!pImpl->configureBufferManager()) {
        yError() << logPrefix << "Failed to configure buffer manager for the logger.";
    }

    if (attachStatus) {
        yDebug() << logPrefix << "attach() successful";
    }

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (attachStatus && !start()) {
        yError() << logPrefix << "Failed to start the loop.";
        return false;
    }

    return attachStatus;
}

bool HumanLogger::detachAll()
{
    return detach();
}