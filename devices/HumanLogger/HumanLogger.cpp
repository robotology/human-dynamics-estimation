// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanLogger.h"
#include <hde/interfaces/IHumanState.h>
#include <hde/interfaces/IHumanDynamics.h>
#include <hde/interfaces/IHumanWrench.h>

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
const std::string LogPrefix = LoggerName + " :";
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
    bool logHumanWrench{false};
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
    interfaces::IHumanWrench* iHumanWrench = nullptr;
    HumanLoggerSettings settings;
    robometry::BufferConfig bufferConfig;
    robometry::BufferManager bufferManager;

    // buffer variables
    // iHumanState
    std::array<double, 3> basePosition;
    std::array<double, 4> baseOrientation;
    std::array<double, 6> baseVelocity;
    std::vector<double> jointPositions;
    std::vector<double> jointVelocities;
    std::vector<std::string> jointNamesState;
    // iHumanDynamics
    std::vector<double> jointTorques;
    std::vector<std::string> jointNamesDynamics;
    // iHumanWrench
    std::vector<double> wrenches;
    std::vector<std::string> wrenchSourceNames;


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
            yError() << LogPrefix << "The IHumanState pointer is null in the driver loop.";
            askToStop();
            return;
        }

        pImpl->basePosition = pImpl->iHumanState->getBasePosition();
        pImpl->baseOrientation = pImpl->iHumanState->getBaseOrientation();
        pImpl->baseVelocity = pImpl->iHumanState->getBaseVelocity();
        pImpl->jointPositions = pImpl->iHumanState->getJointPositions();
        pImpl->jointVelocities = pImpl->iHumanState->getJointVelocities();

        if (pImpl->loggerType == LoggerType::MATLAB) {
            pImpl->bufferManager.push_back(pImpl->basePosition,
                                           timeNow,
                                           "human_state::base_position");
            pImpl->bufferManager.push_back(pImpl->baseOrientation,
                                           timeNow,
                                           "human_state::base_orientation");
            pImpl->bufferManager.push_back(pImpl->baseVelocity,
                                           timeNow,
                                           "human_state::base_velocity");
            pImpl->bufferManager.push_back(pImpl->jointPositions,
                                           timeNow,
                                           "joints_state::positions");
            pImpl->bufferManager.push_back(pImpl->jointVelocities,
                                           timeNow,
                                           "joints_state::velocities");
        }
    }

    if (pImpl->settings.logHumanDynamics)
    {
        if(!pImpl->iHumanDynamics) {
            yError() << LogPrefix << "The IHumanDynamics pointer is null in the driver loop.";
            askToStop();
            return;
        }

        pImpl->jointTorques = pImpl->iHumanDynamics->getJointTorques();

        if (pImpl->loggerType == LoggerType::MATLAB) {
            pImpl->bufferManager.push_back(pImpl->jointTorques,
                                           timeNow,
                                           "human_dynamics::joint_torques");
        }
    }

    if (pImpl->settings.logHumanWrench)
    {
        if(!pImpl->iHumanWrench) {
            yError() << LogPrefix << "The IHumanWrench pointer is null in the driver loop.";
            askToStop();
            return;
        }

        pImpl->wrenches = pImpl->iHumanWrench->getWrenches();

        if (pImpl->loggerType == LoggerType::MATLAB) {
            pImpl->bufferManager.push_back(pImpl->wrenches,
                                           timeNow,
                                           "human_wrench::wrenches");
        }
    }
}

// ======================
// DeviceDriver interface
// ======================

bool HumanLogger::open(yarp::os::Searchable& config)
{
    if (!config.check("period")) {
        yInfo() << LogPrefix << "Using default period: " << DefaultPeriod << "s";
    }

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    setPeriod(period);

    // Load settings in the class
    bool ok = pImpl->loadSettingsFromConfig(config);
    if (!ok) {
        yError() << LogPrefix << "Problem in loading settings from config.";
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
 
        yError() << LogPrefix << "LoggerType not found or not valid";
        return false;
    }

    // Display the current logger level
    switch (this->loggerType) {
        case LoggerType::MATLAB: {
            yInfo() << LogPrefix << "LoggerType set to MATLAB";
            break;
        }
        default:
            break;
    }

    // load logger flag settings
    checkAndLoadBooleanOption(config, "logHumanState", settings.logHumanState);
    checkAndLoadBooleanOption(config, "logHumanDynamics", settings.logHumanDynamics);
    checkAndLoadBooleanOption(config, "logHumanWrench", settings.logHumanWrench);

    // load buffer manager configuration settings
    checkAndLoadBooleanOption(
        config, "saveBufferManagerConfiguration", settings.saveBufferManagerConfiguration);

    std::string experimentName = "experimentName";
    if (config.check(experimentName.c_str()) && config.find(experimentName.c_str()).isString()) {
        bufferConfig.filename = config.find(experimentName.c_str()).asString();
    }
    else {
        yError() << LogPrefix << " missing parameter: " << experimentName;
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
        yError() << LogPrefix << " missing parameter: " << n_samples;
        return false;
    }

    std::string save_periodically = "save_periodically";
    if (config.check(save_periodically.c_str()) && config.find(save_periodically.c_str()).isBool()) {
        bufferConfig.save_periodically = config.find(save_periodically.c_str()).asBool();
    }

    std::string yarpRobotName = "yarp_robot_name";
    std::string defaultYarpRobotName = "human-gazebo";
    if (config.check(yarpRobotName.c_str()) && config.find(yarpRobotName.c_str()).isString()) {
        bufferConfig.yarp_robot_name = config.find(yarpRobotName.c_str()).asString();
    }
    else {
        bufferConfig.yarp_robot_name = defaultYarpRobotName;
        yInfo() << LogPrefix << " missing parameter: " << yarpRobotName
                << " using default value: " << defaultYarpRobotName;
    }

    if (bufferConfig.save_periodically) {
        std::string save_period = "save_period";
        if (config.check(save_period.c_str()) && config.find(save_period.c_str()).isFloat64()) {
            bufferConfig.save_period = config.find(save_period.c_str()).asFloat64();
        }
        else {
            yError() << LogPrefix << " missing parameter: " << save_period;
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

    bufferConfig.mat_file_version = matioCpp::FileVersion::MAT7_3;

    if (!(bufferConfig.auto_save || bufferConfig.save_periodically)) {
        yError()
            << LogPrefix
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
            jointNamesState =  iHumanState->getJointNames();
            ok = ok && bufferManager.addChannel({"human_state::base_position", {3, 1}});
            ok = ok && bufferManager.addChannel({"human_state::base_orientation", {4, 1}});
            ok = ok && bufferManager.addChannel({"human_state::base_velocity", {6, 1}});
            ok = ok && bufferManager.addChannel({"joints_state::positions", {jointNamesState.size(), 1}, jointNamesState});
            ok = ok && bufferManager.addChannel({"joints_state::velocities", {jointNamesState.size(), 1}, jointNamesState});
        }
    }
    if (settings.logHumanDynamics)
    {
        if (loggerType == LoggerType::MATLAB)
        {
            jointNamesDynamics =  iHumanDynamics->getJointNames();
            ok = ok && bufferManager.addChannel({"human_dynamics::joint_torques", {jointNamesDynamics.size(), 1}, jointNamesDynamics});
        }
    }
    if (settings.logHumanWrench)
    {
        if (loggerType == LoggerType::MATLAB)
        {
            wrenchSourceNames =  iHumanWrench->getWrenchSourceNames();
            ok = ok && bufferManager.addChannel({"human_wrench::wrenches", {6 * wrenchSourceNames.size(), 1}, wrenchSourceNames});
        }
    }
    

    ok = ok && bufferManager.configure(bufferConfig);
    if (ok) {
        yDebug() << LogPrefix << " buffer manager configured successfully.";
    }

    return ok;
}

void HumanLogger::threadRelease() {}

// ==========================
// IMultipleWrapper interface
// ==========================

bool HumanLogger::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 3) {
        yError() << LogPrefix << "This wrapper accepts maximum three attached PolyDriver.";
        return false;
    }

    for (int i = 0; i < driverList.size(); i++) {
        yarp::dev::PolyDriver* poly = driverList[i]->poly;

        if (!poly) {
            yError() << LogPrefix << "Passed PolyDriver is nullptr.";
            return false;
        }

        // Get the device name from the driver
        const std::string deviceName = poly->getValue("device").asString();
        interfaces::IHumanState* tmpIHumanState = nullptr;
        interfaces::IHumanDynamics* tmpIHumanDynamics = nullptr;
        interfaces::IHumanWrench* tmpIHumanWrench = nullptr;

        // View IHumanState
        if(pImpl->settings.logHumanState && !pImpl->iHumanState && poly->view(tmpIHumanState))
        {
            // Check the interface
            while (tmpIHumanState->getNumberOfJoints() == 0
                    || tmpIHumanState->getNumberOfJoints() != tmpIHumanState->getJointNames().size()) {
                yInfo() << LogPrefix << "IHumanState interface waiting for first data. Waiting...";
                yarp::os::Time::delay(5);
            }

            pImpl->iHumanState = tmpIHumanState;
        }

        // View IHumanDynamics
        if(pImpl->settings.logHumanDynamics && !pImpl->iHumanDynamics && poly->view(tmpIHumanDynamics))
        {
            // Check the interface
            while (tmpIHumanDynamics->getNumberOfJoints() == 0
                    || tmpIHumanDynamics->getNumberOfJoints() != tmpIHumanDynamics->getJointNames().size()) {
                yInfo() << LogPrefix << "IHumanDynamics interface waiting for first data. Waiting...";
                yarp::os::Time::delay(5);
            }

            pImpl->iHumanDynamics = tmpIHumanDynamics;
        }

        // View IHumanWrench
        if(pImpl->settings.logHumanWrench && !pImpl->iHumanWrench && poly->view(tmpIHumanWrench))
        {
            // Check the interface
            auto numberOfWrenchSources = tmpIHumanWrench->getNumberOfWrenchSources();
            while ( numberOfWrenchSources == 0 ||
                numberOfWrenchSources != tmpIHumanWrench->getWrenchSourceNames().size()) {
                yInfo() << LogPrefix << "IHumanWrench interface waiting for first data. Waiting...";
                yarp::os::Time::delay(5);
                numberOfWrenchSources = tmpIHumanWrench->getNumberOfWrenchSources();
            }

            pImpl->iHumanWrench = tmpIHumanWrench;
        }

        if(!tmpIHumanState && !tmpIHumanDynamics && !tmpIHumanWrench)
        {
            yError()<<LogPrefix<<"The device"<<deviceName<<"does not implement any of the attachable interfaces!";
            return false;
        }
        else
        {
            yInfo()<<LogPrefix<<"Device"<<deviceName<<"successfully attached";
        }
    }

    // Check IHumanState interface
    if (pImpl->settings.logHumanState && !pImpl->iHumanState)
    {
        yError()<<LogPrefix<<"No IHumanState interface attached, stopping";
        return false;
    }

    // Check IHumanDynamics interface
    if (pImpl->settings.logHumanDynamics && !pImpl->iHumanDynamics)
    {
        yError()<<LogPrefix<<"No IHumanDynamics interface attached, stopping";
        return false;
    }

    // Check IHumanDynamics interface
    if (pImpl->settings.logHumanWrench && !pImpl->iHumanWrench)
    {
        yError()<<LogPrefix<<"No IHumanWrench interface attached, stopping";
        return false;
    }

    if (!pImpl->configureBufferManager()) {
        yError() << LogPrefix << "Failed to configure buffer manager for the logger.";
        return false;
    }

    // Start the periodic thread
    if(!start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    pImpl->bufferManager.setDescriptionList(pImpl->jointNamesState);
    
    yInfo() << LogPrefix << "Successfully attached all the required devices";

    return true;
}

bool HumanLogger::detachAll()
{
    while (isRunning()) {
        stop();
    }

    pImpl->iHumanState = nullptr;
    pImpl->iHumanDynamics = nullptr;
    pImpl->iHumanWrench = nullptr;

    return true;
}
