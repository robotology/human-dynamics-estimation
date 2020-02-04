/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanDataCollector.h"
#include "IHumanState.h"
#include "IHumanWrench.h"
#include "IHumanDynamics.h"

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

    double period;
    std::string portPrefix;
    bool firstData = true;

    hde::interfaces::IHumanState* iHumanState = nullptr;
    hde::interfaces::IHumanWrench* iHumanWrenchMeasurements = nullptr;
    hde::interfaces::IHumanWrench* iHumanWrenchEstimates = nullptr; //This interface points to HumanDynamicsEstimator, which gives offset removed wrench measurements and the estimates
    hde::interfaces::IHumanDynamics* iHumanDynamics = nullptr;

    bool isStateProviderDevice = false;
    bool isWrenchProviderDevice = false;
    bool isDynamicsEstimatorDevice = false;

    //TODO: Decide the names and the number of ports needed for IHumanState interface data
    // Yarp ports for streaming data from IHumanState of HumanStateProvider
    yarp::os::BufferedPort<yarp::sig::Vector> basePoseDataPort;
};

HumanDataCollector::HumanDataCollector()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanDataCollector::~HumanDataCollector() {}

bool HumanDataCollector::open(yarp::os::Searchable &config) {

    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period";
    }

    if (!(config.check("portPrefix") && config.find("portPrefix").isString())) {
        yInfo() << LogPrefix << "Using default portPrefix HDE ";
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(0.01)).asFloat64();
    pImpl->portPrefix = config.check("portPrefix", yarp::os::Value("HDE")).asString();

    //TODO: Define rpc to reset the data dump or restarting the visualization??

    yInfo() << LogPrefix << "*** ===========================";
    yInfo() << LogPrefix << "*** Period                    :" << period;
    yInfo() << LogPrefix << "*** portPrefix                :" << pImpl->portPrefix;
    yInfo() << LogPrefix << "*** ===========================";

    // Set period
    setPeriod(pImpl->period);

    return true;
}

bool HumanDataCollector::close() {
    return true;
}

void HumanDataCollector::threadRelease() {}

void HumanDataCollector::run()
{
    if (pImpl->firstData) {

        // =====================================================================
        // Open yarp ports base on the attached devices for streaming human data
        // =====================================================================

        if (pImpl->isStateProviderDevice) {

            // Open base pose data port
            const std::string basePosePortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/basePose:0";
            if (!pImpl->basePoseDataPort.open(basePosePortName)) {
                yError() << LogPrefix << "Failed to open port " << basePosePortName;
                askToStop();
            }

        }


    }


    // =========================================================
    // Get data from different interfaces from attached devices
    // =========================================================

    // Get data from IHumanState interface of HumanStateProvider
    if (pImpl->isStateProviderDevice) {

        std::array<double, 3> CoMPositionInterface = pImpl->iHumanState->getCoMPosition();
        std::array<double, 3> CoMVelocityInterface = pImpl->iHumanState->getCoMVelocity();

        // Base Quantities
        std::string baseName = pImpl->iHumanState->getBaseName();
        std::array<double, 3> basePositionInterface = pImpl->iHumanState->getBasePosition();
        std::array<double, 4> baseOrientationInterface = pImpl->iHumanState->getBaseOrientation();
        std::array<double, 6> baseVelocity = pImpl->iHumanState->getBaseVelocity();

        // Joint Quantities
        size_t stateJoints = pImpl->iHumanState->getNumberOfJoints();
        std::vector<std::string> stateJointNames = pImpl->iHumanState->getJointNames();
        std::vector<double> jointPositionsInterface = pImpl->iHumanState->getJointPositions();
        std::vector<double> jointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();

    }

    // Get data from IHumanWrench interface of HumanWrenchProvider
    if (pImpl->isWrenchProviderDevice) {

        size_t numberOfWrenchMeasurementSources = pImpl->iHumanWrenchMeasurements->getNumberOfWrenchSources();
        std::vector<std::string> wrenchMeasurementsSourceNames = pImpl->iHumanWrenchMeasurements->getWrenchSourceNames();
        std::vector<double> wrenchMeasurementsValues = pImpl->iHumanWrenchMeasurements->getWrenches();

    }

    // Get data from IHumanWrench interface of HumanDynamicsEstimator
    // NOTE: The wrench values coming from HumanDynamicsEstimators are (offsetRemovedWrenchMeasurements & WrenchEstimates) of each link
    if (pImpl->isDynamicsEstimatorDevice) {

        size_t numberOfWrenchEstimatesSources = pImpl->iHumanWrenchEstimates->getNumberOfWrenchSources();
        std::vector<std::string> wrenchEstimatesSourceNames = pImpl->iHumanWrenchEstimates->getWrenchSourceNames();
        std::vector<double> wrenchEstimatesValues = pImpl->iHumanWrenchEstimates->getWrenches();

        // Get data from IHumanDynamics interface of HumanDynamicsEstimator
        size_t dynamicsJoints = pImpl->iHumanDynamics->getNumberOfJoints();
        std::vector<std::string> dynamicsJointNames = pImpl->iHumanDynamics->getJointNames();
        std::vector<double> jointTorques = pImpl->iHumanDynamics->getJointTorques();

    }

    // Flip firstData flag false
    if (pImpl->firstData) {
        pImpl->firstData = false;
    }

    // ============================
    // Put human data to yarp ports
    // ============================



}

bool HumanDataCollector::attach(yarp::dev::PolyDriver *poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver device is nullptr";
        return false;
    }

    // Get the passed device name
    const std::string deviceName = poly->getValue("device").asString();

    // Ensure the passed device is acceptable
    if (deviceName != "human_state_provider" && deviceName != "human_wrench_provider" && deviceName != "human_dynamics_estimator") {
        yError() << "This wrapper does not accept the " << deviceName << " passed as the polydriver device for attaching";
        return  false;
    }

    if (deviceName == "human_state_provider") {

        // Attach to IHumanState interface from HumanStateProvider
        if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
            yError() << LogPrefix << "Failed to view IHumanState interface from the " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanState->getNumberOfJoints() == 0 || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
            yError() << "The viewed IHumanState interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
        pImpl->isStateProviderDevice = true;
    }

    if (deviceName == "human_wrench_provider") {

        // Attach to IHumanWrench interface from HumanWrenchProvider
        if (pImpl->iHumanWrenchMeasurements || !poly->view(pImpl->iHumanWrenchMeasurements) || !pImpl->iHumanWrenchMeasurements) {
            yError() << LogPrefix << "Failed to view IHumanWrench interface from the " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanWrenchMeasurements->getNumberOfWrenchSources() != pImpl->iHumanWrenchMeasurements->getWrenchSourceNames().size()) {
            yError() << "The viewed IHumanWrench interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
        pImpl->isWrenchProviderDevice = true;

    }

    if (deviceName == "human_dynamics_estimator") {

        // Attach to IHumanWrench interface from HumanDynamicsEstimator
        if (pImpl->iHumanWrenchEstimates || !poly->view(pImpl->iHumanWrenchEstimates) || !pImpl->iHumanWrenchEstimates) {
            yError() << LogPrefix << "Failed to view IHumanWrench interface from the " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanWrenchEstimates->getNumberOfWrenchSources() != pImpl->iHumanWrenchEstimates->getWrenchSourceNames().size()) {
            yError() << "The viewed IHumanWrench interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        // Attach to IHumanDynamics interface from HumanDynamicsEstimator
        if (pImpl->iHumanDynamics || !poly->view(pImpl->iHumanDynamics) || !pImpl->iHumanDynamics) {
            yError() << LogPrefix << "Failed to view IHumanDynamics interface from the  " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanDynamics->getNumberOfJoints() == 0 || pImpl->iHumanDynamics->getNumberOfJoints() != pImpl->iHumanDynamics->getJointNames().size()) {
            yError() << "The viewed IHumanDynamics interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
        pImpl->isDynamicsEstimatorDevice = true;
    }

    return true;
}

bool HumanDataCollector::detach()
{
    while (isRunning()) {
        stop();
    }

    //TODO: Close all the yarp ports

    pImpl->iHumanState = nullptr;
    pImpl->iHumanWrenchMeasurements = nullptr;
    pImpl->iHumanWrenchEstimates = nullptr;
    pImpl->iHumanDynamics = nullptr;

    return false;
}

bool HumanDataCollector::attachAll(const yarp::dev::PolyDriverList &driverList)
{
    bool attachStatus = true;

    if (driverList.size() > 3) {
        yError() << LogPrefix << "This wrapper accepts at most three attached PolyDriver devices";
        return false;
    }

    for (size_t i = 0; i < driverList.size(); i++) {
        const yarp::dev::PolyDriverDescriptor* driver = driverList[i];

        if (!driver) {
            yError() << LogPrefix << "Passed PolyDriver device is nullptr";
            return false;
        }

        attachStatus = attachStatus && attach(driver->poly);
    }

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (attachStatus && !start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    return attachStatus;
}

bool HumanDataCollector::detachAll()
{
    return detach();
}
