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

#include "Utils.hpp"

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

    DataBuffersConversionHelper dataBuffersConversionHelper;

    hde::interfaces::IHumanState* iHumanState = nullptr;
    hde::interfaces::IHumanWrench* iHumanWrenchMeasurements = nullptr;
    hde::interfaces::IHumanWrench* iHumanWrenchEstimates = nullptr; //This interface points to HumanDynamicsEstimator, which gives offset removed wrench measurements and the estimates
    hde::interfaces::IHumanDynamics* iHumanDynamics = nullptr;

    bool isStateProviderDevice = false;
    bool isWrenchProviderDevice = false;
    bool isDynamicsEstimatorDevice = false;

    // Interface data buffers

    // Base Quantities
    std::string baseName;
    std::array<double, 3> basePosition;
    std::array<double, 4> baseOrientation;
    std::array<double, 6> baseVelocity;

    // Joint Quantities
    size_t stateNumberOfJoints;
    std::vector<std::string> stateJointNames;
    std::vector<double> jointPositions;
    std::vector<double> jointVelocities;

    // CoM Quantities
    std::array<double, 3> comPosition;
    std::array<double, 3> comVelocity;
    std::array<double, 6> comProperAccInBaseFrame;
    std::array<double, 6> comProperAccInWorldFrame;

    // Wrench Measurements
    size_t numberOfWrenchMeasurementSources;
    std::vector<std::string> wrenchMeasurementsSourceNames;
    std::vector<double> wrenchMeasurementsValues;

    // Wrench Estimates
    size_t numberOfWrenchEstimatesSources;
    std::vector<std::string> wrenchEstimatesSourceNames;
    std::vector<double> wrenchEstimatesValues;

    // Joint Torques
    size_t dynamicsNumberOfJoints;
    std::vector<std::string> dynamicsJointNames;
    std::vector<double> jointTorques;

    //TODO: Decide the names and the number of ports needed for IHumanState interface data
    // Yarp ports for streaming data from IHumanState of HumanStateProvider
    yarp::os::BufferedPort<yarp::sig::Vector> basePoseDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> baseVelocityDataPort;
    yarp::os::BufferedPort<yarp::os::Bottle> stateJointNamesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> jointPositionsDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> jointVelocitiesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comPositionDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comVelocityDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comProperAccelerationInBaseFrameDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comProperAccelerationInWorldFrameDataPort;
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
            const std::string basePosePortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/basePose:o";
            if (!pImpl->basePoseDataPort.open(basePosePortName)) {
                yError() << LogPrefix << "Failed to open port " << basePosePortName;
                askToStop();
            }

            // Open base velocity data port
            const std::string baseVelocityPortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/baseVelocity:o";
            if (!pImpl->baseVelocityDataPort.open(baseVelocityPortName)) {
                yError() << LogPrefix << "Failed to open port " << baseVelocityPortName;
                askToStop();
            }

            // Open state jonit names data port
            const std::string stateJointNamesPortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/stateJointNames:o";
            if (!pImpl->stateJointNamesDataPort.open(stateJointNamesPortName)) {
                yError() << LogPrefix << "Failed to open port " << stateJointNamesPortName;
                askToStop();
            }

            // Open joint positions data port
            const std::string jointPositionsPortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/jointPositions:o";
            if (!pImpl->jointPositionsDataPort.open(jointPositionsPortName)) {
                yError() << LogPrefix << "Failed to open port " << jointPositionsPortName;
                askToStop();
            }

            // Open joint velocities data port
            const std::string jointVelocitiesPortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/jointVelocities:o";
            if (!pImpl->jointVelocitiesDataPort.open(jointVelocitiesPortName)) {
                yError() << LogPrefix << "Failed to open port " << jointVelocitiesPortName;
                askToStop();
            }

            // Open CoM position data port
            const std::string comPositionPortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/comPosition:o";
            if (!pImpl->comPositionDataPort.open(comPositionPortName)) {
                yError() << LogPrefix << "Failed to open port " << comPositionPortName;
                askToStop();
            }

            // Open CoM velocity data port
            const std::string comVelocityPortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/comVelocity:o";
            if (!pImpl->comVelocityDataPort.open(comVelocityPortName)) {
                yError() << LogPrefix << "Failed to open port " << comVelocityPortName;
                askToStop();
            }

            // Open CoM acceleration in base frame data port
            const std::string comProperAccelerationInBaseFramePortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/comProperAccelerationInBaseFrame:o";
            if (!pImpl->comProperAccelerationInBaseFrameDataPort.open(comProperAccelerationInBaseFramePortName)) {
                yError() << LogPrefix << "Failed to open port " << comProperAccelerationInBaseFramePortName;
                askToStop();
            }

            // Open CoM acceleration in world frame data port
            const std::string comProperAccelerationInWorldFramePortName = "/" + pImpl->portPrefix + "/" + DeviceName + "/comProperAccelerationInWorldFrame:o";
            if (!pImpl->comProperAccelerationInWorldFrameDataPort.open(comProperAccelerationInWorldFramePortName)) {
                yError() << LogPrefix << "Failed to open port " << comProperAccelerationInWorldFramePortName;
                askToStop();
            }


        }


    }


    // =========================================================
    // Get data from different interfaces from attached devices
    // =========================================================

    // Get data from IHumanState interface of HumanStateProvider
    if (pImpl->isStateProviderDevice) {

        // Base Quantities
        pImpl->baseName = pImpl->iHumanState->getBaseName();
        pImpl->basePosition = pImpl->iHumanState->getBasePosition();
        pImpl->baseOrientation = pImpl->iHumanState->getBaseOrientation();
        pImpl->baseVelocity = pImpl->iHumanState->getBaseVelocity();

        // Joint Quantities
        pImpl->stateNumberOfJoints = pImpl->iHumanState->getNumberOfJoints();
        pImpl->stateJointNames = pImpl->iHumanState->getJointNames();
        pImpl->jointPositions = pImpl->iHumanState->getJointPositions();
        pImpl->jointVelocities = pImpl->iHumanState->getJointVelocities();

        // CoM Quantities
        pImpl->comPosition = pImpl->iHumanState->getCoMPosition();
        pImpl->comVelocity = pImpl->iHumanState->getCoMVelocity();
        pImpl->comProperAccInBaseFrame = pImpl->iHumanState->getCoMProperAccelerationExpressedInBaseFrame();
        pImpl->comProperAccInWorldFrame = pImpl->iHumanState->getCoMProperAccelerationExpressedInWorldFrame();

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

        pImpl->numberOfWrenchEstimatesSources = pImpl->iHumanWrenchEstimates->getNumberOfWrenchSources();
        pImpl->wrenchEstimatesSourceNames = pImpl->iHumanWrenchEstimates->getWrenchSourceNames();
        pImpl->wrenchEstimatesValues = pImpl->iHumanWrenchEstimates->getWrenches();

        // Get data from IHumanDynamics interface of HumanDynamicsEstimator
        pImpl->dynamicsNumberOfJoints = pImpl->iHumanDynamics->getNumberOfJoints();
        pImpl->dynamicsJointNames = pImpl->iHumanDynamics->getJointNames();
        pImpl->jointTorques = pImpl->iHumanDynamics->getJointTorques();

    }

    // Flip firstData flag false
    if (pImpl->firstData) {
        pImpl->firstData = false;
    }

    // ============================
    // Put human data to yarp ports
    // ============================

    if (pImpl->isStateProviderDevice) {

        // Prepare base pose data
        yarp::sig::Vector& basePoseYarpVector = pImpl->basePoseDataPort.prepare();
        std::array<double, 7> basePoseArray = {pImpl->basePosition[0],
                                               pImpl->basePosition[1],
                                               pImpl->basePosition[2],
                                               pImpl->baseOrientation[0],
                                               pImpl->baseOrientation[1],
                                               pImpl->baseOrientation[2],
                                               pImpl->baseOrientation[3]};


        std::vector<double> basePoseInputVector(basePoseArray.begin(), basePoseArray.end());

        pImpl->dataBuffersConversionHelper.setBuffer(basePoseYarpVector, basePoseInputVector);

        // Prepare base velocity data
        std::vector<double> baseVelocityInputVector(pImpl->baseVelocity.begin(), pImpl->baseVelocity.end());
        yarp::sig::Vector& baseVelocityYarpVector = pImpl->baseVelocityDataPort.prepare();
        pImpl->dataBuffersConversionHelper.setBuffer(baseVelocityYarpVector, baseVelocityInputVector);

        // Prepare stateJointNames
        yarp::os::Bottle& stateJointNamesYarpBottle = pImpl->stateJointNamesDataPort.prepare();
        pImpl->dataBuffersConversionHelper.setBuffer(stateJointNamesYarpBottle, pImpl->stateJointNames);

        // Prepare joint positions
        yarp::sig::Vector& jointPositionsYarpVector = pImpl->jointPositionsDataPort.prepare();
        pImpl->dataBuffersConversionHelper.setBuffer(jointPositionsYarpVector, pImpl->jointPositions);

        // Prepare joint velocities
        yarp::sig::Vector& jointVelocitiesYarpVector = pImpl->jointVelocitiesDataPort.prepare();
        pImpl->dataBuffersConversionHelper.setBuffer(jointVelocitiesYarpVector, pImpl->jointVelocities);

        // Preprare com position
        std::vector<double> comPositionInputVector(pImpl->comPosition.begin(), pImpl->comPosition.end());
        yarp::sig::Vector& comPositionYarpVector = pImpl->comPositionDataPort.prepare();
        pImpl->dataBuffersConversionHelper.setBuffer(comPositionYarpVector, comPositionInputVector);

        // Preprate com velocity
        std::vector<double> comVelocityInputVector(pImpl->comVelocity.begin(), pImpl->comVelocity.end());
        yarp::sig::Vector& comVelocityYarpVector = pImpl->comVelocityDataPort.prepare();
        pImpl->dataBuffersConversionHelper.setBuffer(comVelocityYarpVector, comVelocityInputVector);

        // Prepare com proper acceleration in base frame
        std::vector<double> comProperAccelerationInBaseFrameInputVector(pImpl->comProperAccInBaseFrame.begin(), pImpl->comProperAccInBaseFrame.end());
        yarp::sig::Vector& comProperAccelerationInBaseFrameYarpVector = pImpl->comProperAccelerationInBaseFrameDataPort.prepare();
        pImpl->dataBuffersConversionHelper.setBuffer(comProperAccelerationInBaseFrameYarpVector, comProperAccelerationInBaseFrameInputVector);


        // Prepare com proper acceleration in world frame
        std::vector<double> comProperAccelerationInWorldFrameInputVector(pImpl->comProperAccInWorldFrame.begin(), pImpl->comProperAccInWorldFrame.end());
        yarp::sig::Vector& comProperAccelerationInWorldFrameYarpVector = pImpl->comProperAccelerationInWorldFrameDataPort.prepare();
        pImpl->dataBuffersConversionHelper.setBuffer(comProperAccelerationInWorldFrameYarpVector, comProperAccelerationInWorldFrameInputVector);

        // Send data through yarp ports
        pImpl->basePoseDataPort.write(true);
        pImpl->baseVelocityDataPort.write(true);
        pImpl->stateJointNamesDataPort.write(true);
        pImpl->jointPositionsDataPort.write(true);
        pImpl->jointVelocitiesDataPort.write(true);
        pImpl->comPositionDataPort.write(true);
        pImpl->comVelocityDataPort.write(true);
        pImpl->comProperAccelerationInBaseFrameDataPort.write(true);
        pImpl->comProperAccelerationInWorldFrameDataPort.write(true);
    }



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
