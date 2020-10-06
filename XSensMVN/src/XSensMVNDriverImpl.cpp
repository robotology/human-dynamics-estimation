/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "XSensMVNDriverImpl.h"

#include <chrono>
#include <experimental/filesystem>
#include <map>
#include <string>
#include <ctime>

#include <xme.h>

using namespace xsensmvn;

/* ---------- *
 *  Utilities *
 * ---------- */

/* -------------------------- *
 *  Construtors / Destructors *
 * -------------------------- */
XSensMVNDriverImpl::XSensMVNDriverImpl(
    const xsensmvn::DriverConfiguration& conf,
    std::shared_ptr<xsensmvn::DriverDataSample> dataSampleStorage,
    std::shared_ptr<std::mutex> dataStorageMutex)
    : m_driverStatus(DriverStatus::Disconnected)
    , m_stopProcessor(false)
    , m_newSampleAvailable(false)
    , m_license(nullptr)
    , m_connection(nullptr)
    , m_outDataMutex(dataStorageMutex)
    , m_lastProcessedDataSample(dataSampleStorage)
    , m_calibrator(nullptr)
    , m_driverConfiguration(conf)
    , m_calibrationInfo({})
{}

XSensMVNDriverImpl::~XSensMVNDriverImpl()
{
    if(m_driverConfiguration.saveMVNRecording)
    {
        // Save .mvn recording file
        m_connection->stopRecording();
        while(m_connection->status().isRecording() || m_connection->status().isFlushing()){
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
        m_connection->saveAndCloseFile();
    }
    
    m_connection->removeCallbackHandler(static_cast<XmeCallback*>(this));
}

/* ----------------- *
 *  Public Functions *
 * ----------------- */

// This method is used by the processor thread
// TODO: explain better what it does
void XSensMVNDriverImpl::processDataSamples()
{
    xsInfo << "Starting processing thread";
    while (true) {

        XSensDataSample lastSample;

        {
            // Processor lock scope
            // unique_lock is necessary to unlock the mutex manually
            std::unique_lock<std::mutex> processorLock(m_processorMutex);

            // Wait for being notified if new sample is available or if it is time to stop
            m_processorVariable.wait(processorLock);

            // Check if the thread has been notified by fini()
            if (m_stopProcessor) {
                // Time to stop
                xsInfo << "Closing sample processor thread";
                break;
            }

            // If not acquiring, release processor lock and go to the next cycle
            // It is possible to directly check the value of m_driverStatus since defined
            // atomic
            if (m_driverStatus != DriverStatus::Recording) {
                continue;
            }

            // Move locally the last data sample
            lastSample = std::move(m_lastDataSample);
            m_newSampleAvailable = false;
        }

        // Copy last retrieved data sample to the driver structure
        {
            std::lock_guard<std::mutex> copyLock(*m_outDataMutex);

            // Reset the structure keeping only the name
            m_lastProcessedDataSample->reset();

            // Fill timestamps structure : absolute, relative, systemClock(Unix time)
            // Copy absolute and relative XSens time
            m_lastProcessedDataSample->timestamps->absolute = lastSample.absoluteTime / 1000.0;
            m_lastProcessedDataSample->timestamps->relative =
                static_cast<double>(lastSample.relativeTime) / 1000.0;
            // TODO: is this system time representation what we really want?
            m_lastProcessedDataSample->timestamps->systemTime =
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    std::chrono::high_resolution_clock::now().time_since_epoch())
                    .count(); // From yarp/os/SystemClock.cpp

            if (m_driverConfiguration.dataStreamConfiguration.enableLinkData) {

                // If the link data vector is empty is the first run, so allocate the space
                if (m_lastProcessedDataSample->links.data.empty()) {
                    m_lastProcessedDataSample->links.data.resize(
                        lastSample.humanPose.m_segmentStates.size());
                }

                // Check if expected and actual sample dimensions match
                if (m_lastProcessedDataSample->links.data.size()
                    != lastSample.humanPose.m_segmentStates.size()) {
                    xsError << "The number of links do not match previous samples";
                    xsError << "Skipping sample";
                    continue;
                }

                unsigned segmentIx = 0;
                for (const auto& link : lastSample.humanPose.m_segmentStates) {

                    xsensmvn::LinkData newLink = {};

                    for (unsigned i = 0; i < 3; ++i) {
                        newLink.position.at(i) = link.m_position.at(i);
                        newLink.linearVelocity.at(i) = link.m_velocity.at(i);
                        newLink.linearAcceleration.at(i) = link.m_acceleration.at(i);
                        newLink.angularVelocity.at(i) = link.m_angularVelocity.at(i);
                        newLink.angularAcceleration.at(i) = link.m_angularAcceleration.at(i);
                    }
                    newLink.orientation = {{link.m_orientation.w(),
                                            link.m_orientation.x(),
                                            link.m_orientation.y(),
                                            link.m_orientation.z()}};

                    newLink.name = m_suitLabels.segmentNames.at(segmentIx);

                    m_lastProcessedDataSample->links.data[segmentIx] = newLink;
                    ++segmentIx;
                };
            }

            if (m_driverConfiguration.dataStreamConfiguration.enableSensorData) {

                // If the link data vector is empty is the first run, so allocate the space
                if (m_lastProcessedDataSample->sensors.data.empty()) {
                    m_lastProcessedDataSample->sensors.data.resize(
                        lastSample.suitData.sensorKinematics().size());
                }

                // Check if expected and actual sample dimensions match
                if (m_lastProcessedDataSample->sensors.data.size()
                    != lastSample.suitData.sensorKinematics().size()) {
                    xsError << "The number of sensors do not match previous samples";
                    xsError << "Skipping sample";
                    continue;
                }

                unsigned sensorIx = 0;
                for (const auto& sensor : lastSample.suitData.sensorKinematics()) {

                    SensorData newSensor{};

                    for (unsigned i = 0; i < 3; ++i) {
                        newSensor.position.at(i) = sensor.m_posG.at(i);
                        newSensor.freeBodyAcceleration.at(i) = sensor.m_accG.at(i);
                        newSensor.magneticField.at(i) = sensor.m_mag.at(i);
                    }
                    newSensor.orientation = {
                        {sensor.m_q.w(), sensor.m_q.x(), sensor.m_q.y(), sensor.m_q.z()}};

                    newSensor.name = m_suitLabels.sensorNames.at(sensorIx);

                    m_lastProcessedDataSample->sensors.data[sensorIx] = newSensor;
                    ++sensorIx;
                };
            }

            if (m_driverConfiguration.dataStreamConfiguration.enableJointData) {
                // Xsens store angles this way: dof are the rows, X, Y, Z the columns
                // They are computed using ZXY permutation using current frame formalism
                XmeEulerPermutation anglePermutation = XmeEulerPermutation::XEP_ZXY_YUp;
                XsMatrix angles = m_connection->jointAngles(
                    lastSample.humanPose, anglePermutation, XmePoseReference::XPR_AnatomicalPose);

                // If the link data vector is empty is the first run, so allocate the space
                if (m_lastProcessedDataSample->joints.data.empty()) {
                    m_lastProcessedDataSample->joints.data.resize(angles.rows());
                }

                // Check if expected and actual sample dimensions match
                if (m_lastProcessedDataSample->joints.data.size() != angles.rows()) {
                    xsError << "The number of joints do not match previous samples";
                    xsError << "Skipping sample";
                    continue;
                }

                // Loop through all the joints in the XSens model
                for (size_t jointIx = 0; jointIx < angles.rows(); ++jointIx) {
                    JointData newJoint{};

                    // Fill newly created joint with data coming from XSems
                    // Since the order of XSens matrix columns depends on the chosen permutation it
                    // is safer to copy them manually
                    switch (anglePermutation) {
                        case XmeEulerPermutation::XEP_ZXY_YUp:
                            newJoint.angles.at(0) = angles.value(jointIx, 1);
                            newJoint.angles.at(1) = angles.value(jointIx, 2);
                            newJoint.angles.at(2) = angles.value(jointIx, 0);
                            break;
                        case XmeEulerPermutation::XEP_XZY_YUp:
                            newJoint.angles.at(0) = angles.value(jointIx, 0);
                            newJoint.angles.at(1) = angles.value(jointIx, 2);
                            newJoint.angles.at(2) = angles.value(jointIx, 1);
                            break;
                    }

                    newJoint.name = m_suitLabels.jointNames.at(jointIx);

                    // Push the newly created joint in the joint vector of the last processed frame
                    m_lastProcessedDataSample->joints.data[jointIx] = newJoint;
                }
            }
        }
    }
    xsInfo << "Closing data processing thread";
}

bool XSensMVNDriverImpl::configureAndConnect()
{
    // This function is called only in the constructor. The mutex is not strictly necessary.
    // TODO: probably a non-recursive mutex would be enough.
    std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

    switch (m_driverStatus) {
        case DriverStatus::Disconnected:
        case DriverStatus::Unknown:
            // Continue the execution
            break;
        case DriverStatus::Scanning:
            xsError << "Driver is currently scanning. This state shouldn't be reached here.";
            return false;
        case DriverStatus::Connected:
        case DriverStatus::Calibrating:
        case DriverStatus::CalibratedAndReadyToRecord:
        case DriverStatus::Recording:
            xsError << "Device already connected";
            return false;
    }

    // Create a license instance
    m_license.reset(new XmeLicense());
    if (!m_license) {
        xsError << "Unable to create a valid license instance. Check hardware dongle.";
        return false;
    }

    std::experimental::filesystem::path tmpFolder =
        std::experimental::filesystem::temp_directory_path();
    xsInfo << "Temporary directory is " << tmpFolder.c_str() << std::endl;

    xmeSetPaths(m_driverConfiguration.licensePath.c_str(), "", tmpFolder.c_str(), true);

    // Create a connection instance
    m_connection.reset(XmeControl::construct());
    if (!m_connection) {
        xsError << "Could not open Xsens DLL. Check to use the correct DLL.";
        return false;
    }
    // Configure the connection to enable callbacks support
    m_connection->addCallbackHandler(static_cast<XmeCallback*>(this));

    //----------------------------------------
    // Configure the MVN motion capture system
    //----------------------------------------

    // Get the list of the supported suit configurations
    xsInfo << "--- Supported MVN suit configurations ---";
    XsStringArray suitConfs = m_connection->configurationList();
    for (const auto& xssc : suitConfs) {
        xsInfo << " - " << xssc.c_str();
    }
    xsInfo << "--------------------------------" << std::endl;

    // Check if the user-selected configuration is supported
    int suitConfIsValid = suitConfs.find(m_driverConfiguration.suitConfiguration.c_str(), false);
    if (suitConfIsValid == -1) {
        xsError << "Selected suit configuration: "
                << m_driverConfiguration.suitConfiguration.c_str() << "not supported";
        cleanAndClose();
        return false;
    }

    // Configure the system to use the selected MVN suit configuration
    m_connection->setConfiguration(m_driverConfiguration.suitConfiguration.c_str());
    xsInfo << "Using selected suit configuration:"
           << m_driverConfiguration.suitConfiguration.c_str() << std::endl;

    // Get the list of the supported suit configurations
    xsInfo << "--- Supported MVN acquisition scenarios ---";
    XsStringArray acqScenarios = m_connection->userScenarioList();
    for (const auto& xsas : acqScenarios) {
        xsInfo << " - " << xsas.c_str();
    }
    xsInfo << "--------------------------------" << std::endl;

    int acqScenarioIsValid =
        acqScenarios.find(m_driverConfiguration.acquisitionScenario.c_str(), false);
    if (acqScenarioIsValid == -1) {
        xsWarning << "Selected acquisition scenario: "
                  << m_driverConfiguration.acquisitionScenario.c_str() << "not supported";
        xsWarning << "Using DEFAULT scenario: " << m_connection->userScenario().c_str()
                  << std::endl;
    }
    else {
        m_connection->setUserScenario(m_driverConfiguration.suitConfiguration.c_str());
        xsInfo << "Using selected acquisition scenario:" << m_connection->userScenario().c_str()
               << std::endl;
    }

    //----------------------------------
    // Connect to the Xsens MVN hardware
    //----------------------------------

    // Scan for the suit until either the connection is successfull or the timeout is reached
    // Since this function is called asynchronously, it should be synchronized
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    {
        std::unique_lock<std::mutex> connectionLock(m_connectionMutex);
        m_driverStatus = DriverStatus::Scanning;
        m_connection->setScanMode(true);
        if (m_driverConfiguration.scanTimeout * 1000 > 0) {
            // Timeout enabled
            m_connectionVariable.wait_until(
                connectionLock,
                now + std::chrono::milliseconds(m_driverConfiguration.scanTimeout * 1000));
        }
        else {
            // Timout disabled, keep scannig until the system is found
            m_connectionVariable.wait(connectionLock);
        }
        m_connection->setScanMode(false);
    }

    // Double-check if Xsens MVN system is connected
    if (!m_connection->status().isConnected() && m_driverStatus != DriverStatus::Connected) {
        xsError << "Unable to connect to the XSens MVN suit";
        cleanAndClose();
        return false;
    }

    xsInfo << "Successfully connected to the XSens suit";

    //-----------------------------
    // Create a calibrator instance
    //-----------------------------

    m_calibrator.reset(new xsensmvn::XSensMVNCalibrator(
        *m_connection, m_driverConfiguration.minimumRequiredCalibrationQuality));
    if (!m_calibrator) {
        xsError << "Unable to create the calibrator";
        cleanAndClose();
        return false;
    }

    //-------------------------------------------
    // Configure subject-specific body dimensions
    //-------------------------------------------

    xsInfo << "--- Supported subject dimension parameters ---";
    // Get configurable body dimensions from XSens
    XsStringArray xsBodyDimensions = m_connection->bodyDimensionLabelList();
    if (xsBodyDimensions.empty()) {
        xsInfo << "Supported body dimension retrieved from device is empty";
        cleanAndClose();
        return false;
    }

    for (const auto& xsbd : xsBodyDimensions) {
        xsInfo << " - " << xsbd.c_str();
    }
    xsInfo << "--------------------------------" << std::endl;

    if (!m_driverConfiguration.bodyDimensions.empty()) {
        if (!m_calibrator->setBodyDimensions(m_driverConfiguration.bodyDimensions)) {
            xsError << "Unable to configure the system with the provided body dimensions";
            cleanAndClose();
            return false;
        }
    }

    // Retrieve sensor, segment, and joint names from the device
    if (!(fillSensorNames() && fillSegmentNames() && fillJointNames())) {
        xsError << "Failed to fill sensor, segment, and joint name lists";
        cleanAndClose();
        return false;
    };

    //-------------------------------------
    // Retrieve supported calibration types
    //-------------------------------------

    XsStringArray calibrationTypeSet = m_connection->calibrationLabelList();
    xsInfo << "--- Supported calibration types ---";
    for (const auto& cr : calibrationTypeSet) {
        xsInfo << " - " << cr.c_str();
    }
    xsInfo << "------------------------------------";

    // Get the default calibration routine name from config file
    if (calibrationTypeSet.find(m_driverConfiguration.defaultCalibrationType.c_str()) == -1) {
        xsError << "Provided default calibration type: "
                << m_driverConfiguration.defaultCalibrationType << " not supported";
        cleanAndClose();
        return false;
    }
    xsInfo << "Default calibration type set: " << m_driverConfiguration.defaultCalibrationType;

    //--------------------------------
    // Retrieve supported sample rates
    //--------------------------------

    xsInfo << "--- Supported acquisition frequencies [Hz] ---";
    XsIntArray supportedUpdateRates = m_connection->determineSupportedUpdateRates(
        m_connection->detectedDevices(),
        static_cast<int>(m_connection->sensorCountForConfiguration(m_connection->configuration())),
        false);
    for (const auto& ur : supportedUpdateRates) {
        xsInfo << " - " << ur << " [Hz]";
    }
    xsInfo << "------------------------------------";
    if (supportedUpdateRates.find(m_driverConfiguration.samplingRate) == -1) {
        xsWarning << "Requested sampling rate: " << m_driverConfiguration.samplingRate
                  << " not supported.";
        xsWarning << "Using default sampling rate: " << supportedUpdateRates.at(0) << " Hz";
        m_connection->setSampleRate(static_cast<double>(supportedUpdateRates.at(0)));
    }
    else {
        m_connection->setSampleRate(static_cast<double>(m_driverConfiguration.samplingRate));
    }
    // Sleep for at least (EMPIRICALLY RETRIEVED VALUE) to allow XSens to update its sampling rate
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    xsInfo << "Sampling rate set to: " << m_connection->sampleRate() << " Hz";

    //---------------------------------------
    // Create the datasample processor thread
    //---------------------------------------
    // Create a sample processor thread to free data processing from MVN callbacks
    // TODO: switch to async calls?? std::future? Is it performant enough create a thread
    //       every time data is received? Probably if no allocations are done this is ok.
    //       This would mean lock-free code :)
    m_stopProcessor = false;
    m_processor = std::thread(&XSensMVNDriverImpl::processDataSamples, this);

    return m_driverStatus == DriverStatus::Connected;
}

bool XSensMVNDriverImpl::startAcquisition()
{
    std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

    // Check the internal status and continue only if the driver is CalibratedAndReadyToRecord
    switch (m_driverStatus) {
        case DriverStatus::Disconnected:
        case DriverStatus::Unknown:
        case DriverStatus::Scanning:
            xsError << "Device not ready or not connected";
            return false;
        case DriverStatus::Calibrating:
            xsError << "Device is calibrating. Calibration must end in order to "
                    << "start the acquisition.";
            return false;
        case DriverStatus::CalibratedAndReadyToRecord:
            // Continue the execution
            break;
        case DriverStatus::Recording:
            xsError << "Device already recording";
            return false;
        case DriverStatus::Connected:
            // In theory the suit can stream without being calibrated but we do not support it
            xsError << "Device connected but not calibrated. "
                    << "You need to calibrate the device first.";
            return false;
    }

    // If ready to acquire, ask directly to the SDK if everything is ok
    if (!m_connection || !m_connection->status().isConnected()
        || m_connection->status().isOutOfRange()) {
        xsError << "Suit not connected or out of range";
        xsWarning << "The internal driver status does not match Xsens status";
        return false;
    }

    if (!m_calibrator || m_calibrator->isCalibrationInProgress()) {
        xsError << "Calibration in progress. Unable to start data acquisition";
        xsWarning << "The internal driver status does not match Xsens status";
        return false;
    }

    // Check if the acquisition is already in progress
    if (m_connection->realTimePoseMode()) {
        xsError << "Acquisition already in progress";
        xsWarning << "The internal driver status does not match Xsens status";
        return false;
    }

    // Start the acquisition
    m_connection->setRealTimePoseMode(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!m_connection->realTimePoseMode()) {
        xsError << "Failed to activate real time data flow";
        return false;
    }

    m_driverStatus = DriverStatus::Recording;
    xsInfo << "Acquisition successfully started";
    return true;
}

bool XSensMVNDriverImpl::stopAcquisition()
{
    std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

    // Check the internal status and continue only if the driver is CalibratedAndReadyToRecord
    switch (m_driverStatus) {
        case DriverStatus::Disconnected:
        case DriverStatus::Unknown:
        case DriverStatus::Scanning:
            xsError << "Device not ready or not connected";
            return false;
        case DriverStatus::Calibrating:
            xsError << "Calibration in progress. Unable to stop data acquisition";
            return false;
        case DriverStatus::CalibratedAndReadyToRecord:
        case DriverStatus::Connected:
            xsError << "Device not recording. Unable to stop data acquisition";
            return false;
        case DriverStatus::Recording:
            // Continue the execution
            break;
    }

    // If ready to acquire, ask directly to the SDK if everything is ok
    if (!m_connection || !m_connection->status().isConnected()
        || m_connection->status().isOutOfRange()) {
        xsError << "Suit not connected or out of range";
        xsWarning << "The internal driver status does not match Xsens status";
        return false;
    }

    if (!m_calibrator || m_calibrator->isCalibrationInProgress()) {
        xsError << "Calibration in progress. Unable to stop data acquisition";
        xsWarning << "The internal driver status does not match Xsens status";
        return false;
    }

    // Check if the acquisition is already in progress
    if (!m_connection->realTimePoseMode()) {
        xsError << "Acquisition not in progress";
        xsWarning << "The internal driver status does not match Xsens status";
        return false;
    }

    // Stop the acquisition
    m_connection->setRealTimePoseMode(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (m_connection->realTimePoseMode()) {
        xsError << "Failed to stop real time data flow";
        return false;
    }

    m_driverStatus = DriverStatus::CalibratedAndReadyToRecord;
    xsInfo << "Acquisition successfully stopped";
    return true;
}

bool XSensMVNDriverImpl::calibrate(const std::string calibrationType)
{
    std::string calibType = calibrationType;

    {
        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

        // Handle empty calibrationType parameter
        if (calibType.empty()) {
            if (m_driverConfiguration.defaultCalibrationType.empty()) {
                xsError << "Neither custom nor default calibration type set. Aborting";
                return false;
            }
            calibType = m_driverConfiguration.defaultCalibrationType;
        }

        // Check the internal status and continue only if the driver is CalibratedAndReadyToRecord
        switch (m_driverStatus) {
            case DriverStatus::Disconnected:
            case DriverStatus::Unknown:
            case DriverStatus::Scanning:
                xsError << "Device not ready or not connected";
                return false;
            case DriverStatus::Calibrating:
                xsError << "Calibration already in progress. Unable to calibrate";
                xsError << "Please wait for completion or abort the ongoing acquisition and retry";
                return false;
            case DriverStatus::CalibratedAndReadyToRecord:
                // TODO: it would be possible to store multiple calibrations but it is not yet
                // supported
                xsInfo << "The device has already been calibrated. Discarding the previous "
                          "calibration.";
                break;
            case DriverStatus::Connected:
                // Continue the execution
                break;
            case DriverStatus::Recording:
                xsError << "Acquisition in progress. Unable to calibrate";
                xsError << "Please stop the ongoing acquisition, then retry";
                return false;
        }

        // If ready to acquire, ask directly to the SDK if everything is ok
        if (!m_connection || !m_connection->status().isConnected()
            || m_connection->status().isOutOfRange()) {
            xsError << "Suit not connected or out of range";
            xsWarning << "The internal driver status does not match Xsens status";
            return false;
        }

        if (!m_calibrator || m_calibrator->isCalibrationInProgress()) {
            xsError << "Calibration in progress. Unable to start another calibration";
            xsWarning << "The internal driver status does not match Xsens status";
            return false;
        }
    }

    // Set the status to calibrating
    m_driverStatus = DriverStatus::Calibrating;

    // Ask the calibrator to perform the calibration
    bool calibrationCompleted = m_calibrator->calibrateWithType(calibType);

    // Activate data flow
    m_driverStatus = DriverStatus::Connected;
    if (calibrationCompleted) {
        // Record the success of the calibration
        m_driverStatus = DriverStatus::CalibratedAndReadyToRecord;

        if(m_driverConfiguration.saveMVNRecording)
        {
            //Start recording .mvn file
            time_t rawtime;
            struct tm * timeinfo;
            char buffer[80];

            time(&rawtime);
            timeinfo = localtime(&rawtime);

            strftime(buffer, sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
            std::string time_string(buffer);

            // Replace separators with underscore
            std::replace(time_string.begin(), time_string.end(), ' ', '_'); //Replace space with underscore
            std::replace(time_string.begin(), time_string.end(), '-', '_'); //Replace - with underscore
            std::replace(time_string.begin(), time_string.end(), ':', '_'); //Replace : with underscore
        
            const XsString mvnFileName("recording_" + time_string);

            xsInfo << "Recording to " << mvnFileName.toStdString() << ".mvn file";

            m_connection->createMvnFile(mvnFileName);
            m_connection->startRecording();
        }

        m_calibrator->getLastCalibrationInfo(m_calibrationInfo.type, m_calibrationInfo.quality);
    }

    return calibrationCompleted;
}

std::vector<std::string> XSensMVNDriverImpl::getSuitLinkLabels()
{
    std::lock_guard<std::mutex> guard(m_suitLabels.labelsLock);
    return m_suitLabels.segmentNames;
}

std::vector<std::string> XSensMVNDriverImpl::getSuitSensorLabels()
{
    std::lock_guard<std::mutex> guard(m_suitLabels.labelsLock);
    return m_suitLabels.sensorNames;
}

std::vector<std::string> XSensMVNDriverImpl::getSuitJointLabels()
{
    std::lock_guard<std::mutex> guard(m_suitLabels.labelsLock);
    return m_suitLabels.jointNames;
}

/* ------------------------------------------ *
 *  Public XSens XME Callback Implementations *
 * ------------------------------------------ */

void XSensMVNDriverImpl::onHardwareDisconnected(XmeControl* /*dev*/)
{
    xsInfo << "Xsens MVN hardware disconnected.";
    m_driverStatus = DriverStatus::Disconnected;
    m_connectionVariable.notify_one();
}

void XSensMVNDriverImpl::onHardwareError(XmeControl* dev)
{
    XmeStatus status = dev->status();
    xsWarning << "Hardware error received.";
    xsWarning << "Suit connected: " << (status.isConnected() ? "YES" : "NO");
    xsWarning << "Scanning for XSens MVN suit: " << (status.isScanning() ? "YES" : "NO");
}

void XSensMVNDriverImpl::onHardwareReady(XmeControl* /*dev*/)
{
    xsInfo << "Xsens MVN hardware connected successfully. Ready to start";
    m_driverStatus = DriverStatus::Connected;
    m_connectionVariable.notify_one();
}

void XSensMVNDriverImpl::onLowBatteryLevel(XmeControl* /*dev*/)
{
    xsWarning << "Low battery level. Recharge it as soon as possible.";
}

void XSensMVNDriverImpl::onPoseReady(XmeControl* dev)
{
    // Copy last available sample to temporary structure to avoid waiting for the lock
    XSensDataSample newSample{dev->pose(XME_LAST_AVAILABLE_FRAME).m_relativeTime,
                              dev->pose(XME_LAST_AVAILABLE_FRAME).m_absoluteTime,
                              dev->pose(XME_LAST_AVAILABLE_FRAME),
                              dev->suitSample(XME_LAST_AVAILABLE_FRAME)};

    // Copy acquired sample to the processing queue and notify the processor thread
    std::lock_guard<std::mutex> poseCopyGuard(m_processorMutex);
    m_lastDataSample = std::move(newSample);
    m_newSampleAvailable = true;
    m_processorVariable.notify_one();
}

/* ------------------ *
 *  Private Functions *
 * ------------------ */

bool XSensMVNDriverImpl::cleanAndClose()
{
    std::lock_guard<std::recursive_mutex> connectionGuard(m_objectMutex);

    // Kill calibrator thread
    m_calibrator.reset();

    // Stop the acquisition, disconnect the hardware, and destroy the device connector
    if (m_connection) {
        if (m_driverStatus == DriverStatus::Recording) {
            stopAcquisition();
            m_driverStatus = DriverStatus::CalibratedAndReadyToRecord;
        }
        m_connection->disconnectHardware();
        {
            std::unique_lock<std::mutex> connectionLock(m_connectionMutex);
            m_connectionVariable.wait(connectionLock);

            // Clear the callback handlers list
            m_connection->clearCallbackHandlers();
            m_connection->destruct();
        }
    }

    m_driverStatus = DriverStatus::Disconnected;

    // Close frame processor thread
    {
        std::unique_lock<std::mutex> lock(m_processorMutex);
        m_stopProcessor = true;

        // Notify the processor thread it's time to end
        m_processorVariable.notify_one();
    }

    // The processor lock is now realeased, join the thread to wait for its end
    if (m_processor.joinable()) {
        m_processor.join();
    }

    if (m_license) {
        xmeTerminate();
    }

    return true;
}

bool XSensMVNDriverImpl::fillSegmentNames()
{
    // Lock the mutex to prevent other threads changing the segment list
    //    std::lock_guard<std::recursive_mutex> connectionGuard(m_objectMutex);

    if (!m_connection || m_connection->segmentCount() == 0) {
        xsWarning << "Unable to retrieve segment names from XSens or empty segment list";
        return false;
    }

    // Get segment count and allocate memory for output vector
    int segmentCount = m_connection->segmentCount();
    {
        std::lock_guard<std::mutex> labelGuard(m_suitLabels.labelsLock);
        m_suitLabels.segmentNames.reserve(static_cast<unsigned>(segmentCount));
        // Segment ID is 1-based, vector is 0-based, segmentID = index + 1
        for (int ix = 0; ix < segmentCount; ++ix) {
            m_suitLabels.segmentNames.push_back(m_connection->segmentName(ix + 1).toStdString());
        }
    }
    return true;
}

bool XSensMVNDriverImpl::fillSensorNames()
{
    // Lock the mutex to prevent other threads changing the segment list
    //    std::lock_guard<std::recursive_mutex> connectionGuard(m_objectMutex);

    if (!m_connection || m_connection->sensorCount() == 0) {
        xsWarning << "Unable to retrieve sensor names from XSens or empty sensor list";
        return false;
    }

    unsigned sensorCount = static_cast<unsigned>(m_connection->sensorCount());
    // Since sensor names are not directly available, get the suit sensors status
    XmeDeviceStatusArray sensorsStatus = m_connection->status().suitStatus().m_sensors;

    // Double check if sensorCount and sensorStatus.size() are equals
    if (sensorCount != sensorsStatus.size()) {
        xsWarning << "Mismatch in sensor count and sensor states dimensions";
        return false;
    }

    {
        std::lock_guard<std::mutex> labelGuard(m_suitLabels.labelsLock);
        m_suitLabels.sensorNames.reserve(sensorCount);
        for (unsigned ix = 0; ix < sensorCount; ++ix) {
            // Get the name of the segment associated to the ix-th sensor (everything is
            // 0-based)
            m_suitLabels.sensorNames.push_back(
                m_connection->segmentName(sensorsStatus[ix].m_segmentId).toStdString());
        }
    }
    return true;
}

bool XSensMVNDriverImpl::fillJointNames()
{
    // Lock the mutex to prevent other threads changing the segment list
    //    std::lock_guard<std::recursive_mutex> connectionGuard(m_objectMutex);

    if (!m_connection) {
        xsWarning << "Unable to retrieve joint names";
        return false;
    }

    {
        std::lock_guard<std::mutex> labelGuard(m_suitLabels.labelsLock);
        m_suitLabels.jointNames.reserve(m_connection->joints().size());

        for (const auto& joint : m_connection->joints()) {
            m_suitLabels.jointNames.push_back(joint.name().toStdString());
        }
    }
    return true;
}
