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

#include <xme.h>

using namespace xsensmvn;

/* ---------- *
 *  Utilities *
 * ---------- */

/* -------------------------- *
 *  Construtors / Destructors *
 * -------------------------- */
XSensMVNDriverImpl::XSensMVNDriverImpl(
    const xsensmvn::DriverConfiguration conf,
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
    m_connection->removeCallbackHandler(static_cast<XmeCallback*>(this));
}

/* ----------------- *
 *  Public Functions *
 * ----------------- */

void XSensMVNDriverImpl::processDataSamples()
{
    xsInfo << "Starting processing thread";
    while (true) {

        // Processor lock scope
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
            processorLock.unlock();
            continue;
        }

        // Move locally the last data sample
        XSensDataSample lastSample = std::move(m_lastDataSample);
        m_newSampleAvailable = false;

        // Unlock processor scope
        processorLock.unlock();

        // Copy last retrieved data sample to the driver structure
        {
            std::lock_guard<std::mutex> copyLock(*m_outDataMutex);

            // TODO: fill suit name

            // Fill timestamps structure : absolute, relative, systemClock(Unix time)
            m_lastProcessedDataSample->timestamps->absolute = lastSample.absoluteTime / 1000.0;
            m_lastProcessedDataSample->timestamps->relative =
                static_cast<double>(lastSample.relativeTime) / 1000.0;
            m_lastProcessedDataSample->timestamps->systemTime =
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    std::chrono::high_resolution_clock::now().time_since_epoch())
                    .count(); // From yarp/os/SystemClock.cpp

            if (m_driverConfiguration.dataStreamConfiguration.enableLinkData) {
                // TODO: check if expected and actual sample dimensions match
                // Copy absolute and relative XSens time

                m_lastProcessedDataSample->links.data.reserve(
                    lastSample.humanPose.m_segmentStates.size());

                unsigned segmentIx = 0;
                for (const auto& link : lastSample.humanPose.m_segmentStates) {

                    xsensmvn::LinkData newLink{};

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
                    {
                        std::lock_guard<std::mutex> labelGuard(m_suitLabels.labelsLock);
                        newLink.name = m_suitLabels.segmentNames.at(segmentIx);
                    }
                    m_lastProcessedDataSample->links.data.push_back(newLink);
                };
            }

            if (m_driverConfiguration.dataStreamConfiguration.enableSensorData) {
                m_lastProcessedDataSample->sensors.data.reserve(
                    lastSample.suitData.sensorKinematics().size());

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

                    {
                        std::lock_guard<std::mutex> labelGuard(m_suitLabels.labelsLock);
                        newSensor.name = m_suitLabels.sensorNames.at(sensorIx);
                    }

                    m_lastProcessedDataSample->sensors.data.push_back(newSensor);
                };
            }

            if (m_driverConfiguration.dataStreamConfiguration.enableJointData) {
                // TODO: add support for joint angles
                xsWarning << "Joint angles not supported yet.";
            }
        }
    }
    xsInfo << "Closing data processing thread";
}

bool XSensMVNDriverImpl::configureAndConnect()
{

    std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

    // Check if the device is already connected
    if (m_connection
        && (m_driverStatus != DriverStatus::Disconnected && m_driverStatus != DriverStatus::Unknown
            && m_driverStatus != DriverStatus::Scanning)) {
        xsError << "Device already connected";
        return false;
    }

    // Create a license istance
    m_license.reset(new XmeLicense());
    if (!m_license) {
        xsError << "Unable to create a valid license istance. Check hardware dongle.";
        return false;
    }

    std::experimental::filesystem::path tmpFoder =
        std::experimental::filesystem::temp_directory_path();
    xsInfo << "Temporary directory is " << tmpFoder.c_str();

    xmeSetPaths(m_driverConfiguration.licensePath.c_str(), "", tmpFoder.c_str(), true);

    // Create a connection istance
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
           << m_driverConfiguration.suitConfiguration.c_str();

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
        xsWarning << "Using DEFAULT scenario: " << m_connection->userScenario().c_str();
    }
    else {
        m_connection->setUserScenario(m_driverConfiguration.suitConfiguration.c_str());
        xsInfo << "Using selected acquisition scenario:" << m_connection->userScenario().c_str();
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

    //----------------------------
    // Create a calibrator istance
    //----------------------------

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

    //---------------------------------------
    // Create the datasample processor thread
    //---------------------------------------
    // Create a sample processor thread to free data processing from MVN callbacks
    m_stopProcessor = false;
    m_processor = std::thread(&XSensMVNDriverImpl::processDataSamples, this);

    return m_driverStatus == DriverStatus::Connected;
}

bool XSensMVNDriverImpl::startAcquisition()
{
    std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

    // Check if the device is available, is connected and is inside WiFi range
    if (!m_connection || !m_connection->status().isConnected()
        || m_connection->status().isOutOfRange() || m_driverStatus == DriverStatus::Disconnected
        || m_driverStatus == DriverStatus::Unknown || m_driverStatus == DriverStatus::Scanning) {
        xsError << "Device not connected or out of range. Unable to start data acquisition";
        return false;
    }

    // Check that there are no calibration in progress
    if (!m_calibrator && m_calibrator->isCalibrationInProgress()
        && m_driverStatus == DriverStatus::Calibrating) {
        xsError << "Calibration in progress. Unable to start data acquisition";
        return false;
    }

    // Check if the acquisition is alredy in progress
    if (m_connection->realTimePoseMode() && m_driverStatus == DriverStatus::Recording) {
        xsWarning << "Acquisition already in progress";
        return false;
    }

    // Here device is connected, not calibrating and not already aquiring, last check if it is
    // calibrated and ready to start Start acquisition
    if (m_driverStatus != DriverStatus::CalibratedAndReadyToRecord) {
        xsError << "Device is not calibrated. Calibrate before recording";
        return false;
    }

    m_driverStatus = DriverStatus::Recording;
    m_connection->setRealTimePoseMode(true);
    xsInfo << "Acquisition successfully started";
    return true;
}

bool XSensMVNDriverImpl::stopAcquisition()
{
    std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

    // Check if the device is available, is connected and is inside WiFi range
    if (!m_connection || !m_connection->status().isConnected()
        || m_connection->status().isOutOfRange() || m_driverStatus == DriverStatus::Disconnected
        || m_driverStatus == DriverStatus::Unknown || m_driverStatus == DriverStatus::Scanning) {
        xsError << "Device not connected or out of range. Unable to stop data acquisition";
        return false;
    }

    // Check that there are no calibration in progress
    if (!m_calibrator && m_calibrator->isCalibrationInProgress()
        && m_driverStatus == DriverStatus::Calibrating) {
        xsError << "Calibration in progress. Unable to stop data acquisition";
        return false;
    }

    if (m_driverStatus == DriverStatus::CalibratedAndReadyToRecord) {
        xsError << "Device not recording. Unable to stop data acquisition";
        return false;
    }

    m_driverStatus = DriverStatus::CalibratedAndReadyToRecord;
    m_connection->setRealTimePoseMode(false);
    xsInfo << "Acquisition successfully stopped";
    return true;
}

bool XSensMVNDriverImpl::calibrate(const std::string calibrationType)
{
    std::string calibType = calibrationType;
    {
        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

        // Check device and calibrator availability
        if (!m_connection || !m_connection->status().isConnected()
            || m_connection->status().isOutOfRange() || m_driverStatus == DriverStatus::Disconnected
            || m_driverStatus != DriverStatus::Connected || m_driverStatus == DriverStatus::Unknown
            || m_driverStatus == DriverStatus::Scanning) {
            xsError << "Device not connected or out of range. Unable to calibrate";
            return false;
        }

        // Check if there is an acquisition in progress
        if (m_driverStatus == DriverStatus::Recording) {
            xsError << "Acquisition in progress. Unable to calibrate";
            xsError << "Please stop the ongoing acquisition, then retry";
            return false;
        }

        // Check if there is an acquisition in progress
        if (m_driverStatus == DriverStatus::Calibrating) {
            xsError << "Calibration already in progress. Unable to calibrate";
            xsError << "Please wait for completion or abort the ongoing acquisition and retry";
            return false;
        }

        // Handle empty calibrationType parameter
        if (calibType.empty()) {
            if (m_driverConfiguration.defaultCalibrationType.empty()) {
                xsError << "Neither custom nor default calibration type set. Aborting";
                return false;
            }
            calibType = m_driverConfiguration.defaultCalibrationType;
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

void XSensMVNDriverImpl::onHardwareDisconnected(XmeControl* dev)
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

void XSensMVNDriverImpl::onHardwareReady(XmeControl* dev)
{
    xsInfo << "Xsens MVN hardware connected successfully. Ready to start";
    m_driverStatus = DriverStatus::Connected;
    m_connectionVariable.notify_one();
}

void XSensMVNDriverImpl::onLowBatteryLevel(XmeControl* dev)
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
    // Check for existence and remove calibrator thread
    if (m_calibrator) {
        m_calibrator->~XSensMVNCalibrator();
    }

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
    std::lock_guard<std::recursive_mutex> connectionGuard(m_objectMutex);

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
    std::lock_guard<std::recursive_mutex> connectionGuard(m_objectMutex);

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
    std::lock_guard<std::recursive_mutex> connectionGuard(m_objectMutex);

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
