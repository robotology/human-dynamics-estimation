// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "XSensMVNDriverImpl.h"

#include <chrono>
#include <experimental/filesystem>
#include <string>

#include <xme.h>

using namespace xsensmvn;

/* ---------- *
 *  Utilities *
 * ---------- */

/* -------------------------- *
 *  Construtors / Destructors *
 * -------------------------- */
XSensMVNDriverImpl::XSensMVNDriverImpl(const DriverConfiguration conf)
    : m_driverConfiguration(conf)
    , m_newSampleAvailable(false)
    , m_license(nullptr)
    , m_connection(nullptr)
    , m_hardwareConnected(false)
    , m_stopProcessor(false)
    , m_segmentNames({})
    , m_sensorNames({})
    , m_jointNames({})
{

    configureAndConnect();
}

// XSensMVNDriverImpl::~XSensMVNDriverImpl() {} // namespace xsensmvn

/* ------------------------------------------ *
 *  Public XSens XME Callback Implementations *
 * ------------------------------------------ */

void XSensMVNDriverImpl::onHardwareDisconnected(XmeControl*)
{
    xsInfo << "Xsens MVN hardware disconnected.";
    m_driverStatus = DriverStatus::Disconnected;

    std::lock_guard<std::mutex> poseCopyGuard(m_connectionMutex);
    m_hardwareConnected = false;
    m_connectionVariable.notify_one();
}

void XSensMVNDriverImpl::onHardwareError(XmeControl* dev)
{
    XmeStatus status = dev->status();
    xsWarning << "Hardware error received.";
    xsWarning << "Suit connected: %s" << (status.isConnected() ? "YES" : "NO");
    xsWarning << "Scanning for XSens MVN suit: " << (status.isScanning() ? "YES" : "NO");
}

void XSensMVNDriverImpl::onHardwareReady(XmeControl* dev)
{
    xsInfo << "Xsens MVN hardware connected successfully. Ready to start";
    m_driverStatus = DriverStatus::Connected;

    std::lock_guard<std::mutex> poseCopyGuard(m_connectionMutex);
    m_hardwareConnected = true;
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

void XSensMVNDriverImpl::processDataSamples()
{
    xsInfo << "Starting processing thread";
    while (true) {

        // Processor lock scope
        std::unique_lock<std::mutex> processorLock(m_processorMutex);

        // Wait for being notified if new sample is available or if it is time to stop
        m_processorVariable.wait(processorLock,
                                 [&]() { return m_stopProcessor || m_newSampleAvailable; });

        // Check if the thread has been notified by fini()
        if (m_stopProcessor) {
            // Time to stop
            xsInfo << "Closing sample processor thread";
            break;
        }

        // If not acquiring, release processor lock and go to the next cycle
        // It is possible to directly check the value of m_driverStatus since defined
        // atomic
        if (m_driverStatus == DriverStatus::Recording) {
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
            std::lock_guard<std::mutex> copyLock(m_outDataMutex);

            // TODO: fill suit name

            // Copy absolute and relative XSens time
            m_lastProcessedDataSample.absoluteTime = lastSample.absoluteTime / 1000.0;
            m_lastProcessedDataSample.relativeTime = lastSample.relativeTime / 1000.0;

            if (m_driverConfiguration.dataStreamConfiguration.enableLinkData) {
                // TODO: check if expected and actual sample dimensions match

                m_lastProcessedDataSample.links.reserve(
                    lastSample.humanPose.m_segmentStates.size());

                int segmentIx = 0;
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
                    newLink.name = m_segmentNames.at(segmentIx);
                    m_lastProcessedDataSample.links.push_back(newLink);
                };
            }

            if (m_driverConfiguration.dataStreamConfiguration.enableSensorData) {
                m_lastProcessedDataSample.sensors.reserve(
                    lastSample.suitData.sensorKinematics().size());

                int sensorIx = 0;
                for (const auto& sensor : lastSample.suitData.sensorKinematics()) {

                    SensorData newSensor{};

                    for (unsigned i = 0; i < 3; ++i) {
                        newSensor.position.at(i) = sensor.m_posG.at(i);
                        newSensor.freeBodyAcceleration.at(i) = sensor.m_accG.at(i);
                        newSensor.magneticField.at(i) = sensor.m_mag.at(i);
                    }
                    newSensor.orientation = {
                        {sensor.m_q.w(), sensor.m_q.x(), sensor.m_q.y(), sensor.m_q.z()}};

                    newSensor.name = m_sensorNames.at(sensorIx);
                    m_lastProcessedDataSample.sensors.push_back(newSensor);
                };
            }

            if (m_driverConfiguration.dataStreamConfiguration.enableJointData) {
                // TODO: add support for joint angles
                xsWarning << "Joint angles not supported yet.";
            }
        }
        xsInfo << "Frame processed";
    }
    xsInfo << "Closing data processing thread";
}

/* ------------------ *
 *  Private Functions *
 * ------------------ */

bool XSensMVNDriverImpl::configureAndConnect()
{

    std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

    // Check if the device is already connected
    if (m_connection) {
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
    m_segmentNames.reserve(static_cast<unsigned>(segmentCount));

    // Segment ID is 1-based, vector is 0-based, segmentID = index + 1
    for (int ix = 0; ix < segmentCount; ++ix) {
        m_segmentNames.push_back(m_connection->segmentName(ix + 1).toStdString());
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

    m_sensorNames.reserve(sensorCount);
    for (unsigned ix = 0; ix < sensorCount; ++ix) {
        // Get the name of the segment associated to the ix-th sensor (everything is
        // 0-based)
        m_sensorNames.push_back(
            m_connection->segmentName(sensorsStatus[ix].m_segmentId).toStdString());
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

    m_jointNames.reserve(m_connection->joints().size());

    for (const auto& joint : m_connection->joints()) {
        m_jointNames.push_back(joint.name().toStdString());
    }

    return true;
}
