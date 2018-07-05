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
#include <string>

#include <xme.h>

// void yarp::dev::XsensMVN::XsensMVNPrivate::resizeVectorToOuterAndInnerSize(
//    std::vector<yarp::sig::Vector>& vector,
//    unsigned outerSize,
//    unsigned innerSize)
//{
//    size_t oldSize = vector.size();
//    vector.resize(outerSize);
//    for (unsigned i = oldSize; i < outerSize; ++i) {
//        vector[i].resize(innerSize, 0.0);
//    }
//}
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
// namespace xsensmvn
//    bool yarp::dev::XsensMVN::XsensMVNPrivate::init(yarp::os::Searchable& config)
//    {
//        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//        // Check if the device is already connected
//        if (m_connection) {
//            yError("Device already connected. Unable to configure");
//            return false;
//        }

//        // Create a license istance
//        m_license.reset(new XmeLicense());
//        if (!m_license) {
//            yError("Unable to create a valid license istance. Check hardware dongle.");
//            return false;
//        }

//        //---------------------------------------------
//        // Retrieve paths and configure xme to use them
//        //---------------------------------------------
//        // The location of the other files (xmedef, *.mvnc) can be configured at runtime by
//        calling
//        // xmeSetPaths. This expects in this order:
//        // - the path to runtime dependencies (xmedef.xsb, and .mvnc files),
//        // - the path to (props.xsb - probably not useful),
//        // - the path where the file xme.log will be stored (a temporary directory)
//        // - bool: overwrite previous logs
//        // Note: If any of these strings are left empty (i.e. "") the default value will be
//        used

//        // Retrieve temporary folder
//        std::experimental::filesystem::path tmpFoder =
//            std::experimental::filesystem::temp_directory_path();
//        yInfo("Temporary directory, retrieved with std::experimental, is %s",
//        tmpFoder.c_str());

//        // Read from conf file the rundepsFolder
//        yarp::os::ConstString rundepsFolder =
//            config
//                .check("xsens-rundeps-dir",
//                       yarp::os::Value(""),
//                       "Path to XSens MVN SDK runtime dependencies")
//                .asString();
//        yInfo("Reading runtime dependencies from %s", rundepsFolder.c_str());

//        // Configure xme to use the specified folders
//        xmeSetPaths(rundepsFolder.c_str(), "", tmpFoder.c_str(), true);

//        // Create a connection istance
//        m_connection.reset(XmeControl::construct());
//        if (!m_connection) {
//            yError("Could not open Xsens DLL. Check to use the correct DLL.");
//            return false;
//        }
//        // Configure the connection to enable callbacks support
//        m_connection->addCallbackHandler(this);

//        //----------------------------------------
//        // Configure the MVN motion capture system
//        //----------------------------------------

//        // Get the list of the supported suit configurations
//        yInfo("--- Supported MVN suit configurations ---");
//        XsStringArray suitConfs = m_connection->configurationList();
//        for (XsStringArray::const_iterator it = suitConfs.begin(); it != suitConfs.end();
//        ++it) {
//            yInfo(" - %s", it->c_str());
//        }
//        yInfo("--------------------------------");

//        // Get the user-selected suit configuration
//        yarp::os::ConstString suitConf =
//            config.check("suit-config", yarp::os::Value(""), "MVN suit configuration to use")
//                .asString();

//        // Check if the user-selected configuration is supported
//        bool confIsSupported = false;
//        for (const XsString& conf : suitConfs) {
//            if (conf == suitConf.c_str()) {
//                confIsSupported = true;
//                break;
//            }
//        }
//        if (!confIsSupported) {
//            yError("Selected suit configuration %s not supported", suitConf.c_str());
//            fini();
//            return false;
//        }

//        // Configure the system to use the selected MVN suit configuration
//        m_connection->setConfiguration(suitConf.c_str());
//        yInfo("Using suit configuration: %s", suitConf.c_str());

//        // TODO: add option to select the available MVN scenario

//        // Get the user-selected absolute reference system name
//        // Information used ONLY at YARP level, is just an alias for the output data reference
//        frame
//        // info
//        yarp::os::ConstString referenceFrame = config
//                                                   .check("reference-frame-alias",
//                                                          yarp::os::Value("xsens_world"),
//                                                          "Absolute reference frame alias")
//                                                   .asString();

//        //----------------------------------
//        // Connect to the Xsens MVN hardware
//        //----------------------------------

//        // Get the timeout [ms] for connecting with the XSens MVN hardware
//        int scanTimeout =
//            config.check("scan-timeout", yarp::os::Value(10), "Scan timeout [s]. -1 to
//            disable.")
//                .asInt()
//            * 1000;

//        // Scan for the suit until either the connection is successfull or the timeout is
//        reached
//        // Since this function is called asynchronously, it should be synchronized
//        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
//        {
//            std::unique_lock<std::mutex> connectionLock(m_connectionMutex);
//            m_hardwareConnected = false;
//            m_connection->setScanMode(true);
//            if (scanTimeout > 0) {
//                // Timeout enabled
//                m_connectionVariable.wait_until(connectionLock,
//                                                now + std::chrono::milliseconds(scanTimeout),
//                                                [&]() { return m_hardwareConnected; });
//            }
//            else {
//                // Timout disabled, keep scannig until the system is found
//                m_connectionVariable.wait(connectionLock, [&]() { return m_hardwareConnected;
//                });
//            }
//            m_connection->setScanMode(false);
//        }

//        // Double-check if Xsens MVN system is connected
//        if (!m_connection->status().isConnected()) {
//            yError("Unable to connect to the XSens MVN suit");
//            fini();
//            return false;
//        }

//        yInfo("XSens suit connected");

//        //----------------------------
//        // Create a calibrator istance
//        //----------------------------

//        m_calibrator.reset(new xsens::XsensMVNCalibrator(*m_connection));
//        if (!m_calibrator) {
//            yError("Unable to create the calibrator");
//            fini();
//            return false;
//        }
//        m_calibrator->addDelegate(*this);

//        //-------------------------------------------
//        // Configure subject-specific body dimensions
//        //-------------------------------------------

//        yInfo("Configuring subject-specific body dimensions");

//        // Get configurable body dimensions from XSens
//        XsStringArray bodyDimensionSet = m_connection->bodyDimensionLabelList();
//        if (bodyDimensionSet.empty()) {
//            yError("Unknown error occurred in retrieving supported body dimension from
//            device."); fini(); return false;
//        }

//        // Get subject-specific body dimensions from the configuration file and push them to
//        // bodyDimensionMap
//        yarp::os::Bottle subjectBodyDimensionSet =
//            config.findGroup("body-dimensions", "Subject-specific body dimensions");
//        std::map<std::string, double> bodyDimensionsMap;

//        yInfo("--- Configurable body dimensions supported by XSens MVN : user-provided value
//        [m] "
//              "---");
//        for (const auto& bd : bodyDimensionSet) {
//            yarp::os::Value dimension = subjectBodyDimensionSet.find(bd.c_str());
//            if (dimension.isNull() || !dimension.isDouble()) {
//                yInfo(" - %s : Not Provided", bd.c_str());
//                continue;
//            }
//            double dimValue = dimension.asDouble();
//            yInfo(" - %s : %lf [m]", bd.c_str(), dimValue);
//            bodyDimensionsMap.insert(
//                std::map<std::string, double>::value_type(bd.c_str(), dimValue));
//        }
//        yInfo("------------------------------------------------------------------------------------"
//              "--");

//        // Provide the bodyDimensionMap to the calibrator and (indirectly) to MVN Engine
//        if (!bodyDimensionsMap.empty()) {
//            if (!setBodyDimensions(bodyDimensionsMap)) {
//                yError("Unable to configure the system with the provided body dimensions");
//                fini();
//                return false;
//            }
//        }

//        //---------------------------------------------
//        // Retrieve model segment and suit sensor lists
//        //---------------------------------------------

//        const std::vector<yarp::experimental::dev::FrameReference> segmentNameList =
//            getSegmentInfos();

//        yInfo("--- XSens body segments --- [XSensSegmentID] - ReferenceFrame - Name ---");
//        // XSensSegmentID is 1-based, segmentNamesVector is 0-based
//        // segmentID = index + 1
//        for (unsigned ix = 0; ix < segmentNameList.size(); ++ix) {
//            yInfo(" - [%d] - %s - %s",
//                  ix + 1,
//                  segmentNameList[ix].frameReference.c_str(),
//                  segmentNameList[ix].frameName.c_str());
//        }
//        yInfo("------------------------------------------------------------------------");

//        // TODO: get sensors list

//        //-------------------------------------
//        // Retrieve supported calibration types
//        //-------------------------------------

//        XsStringArray calibrationTypeSet = m_connection->calibrationLabelList();
//        yInfo("--- Supported calibration types ---");
//        for (const auto& cr : calibrationTypeSet) {
//            yInfo(" - %s", cr.c_str());
//        }
//        yInfo("------------------------------------");

//        // Get the default calibration routine name from config file
//        yarp::os::ConstString defaultCalibrationType = config
//                                                           .check("default-calibration-type",
//                                                                  yarp::os::Value("Npose"),
//                                                                  "Default calibration type")
//                                                           .asString();

//        // Copy the value from yarp::os::ConstString to the std::string
//        m_defaultCalibrationType.assign(defaultCalibrationType.c_str(),
//                                        defaultCalibrationType.length());

//        yInfo("Default calibration type: %s", m_defaultCalibrationType.c_str());

//        //---------------------------
//        // Initialize data structures
//        //---------------------------
//        // Initialize size of vectors (model is set before calibration)
//        resizeVectorToOuterAndInnerSize(m_lastSegmentPosesRead, m_connection->segmentCount(),
//        7); resizeVectorToOuterAndInnerSize(
//            m_lastSegmentVelocitiesRead, m_connection->segmentCount(), 6);
//        resizeVectorToOuterAndInnerSize(
//            m_lastSegmentAccelerationsRead, m_connection->segmentCount(), 6);

//        resizeVectorToOuterAndInnerSize(m_lastSensorPositionsRead,
//        m_connection->sensorCount(), 3); resizeVectorToOuterAndInnerSize(
//            m_lastSensorOrientationsRead, m_connection->sensorCount(), 4);
//        resizeVectorToOuterAndInnerSize(
//            m_lastSensorFreeAccelerationsRead, m_connection->sensorCount(), 3);

//        //---------------------------------------
//        // Create the datasample processor thread
//        //---------------------------------------
//        // Create a sample processor thread to free data processing from MVN callbacks
//        m_stopProcessor = false;
//        m_processor = std::thread(&yarp::dev::XsensMVN::XsensMVNPrivate::processNewFrame,
//        this);

//        return m_hardwareConnected;
//    }

//    bool yarp::dev::XsensMVN::XsensMVNPrivate::fini()
//    {
//        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//        // Check for existence and remove calibrator thread
//        if (m_calibrator) {
//            m_calibrator->removeDelegate(*this);
//        }

//        // Stop the acquisition, disconnect the hardware, and destroy the device connector
//        if (m_connection) {
//            stopAcquisition();
//            m_connection->disconnectHardware();
//            {
//                std::unique_lock<std::mutex> connectionLock(m_connectionMutex);
//                m_connectionVariable.wait(connectionLock,
//                                          [&]() { return
//                                          !m_connection->status().isConnected(); });

//                // Clear the callback handlers list
//                m_connection->clearCallbackHandlers();
//                m_connection->destruct();
//            }
//        }

//        // Close frame processor thread
//        {
//            std::unique_lock<std::mutex> lock(m_processorMutex);
//            m_stopProcessor = true;

//            // Notify the processor thread it's time to end
//            m_processorVariable.notify_one();
//        }

//        // The processor lock is now realeased, join the thread to wait for its end
//        if (m_processor.joinable()) {
//            m_processor.join();
//        }

//        if (m_license) {
//            xmeTerminate();
//        }

//        return true;
//    }

//    const std::vector<yarp::experimental::dev::FrameReference>
//    yarp::dev::XsensMVN::XsensMVNPrivate::getSegmentInfos() const
//    {
//        // Initialize empty output vector
//        std::vector<yarp::experimental::dev::FrameReference> segments{};

//        // Lock the mutex to prevent other threads changing the segment list
//        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//        if (!m_connection || m_connection->segmentCount() < 0) {
//            yWarning("Unable to retrieve segment names from XSens or empty segment list");
//            return segments;
//        }

//        // Get segment count and allocate memory for output vector
//        unsigned segmentCount = static_cast<unsigned>(m_connection->segmentCount());
//        segments.reserve(segmentCount);

//        // Segment ID is 1-based, vector is 0-based, segmentID = index + 1
//        for (unsigned ix = 0; ix < segmentCount; ++ix) {
//            yarp::experimental::dev::FrameReference frameInfo = {
//                m_referenceFrame, m_connection->segmentName(ix + 1).c_str()};
//            segments.push_back(frameInfo);
//        }

//        return segments;
//    }

//    const std::vector<yarp::experimental::dev::FrameReference>
//    yarp::dev::XsensMVN::XsensMVNPrivate::getSensorInfos() const
//    {
//        // Initialize empty output vector
//        std::vector<yarp::experimental::dev::FrameReference> sensors{};

//        // Lock the mutex to prevent other threads changing the segment list
//        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//        if (!m_connection || m_connection->sensorCount() < 0) {
//            yWarning("Unable to retrieve sensor list from XSens or empty sensor list");
//            return sensors;
//        }
//        // Get sensor count
//        unsigned sensorCount = static_cast<unsigned>(m_connection->sensorCount());

//        // Since sensor names are not directly available, get the suit sensors status
//        XmeDeviceStatusArray sensorsStatus = m_connection->status().suitStatus().m_sensors;

//        // Double check if sensorCount and sensorStatus.size() are equals
//        if (sensorCount != static_cast<unsigned>(sensorsStatus.size())) {
//            yWarning("Mismatch in sensor count and sensor states dimensions");
//            return sensors;
//        }

//        // allocate memory for output vector
//        sensors.reserve(sensorCount);

//        for (unsigned ix = 0; ix < sensorCount; ++ix) {
//            // Get the name of the segment associated to the ix-th sensor (everything is
//            0-based) XsString sensorInSegmentName =
//            m_connection->segmentName(sensorsStatus[ix].m_segmentId);
//            // Imu quantities in MVN 2018 are expressed w.r.t. global frame
//            yarp::experimental::dev::FrameReference frameInfo = {m_referenceFrame,
//                                                                 sensorInSegmentName.c_str()};
//            sensors.push_back(frameInfo);
//        }
//        return sensors;
//    }

//    bool yarp::dev::XsensMVN::XsensMVNPrivate::startAcquisition()
//    {
//        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//        // Check if the device is available, is connected and is inside WiFi range
//        if (!m_connection || !m_connection->status().isConnected()
//            || m_connection->status().isOutOfRange()) {
//            yError("Device not connected or out of range. Unable to start data acquisition");
//            return false;
//        }

//        // Check that there are no calibration in progress
//        if (m_calibrator && m_calibrator->isCalibrationInProgress()) {
//            yError("Calibration in progress. Unable to start data acquisition");
//            return false;
//        }

//        // TODO: Check if the device is calibrated, otherwise acquisition cannot start

//        // Check if the acquisition is alredy in progress
//        if (m_connection->realTimePoseMode() && m_acquiring
//            && m_driverStatus == yarp::experimental::dev::IFrameProviderStatusOK) {
//            yWarning("Acquisition already in progress");
//            return false;
//        }

//        // Start acquisition
//        m_acquiring = true;
//        m_driverStatus = yarp::experimental::dev::IFrameProviderStatusOK;
//        m_connection->setRealTimePoseMode(true);
//        yInfo("Acquisition successfully started");
//        return true;
//    }

//    bool yarp::dev::XsensMVN::XsensMVNPrivate::stopAcquisition()
//    {
//        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//        // Check if the device is available, is connected and is inside WiFi range
//        if (!m_connection || !m_connection->status().isConnected()
//            || m_connection->status().isOutOfRange()) {
//            yError("Device not connected or out of range. Unable to stop data acquisition");
//            return false;
//        }

//        // Stop acquisition
//        m_connection->setRealTimePoseMode(false);
//        m_driverStatus = yarp::experimental::dev::IFrameProviderStatusNoData;
//        m_acquiring = false;
//        yInfo("Acquisition successfully stopped");
//        return true;
//    }

//    bool yarp::dev::XsensMVN::XsensMVNPrivate::setBodyDimensions(
//        const std::map<std::string, double>& dimensions)
//    {
//        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//        // Check if device and calibrator objects are available
//        if (!m_connection || !m_calibrator) {
//            yError("Device or calibrator not available. Unable to set body dimensions");
//            return false;
//        }

//        // Ask the calibrator to set the specified body dimensions
//        return m_calibrator->setBodyDimensions(dimensions);
//    }

//    // Read fron the device the estimated body dimensions according to the scaling done by
//    XSens
//    // after every call to setBodyDimension
//    bool yarp::dev::XsensMVN::XsensMVNPrivate::getBodyDimensions(
//        std::map<std::string, double>& dimensions) const
//    {
//        std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//        // Check if device and calibrator objects are available
//        if (!m_connection || !m_calibrator) {
//            yError("Device or calibrator not available. Unable to set body dimensions");
//            return false;
//        }
//        return m_calibrator->getBodyDimensions(dimensions);
//    }

//    // Perform XSens calibration using the specified calibrationType
//    bool yarp::dev::XsensMVN::XsensMVNPrivate::calibrateWithType(const std::string&
//    calibrationType)
//    {
//        std::string tmpCalibType = calibrationType;
//        {
//            std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//            // Check device and calibrator availability
//            if (!m_connection || !m_calibrator) {
//                yError("Device or calibrator not available. Unable to start calibration
//                routine"); return false;
//            }

//            // Check if there is an acquisition in progress
//            if (m_acquiring) {
//                yError("Acquisition in progress. Unable to start calibration routine.");
//                yError("First stop current acquisition, then retry");
//                return false;
//            }

//            // Handle empty calibrationType parameter
//            if (tmpCalibType.empty()) {
//                if (m_defaultCalibrationType.empty()) {
//                    yError("Neither custom nor default calibration type set. Aborting");
//                    return false;
//                }
//                tmpCalibType = m_defaultCalibrationType;
//            }
//        }

//        // Activate data flow
//        m_driverStatus = yarp::experimental::dev::IFrameProviderStatusOK;

//        // Ask the calibrator to perform the calibration
//        bool calibrationCompleted = m_calibrator->calibrateWithType(tmpCalibType);

//        // Deactivate data flow
//        m_driverStatus = yarp::experimental::dev::IFrameProviderStatusNoData;

//        if (calibrationCompleted) {
//            m_calibrationType = tmpCalibType;
//        }

//        return calibrationCompleted;
//    }

//    // TODO: Check what is the abort calibration flow and arrange stuff accordingly
//    bool yarp::dev::XsensMVN::XsensMVNPrivate::abortCalibration()
//    {
//        // I don't know if this makes sense, but how can I abort the calibration
//        // if I have to take the same lock of the calibrate function?
//        {
//            std::lock_guard<std::recursive_mutex> globalGuard(m_objectMutex);

//            // Check device and calibrator availability
//            if (!m_connection || !m_calibrator) {
//                yError("Device or calibrator not available. Unable to abort calibration
//                routine"); return false;
//            }
//        }
//        m_driverStatus = yarp::experimental::dev::IFrameProviderStatusNoData;
//        m_calibrator->abortCalibration();
//        return true;
//    }

//    void yarp::dev::XsensMVN::XsensMVNPrivate::processNewFrame()
//    {
//        yDebug("Starting sample processor thread");
//        while (true) {

//            // Processor lock scope
//            std::unique_lock<std::mutex> processorLock(m_processorMutex);

//            // Wait for being notified if new sample is available or if it is time to stop
//            m_processorVariable.wait(processorLock,
//                                     [&]() { return m_stopProcessor || m_lastSampleAvailable;
//                                     });

//            // Check if the thread has been notified by fini()
//            if (m_stopProcessor) {
//                // Time to stop
//                yDebug("Closing sample processor thread");
//                break;
//            }

//            // Move locally the last data sample
//            SampleData lastSample = std::move(m_lastSampleData);
//            m_lastSampleAvailable = false;

//            // Unlock processor scope
//            processorLock.unlock();

//            // Global lock scope
//            {
//                std::lock_guard<std::recursive_mutex> globalLock(m_objectMutex);
//                if (!m_acquiring) {
//                    continue;
//                }
//            }

//            // Process the new data sample
//            {
//                std::lock_guard<std::mutex> readLock(m_dataMutex);

//                // Check if expected and actual sensor and segment counts match
//                if (m_lastSegmentPosesRead.size() !=
//                lastSample.humanPose.m_segmentStates.size()
//                    || m_lastSensorOrientationsRead.size()
//                           != lastSample.suitData.sensorKinematics().size()) {
//                    m_driverStatus = yarp::experimental::dev::IFrameProviderStatusError;
//                    yWarning("Sample Processor Thread: Error in data processing, size
//                    mismatch. "
//                             "Skipping current sample");
//                    continue;
//                }

//                // TODO: add option to select which timestamp to use
//                int64_t xsAbsoluteTime = lastSample.humanPose.m_absoluteTime;
//                double time = xsAbsoluteTime / 1000.0;

//                m_lastTimestamp = yarp::os::Stamp(lastSample.humanPose.m_frameNumber, time);

//                for (unsigned sx = 0; sx < lastSample.humanPose.m_segmentStates.size(); ++sx)
//                {
//                    const XmeSegmentState& segmentData =
//                        lastSample.humanPose.m_segmentStates.at(sx);
//                    yarp::sig::Vector& segmentPosition = m_lastSegmentPosesRead.at(sx);
//                    yarp::sig::Vector& segmentVelocity = m_lastSegmentVelocitiesRead.at(sx);
//                    yarp::sig::Vector& segmentAcceleration =
//                    m_lastSegmentAccelerationsRead.at(sx);

//                    for (unsigned i = 0; i < 3; ++i) {
//                        // linear part
//                        segmentPosition(i) = segmentData.m_position.at(i);
//                        segmentVelocity(i) = segmentData.m_velocity.at(i);
//                        segmentAcceleration(i) = segmentData.m_acceleration.at(i);
//                        // angular part for velocity and acceleration
//                        segmentVelocity(3 + i) = segmentData.m_angularVelocity.at(i);
//                        segmentAcceleration(3 + i) = segmentData.m_angularAcceleration.at(i);
//                    }
//                    // Do the quaternion explicitly to avoid issues in format
//                    segmentPosition(3) = segmentData.m_orientation.w();
//                    segmentPosition(4) = segmentData.m_orientation.x();
//                    segmentPosition(5) = segmentData.m_orientation.y();
//                    segmentPosition(6) = segmentData.m_orientation.z();
//                }

//                for (unsigned sx = 0; sx < lastSample.suitData.sensorKinematics().size();
//                ++sx) {
//                    const XmeSensorKinematics& sensorData =
//                        lastSample.suitData.sensorKinematics().at(sx);
//                    yarp::sig::Vector& sensorPosition = m_lastSensorPositionsRead.at(sx);
//                    yarp::sig::Vector& sensorOrientation =
//                    m_lastSensorOrientationsRead.at(sx); yarp::sig::Vector&
//                    sensorFreeAcceleration =
//                        m_lastSensorFreeAccelerationsRead.at(sx);

//                    for (unsigned i = 0; i < 3; ++i) {
//                        sensorPosition(i) = sensorData.m_posG.at(i);
//                        sensorFreeAcceleration(i) = sensorData.m_accG.at(i);
//                    }
//                    sensorOrientation(0) = sensorData.m_q.w();
//                    sensorOrientation(1) = sensorData.m_q.x();
//                    sensorOrientation(2) = sensorData.m_q.y();
//                    sensorOrientation(3) = sensorData.m_q.z();
//                }

//                m_driverStatus = yarp::experimental::dev::IFrameProviderStatusOK;
//            }
//        }
//        yDebug("Sample processor thread closed");
//    }

//    yarp::experimental::dev::IFrameProviderStatus
//    yarp::dev::XsensMVN::XsensMVNPrivate::getLastSegmentReadTimestamp(yarp::os::Stamp&
//    timestamp)
//    {
//        std::lock_guard<std::mutex> readLock(m_dataMutex);
//        timestamp = m_lastTimestamp;
//        return m_driverStatus;
//    }

//    yarp::experimental::dev::IIMUFrameProviderStatus
//    yarp::dev::XsensMVN::XsensMVNPrivate::getLastSensorReadTimestamp(yarp::os::Stamp&
//    timestamp)
//    {
//        std::lock_guard<std::mutex> readLock(m_dataMutex);
//        timestamp = m_lastTimestamp;
//        return
//        yarp::experimental::dev::IIMUFrameProviderStatus(static_cast<int>(m_driverStatus));
//    }

//    yarp::experimental::dev::IFrameProviderStatus
//    yarp::dev::XsensMVN::XsensMVNPrivate::getLastSegmentInformation(
//        yarp::os::Stamp& timestamp,
//        std::vector<yarp::sig::Vector>& lastPoses,
//        std::vector<yarp::sig::Vector>& lastVelocities,
//        std::vector<yarp::sig::Vector>& lastAccelerations)
//    {
//        // These also ensure all the sizes are the same
//        if (lastPoses.size() < m_lastSegmentPosesRead.size()) {
//            // This will cause an allocation
//            resizeVectorToOuterAndInnerSize(lastPoses, m_lastSegmentPosesRead.size(), 7);
//        }
//        if (lastVelocities.size() < m_lastSegmentPosesRead.size()) {
//            // This will cause an allocation
//            resizeVectorToOuterAndInnerSize(lastVelocities, m_lastSegmentPosesRead.size(), 6);
//        }
//        if (lastAccelerations.size() < m_lastSegmentPosesRead.size()) {
//            // This will cause an allocation
//            resizeVectorToOuterAndInnerSize(lastAccelerations, m_lastSegmentPosesRead.size(),
//            6);
//        }

//        {
//            std::lock_guard<std::mutex> readLock(m_dataMutex);

//            if (m_acquiring) {
//                // we should receive data
//                double now = yarp::os::Time::now();
//                if ((now - m_lastTimestamp.getTime()) > 1.0) {
//                    m_driverStatus = yarp::experimental::dev::IFrameProviderStatusTimeout;
//                }
//                // yInfo("Reading data with time %lf at YARP Time %lf",
//                // m_lastSegmentTimestamp.getTime(), now);
//                /*else {
//                    m_driverStatus = yarp::experimental::dev::IFrameProviderStatusOK;
//                }*/
//            }
//            // get anyway data out
//            timestamp = m_lastTimestamp;
//            for (unsigned i = 0; i < m_lastSegmentPosesRead.size(); ++i) {
//                lastPoses[i] = m_lastSegmentPosesRead[i];
//                lastVelocities[i] = m_lastSegmentVelocitiesRead[i];
//                lastAccelerations[i] = m_lastSegmentAccelerationsRead[i];
//            }
//        }

//        return m_driverStatus;
//    }

//    yarp::experimental::dev::IIMUFrameProviderStatus
//    yarp::dev::XsensMVN::XsensMVNPrivate::getLastSuitInformations(
//        yarp::os::Stamp& timestamp,
//        std::vector<yarp::sig::Vector>& lastOrientations,
//        std::vector<yarp::sig::Vector>& lastPositions,
//        std::vector<yarp::sig::Vector>& lastFreeAccelerations)
//    {
//        // These also ensure all the sizes are the same
//        if (lastOrientations.size() < m_lastSensorOrientationsRead.size()) {
//            // This will cause an allocation
//            resizeVectorToOuterAndInnerSize(
//                lastOrientations, m_lastSensorOrientationsRead.size(), 4);
//        }
//        if (lastPositions.size() < m_lastSensorPositionsRead.size()) {
//            // This will cause an allocation
//            resizeVectorToOuterAndInnerSize(lastPositions, m_lastSensorPositionsRead.size(),
//            3);
//        }
//        if (lastFreeAccelerations.size() < m_lastSensorFreeAccelerationsRead.size()) {
//            // This will cause an allocation
//            resizeVectorToOuterAndInnerSize(
//                lastFreeAccelerations, m_lastSensorFreeAccelerationsRead.size(), 3);
//        }

//        {
//            std::lock_guard<std::mutex> readLock(m_dataMutex);

//            if (m_acquiring) {
//                // we should receive data
//                double now = yarp::os::Time::now();
//                if ((now - m_lastTimestamp.getTime()) > 1.0) {
//                    m_driverStatus = yarp::experimental::dev::IFrameProviderStatusTimeout;
//                }
//                // yInfo("Reading data with time %lf at YARP Time %lf",
//                // m_lastSegmentTimestamp.getTime(), now);
//                /*else {
//                m_driverStatus = yarp::experimental::dev::IFrameProviderStatusOK;
//                }*/
//            }
//            // get anyway data out
//            timestamp = m_lastTimestamp;
//            for (unsigned i = 0; i < m_lastSensorOrientationsRead.size(); ++i) {
//                lastOrientations[i] = m_lastSensorOrientationsRead[i];
//                lastPositions[i] = m_lastSensorPositionsRead[i];
//                lastFreeAccelerations[i] = m_lastSensorFreeAccelerationsRead[i];
//            }
//        }
//        return
//        yarp::experimental::dev::IIMUFrameProviderStatus(static_cast<int>(m_driverStatus));
//    }

//    /*-----------------
//      Callback funtions
//      -----------------*/

//    void yarp::dev::XsensMVN::XsensMVNPrivate::onHardwareReady(XmeControl* dev)
//    {
//        yInfo("Xsens MVN hardware connected successfully. Ready to start");
//        std::unique_lock<std::mutex> lock(m_connectionMutex);
//        m_hardwareConnected = true;
//        m_connectionVariable.notify_one();
//    }

//    void yarp::dev::XsensMVN::XsensMVNPrivate::onHardwareError(XmeControl* dev)
//    {
//        XmeStatus status = dev->status();
//        yWarning("Hardware error received.");
//        yWarning("Suit connected: %s", (status.isConnected() ? "YES" : "NO"));
//        yWarning("Scanning for XSens MVN suit: %s", (status.isScanning() ? "YES" : "NO"));
//    }

//    void yarp::dev::XsensMVN::XsensMVNPrivate::onPoseReady(XmeControl* dev)
//    {
//        // Copy last available sample to temporary structure to avoid waiting for the lock
//        SampleData newSample;
//        newSample.humanPose = dev->pose(XME_LAST_AVAILABLE_FRAME);
//        newSample.suitData = dev->suitSample(XME_LAST_AVAILABLE_FRAME);

//        std::unique_lock<std::mutex> lock(m_processorMutex);
//        // Copy acquired sample to the processing queue and notify the processor thread
//        m_lastSampleData = std::move(newSample);
//        m_lastSampleAvailable = true;
//        m_processorVariable.notify_one();
//    }

//    void yarp::dev::XsensMVN::XsensMVNPrivate::onHardwareDisconnected(XmeControl*)
//    {
//        yInfo("Xsens MVN hardware disconnected.");
//        std::unique_lock<std::mutex> lock(m_connectionMutex);
//        m_hardwareConnected = false;
//        m_connectionVariable.notify_one();
//    }

//    // TODO: not sure what it does, to be investigated
//    void yarp::dev::XsensMVN::XsensMVNPrivate::calibratorHasReceivedNewCalibrationPose(
//        const xsens::XsensMVNCalibrator* const sender,
//        std::vector<yarp::sig::Vector> newPose)
//    {
//        // only manage my own calibrator
//        // if (sender != m_calibrator)
//        //    return;
//        std::lock_guard<std::mutex> readLock(m_dataMutex);

//        for (unsigned index = 0; index < newPose.size(); ++index) {
//            m_lastSegmentPosesRead[index] = newPose[index];
//            m_lastSegmentVelocitiesRead[index].zero();
//            m_lastSegmentAccelerationsRead[index].zero();
//        }
//    }
