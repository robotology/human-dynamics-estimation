/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENS_MVN_DRIVER_IMPL_H
#define XSENS_MVN_DRIVER_IMPL_H

#include "XSensMVNCalibrator.h"
#include "XSensMVNDriver.h"

#include <xsens/xmecallback.h>
#include <xsens/xmecontrol.h>
#include <xsens/xmelicense.h>

#include <atomic>
#include <string>
#include <vector>

namespace xsensmvn {
    class XSensMVNDriverImpl;
} // namespace xsensmvn

class xsensmvn::XSensMVNDriverImpl : public XmeCallback
{

private:
    struct XSensDataSample
    {
        long long relativeTime;
        long long absoluteTime;
        XmePose humanPose;
        XmeSuitSample suitData;
    };

    struct CalibrationInfo
    {
        std::string type;
        xsensmvn::CalibrationQuality quality;
    };

    struct SuitLabels
    {
        std::mutex labelsLock;
        std::vector<std::string> segmentNames;
        std::vector<std::string> sensorNames;
        std::vector<std::string> jointNames;
    };

    /* ---------- *
     *  Variables *
     * ---------- */

    // Status of the driver
    std::atomic<xsensmvn::DriverStatus> m_driverStatus;
    bool m_stopProcessor;

    // XSens-style data variables
    XSensDataSample m_lastDataSample;
    bool m_newSampleAvailable;

    // Link, Sensor, and Joint names lists
    SuitLabels m_suitLabels;

    // XSens object prointers
    std::unique_ptr<XmeLicense> m_license;
    std::unique_ptr<XmeControl> m_connection;

    // Data processor thread
    std::thread m_processor;

    // Condition variables and mutexs
    std::condition_variable m_processorVariable;
    std::condition_variable m_connectionVariable;

    mutable std::recursive_mutex m_objectMutex;
    mutable std::mutex m_outDataMutex;
    std::mutex m_connectionMutex;
    std::mutex m_processorMutex;

    /* ---------- *
     *  Functions *
     * ---------- */

    void processDataSamples();

    bool fillSegmentNames();
    bool fillSensorNames();
    bool fillJointNames();

public:
    /* ---------- *
     *  Variables *
     * ---------- */

    // Driver output data
    xsensmvn::DriverDataSample m_lastProcessedDataSample;

    // Calibrator object pointer
    std::unique_ptr<xsensmvn::XSensMVNCalibrator> m_calibrator;

    // Driver configuration
    const DriverConfiguration m_driverConfiguration;

    // Calibration Infos
    CalibrationInfo m_calibrationInfo;

    /* -------------------------- *
     *  Construtors / Destructors *
     * -------------------------- */
    XSensMVNDriverImpl(const xsensmvn::DriverConfiguration conf);
    XSensMVNDriverImpl(const XSensMVNDriverImpl&) = delete;
    XSensMVNDriverImpl& operator=(const XSensMVNDriverImpl&) = delete;

    ~XSensMVNDriverImpl() override;

    /* ---------- *
     *  Functions *
     * ---------- */

    bool configureAndConnect();

    bool startAcquisition();
    bool stopAcquisition();

    bool cleanAndClose();

    bool calibrate(const std::string calibrationType);

    xsensmvn::DriverStatus getDriverStatus() const { return m_driverStatus; }
    std::vector<std::string> getSuitLinkLabels();
    std::vector<std::string> getSuitSensorLabels();
    std::vector<std::string> getSuitJointLabels();

    /* -------------------- *
     *  XSens XME Callbacks *
     * -------------------- */

    // Hardware has been successfully disconnected
    void onHardwareDisconnected(XmeControl* dev) override;
    // Unspecified hardware error
    void onHardwareError(XmeControl* dev) override;
    // Hardware is ready after being detected
    void onHardwareReady(XmeControl* dev) override;
    // Called when device battery becomes lower than 10%
    void onLowBatteryLevel(XmeControl* dev) override;
    // Xme acquisition status callbacks
    void onPoseReady(XmeControl* dev) override; // A new data sample is available
};

#endif // XSENS_MVN_DRIVER_IMPL_H
