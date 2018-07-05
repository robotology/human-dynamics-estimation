/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENS_MVN_DRIVER_IMPL_H
#define XSENS_MVN_DRIVER_IMPL_H

#include "XSensLogger.h"
#include "XSensMVNCalibrator.h"

#include "XSensMVNDriver.h"

#include <xsens/xmecallback.h>
#include <xsens/xmecontrol.h>
#include <xsens/xmelicense.h>
#include <xsens/xmepose.h>
#include <xsens/xmesuitsample.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

// struct XmeControl;
// class XsensCallbackHandler;

namespace xsensmvn {
    // class XSensMVNCalibrator;
    class XSensMVNDriverImpl;
} // namespace xsensmvn

class xsensmvn::XSensMVNDriverImpl : public XmeCallback
{

private:
    // Structure to store single-sample data
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
        // xsensmvn::CalibrationQuality quality;
        bool appliedToEngine;
    };

    /* ---------- *
     *  Variables *
     * ---------- */

    // Driver Configuration
    const DriverConfiguration m_driverConfiguration;

    // Status of the driver
    std::atomic<xsensmvn::DriverStatus> m_driverStatus;

    // XSens-style data variables
    XSensDataSample m_lastDataSample;
    bool m_newSampleAvailable;

    // Driver output data
    xsensmvn::DriverDataSample m_lastProcessedDataSample;

    // XSens object prointers
    std::unique_ptr<XmeLicense> m_license;
    std::unique_ptr<XmeControl> m_connection;

    // Calibrator object pointer
    std::unique_ptr<xsensmvn::XSensMVNCalibrator> m_calibrator;

    // Data processor thread
    std::thread m_processor;

    // Variables for multithreading
    std::condition_variable m_processorVariable;
    std::condition_variable m_connectionVariable;

    std::mutex m_connectionMutex;
    std::mutex m_processorMutex;

    mutable std::recursive_mutex m_objectMutex;
    mutable std::mutex m_outDataMutex;

    // Status variables
    std::atomic_bool m_hardwareConnected;

    bool m_acquiring;
    bool m_stopProcessor;

    std::atomic<std::vector<std::string>> m_segmentNames;
    std::atomic<std::vector<std::string>> m_sensorNames;
    std::atomic<std::vector<std::string>> m_jointNames;

    /* ---------- *
     *  Functions *
     * ---------- */

    void processDataSamples();

    bool fillSegmentNames();
    bool fillSensorNames();
    bool fillJointNames();

    bool configureAndConnect();

public:
    /* -------------------------- *
     *  Construtors / Destructors *
     * -------------------------- */
    XSensMVNDriverImpl(const xsensmvn::DriverConfiguration conf);
    XSensMVNDriverImpl(const XSensMVNDriverImpl&) = delete;
    XSensMVNDriverImpl& operator=(const XSensMVNDriverImpl&) = delete;

    virtual ~XSensMVNDriverImpl() override { m_connection->addCallbackHandler(this); }

    /* -------------------- *
     *  XSens XME Callbacks *
     * -------------------- */

    // Hardware has been successfully disconnected
    virtual void onHardwareDisconnected(XmeControl* dev) override;
    // Unspecified hardware error
    virtual void onHardwareError(XmeControl* dev) override;
    // Hardware is ready after being detected
    virtual void onHardwareReady(XmeControl* dev) override;
    // Called when device battery becomes lower than 10%
    virtual void onLowBatteryLevel(XmeControl* dev) override;
    // Xme acquisition status callbacks
    virtual void onPoseReady(XmeControl* dev) override; // A new data sample is available
};

#endif // XSENS_MVN_DRIVER_IMPL_H
