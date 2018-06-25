/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENS_MVN_CALIBRATOR_H
#define XSENSMVNCALIBRATOR_H

#include "XSensCalibrationQualities.h"
#include "XSensLogger.h"

#include <xsens/xmecallback.h>
#include <xsens/xmecontrol.h>

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <vector>

namespace xsens {
    class XSensMVNCalibrator;
} // namespace xsens

class xsens::XSensMVNCalibrator : public XmeCallback
{
private:
    /* ---------- *
     *  Variables *
     * ---------- */

    // Reference to the XmeControl object owned from the driver thread
    XmeControl& m_suitsConnector;

    // Minimum quality required for applying calibration
    xsens::CalibrationQuality m_minimumAccaptableQuality;

    // Atomic variable for internal state machine
    std::atomic_bool m_calibrationAborted;
    std::atomic_bool m_calibrationInProgress;
    std::atomic_bool m_calibrationProcessed;
    std::atomic_bool m_operationCompleted;

    /* ---------- *
     *  Functions *
     * ---------- */

    void cleanup();

public:
    /* -------------------------- *
     *  Construtors / Destructors *
     * -------------------------- */

    XSensMVNCalibrator(XmeControl& connector,
                       const std::map<std::string, double>& bodyDimensions = {},
                       const xsens::CalibrationQuality minAcceptableQuality =
                           xsens::CalibrationQuality::ACCEPTABLE);
    virtual ~XSensMVNCalibrator();

    /* ---------- *
     *  Functions *
     * ---------- */

    // Calibrator controls
    bool calibrateWithType(std::string calibrationType);
    bool abortCalibration();

    // Body-dimensions set/get
    bool setBodyDimensions(const std::map<std::string, double>& bodyDimensions);
    bool getBodyDimensions(std::map<std::string, double>& dimensions) const;

    // Minimum calibration quality considered to be satisfactory set/get
    bool setMinimumAcceptableCalibrationQuality(const xsens::CalibrationQuality quality);
    xsens::CalibrationQuality& getMinimumAcceptableCalibrationQuality();

    // Status get
    bool isCalibrationInProgress();

    // XSens XME Callbacks
    virtual void onCalibrationAborted(XmeControl* dev);
    virtual void onCalibrationComplete(XmeControl* dev);
    virtual void onCalibrationProcessed(XmeControl* dev);
};

#endif // XSENS_MVN_CALIBRATOR_H
