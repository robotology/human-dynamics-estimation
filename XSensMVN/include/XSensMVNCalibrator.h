/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENS_MVN_CALIBRATOR_H
#define XSENS_MVN_CALIBRATOR_H

#include "XSensCalibrationQualities.h"
#include "XSensLogger.h"

#include <xsens/xmecallback.h>
#include <xsens/xmecontrol.h>

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <vector>

namespace xsensmvn {
    class XSensMVNCalibrator;
} // namespace xsensmvn

class xsensmvn::XSensMVNCalibrator : public XmeCallback
{
private:
    /* ---------- *
     *  Variables *
     * ---------- */

    // Reference to the XmeControl object owned from the driver thread
    XmeControl& m_suitsConnector;

    // Minimum quality required for applying calibration
    xsensmvn::CalibrationQuality m_minimumAccaptableQuality;

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
                       const xsensmvn::CalibrationQuality minAcceptableQuality =
                           xsensmvn::CalibrationQuality::ACCEPTABLE);
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
    bool setMinimumAcceptableCalibrationQuality(const xsensmvn::CalibrationQuality quality);
    xsensmvn::CalibrationQuality& getMinimumAcceptableCalibrationQuality();

    // Status get
    bool isCalibrationInProgress();

    /* -------------------- *
     *  XSens XME Callbacks *
     * -------------------- */
    virtual void onCalibrationAborted(XmeControl* dev);
    virtual void onCalibrationComplete(XmeControl* dev);
    virtual void onCalibrationProcessed(XmeControl* dev);
};

#endif // XSENS_MVN_CALIBRATOR_H
