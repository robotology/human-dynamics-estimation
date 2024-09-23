// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

    // Last applied calibration infos
    xsensmvn::CalibrationQuality m_achievedCalibrationQuality;
    std::string m_usedCalibrationType;

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
    bool getBodyDimension(const std::string bodyName, double& dim);

    // Minimum calibration quality considered to be satisfactory set/get
    bool setMinimumAcceptableCalibrationQuality(const xsensmvn::CalibrationQuality quality);
    xsensmvn::CalibrationQuality& getMinimumAcceptableCalibrationQuality();

    // Last calibration info get
    void getLastCalibrationInfo(std::string& type, CalibrationQuality quality);

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
