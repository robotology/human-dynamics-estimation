// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IXSENS_MVN_CONTROL_H
#define IXSENS_MVN_CONTROL_H

#include <map>
#include <string>

namespace xsensmvn {
    class IXsensMVNControl;
}

class xsensmvn::IXsensMVNControl
{
public:
    /* -------------------------- *
     *  Construtors / Destructors *
     * -------------------------- */

    IXsensMVNControl() = default;
    virtual ~IXsensMVNControl() = default;

    /* ---------- *
     *  Functions *
     * ---------- */

    // Body dimensions set/get
    virtual bool setBodyDimensions(const std::map<std::string, double>& dimensions) = 0;
    virtual bool getBodyDimensions(std::map<std::string, double>& dimensions) const = 0;
    virtual bool getBodyDimension(const std::string bodyName, double& dimension) const = 0;

    // Calibration methods
    virtual bool calibrate(const std::string& calibrationType = {}) = 0;
    virtual bool abortCalibration() = 0;

    // Acquisition methods
    virtual bool startAcquisition() = 0;
    virtual bool stopAcquisition() = 0;
};

#endif // IXSENS_MVN_CONTROL_H
