/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

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
