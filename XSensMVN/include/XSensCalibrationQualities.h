// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XSENS_CALIBRATION_QUALITIES_H
#define XSENS_CALIBRATION_QUALITIES_H

#include <map>

namespace xsensmvn {

    enum CalibrationQuality
    {
        UNKNOWN = 0, // Unknown quality or not yet performed
        FAILED = 1, // Failed to retrieve a meaningful calibration
        POOR = 2, // Some serious issues, a new calibration is recommended
        ACCEPTABLE = 3, // Some anomalies, but nothing serious
        GOOD = 4, // Good resulting quality, no problems detected
    };
}

#endif // XSENS_CALIBRATION_QUALITIES_H
