/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

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
