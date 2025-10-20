// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

namespace yarp wearable.msg

/**
 * Methods definition for the XsensDriver Wrapper service
 */
service XsensSuitControlService {

    /**
     * Calibrate the Xsens device with the default calibration procedure
     * @return true if the calibration is successful, false otherwise
     */
    bool calibrate();

    /**
     * Calibrate the Xsens device with the calibration procedure identified
     * by the specified parameter
     *
     * @param calibrationType name of the calibration to execute
     * @return true if the calibration is successful, false otherwise
     */
    bool calibrateWithType(1: string calibrationType);

    /**
     * Abort the calibration procedure
     */
    bool abortCalibration();

    /**
     * Start acquiring data from the Xsens suit
     */
    bool startAcquisition();

    /**
     * Stop acquiring data from the suit
     */
    bool stopAcquisition();
}
