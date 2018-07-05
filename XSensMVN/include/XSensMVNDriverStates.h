/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENS_MVN_DRIVER_STATES_H
#define XSENS_MVN_DRIVER_STATES_H

/* Hack required to print verbose log messages when not in Release
 * It is not possible to directly use the NDEBUG variable since XSens defines regardless from the
 * compile option
 * This brings as consequence that the Logger should be included in the .h files ALWAYS before XSens
 * SDK files
 */
namespace xsens {
    enum class DriverStatus
    {
        Disconnected = 0,
        Scanning,
        Connected,
        Calibrating,
        CalibratedAndReadyToRecord,
        Recording,
        Unknown,
    };
}

#endif // XSENS_MVN_DRIVER_STATES_H
