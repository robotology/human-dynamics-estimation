// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
