/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENS_MVN_DRIVER_H
#define XSENS_MVN_DRIVER_H

#include "XSensCalibrationQualities.h"

#include <array>
#include <map>
#include <vector>

namespace xsensmvn {

    class XSensMVNDriver;
    class XSensMVNDriverImpl;

    /* ------------ *
     *  Definitions *
     * ------------ */

    using Vector3 = std::array<double, 3>;
    using Quaternion = std::array<double, 4>;
    using bodyDimensions = std::map<std::string, double>;

    struct LinkData
    {
        std::string name;
        Vector3 position;
        Vector3 linearVelocity;
        Vector3 linearAcceleration;
        Quaternion orientation;
        Vector3 angularVelocity;
        Vector3 angularAcceleration;
    };

    struct SensorData
    {
        std::string name;
        Vector3 position;
        Quaternion orientation;
        Vector3 freeBodyAcceleration;
        Vector3 magneticField;
    };

    struct JointData
    {
        std::string name;
        Vector3 jointAngles;
    };

    struct DriverDataSample
    {
        std::string suitName;
        double relativeTime; // [s]
        double absoluteTime; // [s]
        std::vector<LinkData> links;
        std::vector<SensorData> sensors;
        std::vector<JointData> joints;
    };

    struct DriverDataStreamConfig
    {
        bool enableLinkData;
        bool enableSensorData;
        bool enableJointData;
    };

    struct DriverConfiguration
    {
        const std::string licensePath;
        const std::string suitConfiguration;
        const std::string acquisitionScenario;
        const std::string defaultCalibrationType;
        const xsensmvn::CalibrationQuality minimumRequiredCalibrationQuality;
        const int scanTimeout;
        const bodyDimensions bodyDimensions;
        const DriverDataStreamConfig dataStreamConfiguration;
    };

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

} // namespace xsensmvn
