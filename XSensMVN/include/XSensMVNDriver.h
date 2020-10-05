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

#include "IXsensMVNControl.h"

#include <array>
#include <map>
#include <mutex>
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
        Vector3 angles; // RPY fix frame X-Y-Z
        Vector3 velocities;
        Vector3 accelerations;
    };

    struct Timestamp
    {
        double systemTime;
        double relative; // [s]
        double absolute; // [s]
    };

    struct LinkDataVector
    {
        std::shared_ptr<Timestamp> time = nullptr;
        std::vector<LinkData> data;
    };

    struct SensorDataVector
    {
        std::shared_ptr<Timestamp> time = nullptr;
        std::vector<SensorData> data;
    };

    struct JointDataVector
    {
        std::shared_ptr<Timestamp> time = nullptr;
        std::vector<JointData> data;
    };

    struct DriverDataSample
    {
        std::shared_ptr<Timestamp> timestamps = nullptr;
        LinkDataVector links;
        SensorDataVector sensors;
        JointDataVector joints;

        DriverDataSample()
            : timestamps(new Timestamp)
        {
            links.time = timestamps;
            sensors.time = timestamps;
            joints.time = timestamps;
        }

        inline void reset()
        {
            timestamps->absolute = 0.0;
            timestamps->relative = 0.0;
            timestamps->systemTime = 0.0;

            for (auto& element : links.data) {
                const std::string name = element.name;
                element = {};
                element.name = name;
            }

            for (auto& element : sensors.data) {
                const std::string name = element.name;
                element = {};
                element.name = name;
            }

            for (auto& element : joints.data) {
                const std::string name = element.name;
                element = {};
                element.name = name;
            }
        }
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
        const int samplingRate;
        const bodyDimensions bodyDimensions;
        const DriverDataStreamConfig dataStreamConfiguration;
        const bool saveMVNRecording;
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

class xsensmvn::XSensMVNDriver : public IXsensMVNControl
{
private:
    /* ---------- *
     *  Variables *
     * ---------- */

    // Struct to internally store (cache) a single data sample
    std::shared_ptr<DriverDataSample> m_dataSample = nullptr;
    std::shared_ptr<std::mutex> m_dataMutex = nullptr;

    // Pointer to the driver implementation
    std::unique_ptr<XSensMVNDriverImpl> m_pimpl;

    // Move the data from the implementation internal memory to the local one
    // void cacheData();

public:
    /* -------------------------- *
     *  Construtors / Destructors *
     * -------------------------- */

    XSensMVNDriver() = delete;
    XSensMVNDriver(const xsensmvn::DriverConfiguration& conf);
    ~XSensMVNDriver() override;

    /* ---------- *
     *  Functions *
     * ---------- */

    // Prevent copy
    XSensMVNDriver(const XSensMVNDriver& other) = delete;
    XSensMVNDriver& operator=(const XSensMVNDriver& other) = delete;

    // Driver opening and closing
    bool configureAndConnect();
    bool terminate();

    // Minimum calibration quality considered to be satisfactory set/get
    bool setMinimumAcceptableCalibrationQuality(const xsensmvn::CalibrationQuality quality) const;
    const xsensmvn::CalibrationQuality& getMinimumAcceptableCalibrationQuality() const;

    const xsensmvn::DriverConfiguration& getDriverConfiguration() const;

    // Data accessor
    // Data are returned as reference, but the user should not store it because the data are
    // automaically updated by the driver callback. This is intended to be used only to avoid
    // copying all the data in case just one is needed.
    const DriverDataSample getDataSample();
    const LinkDataVector getLinkDataSample();
    const SensorDataVector getSensorDataSample();
    const JointDataVector getJointDataSample();

    // Metadata accessor
    //    double getSampleRelativeTime() const;
    //    double getSampleAbsoluteTime() const;

    // Labels accessor
    std::vector<std::string> getSuitLinkLabels() const;
    std::vector<std::string> getSuitSensorLabels() const;
    std::vector<std::string> getSuitJointLabels() const;

    // Status get
    xsensmvn::DriverStatus getStatus() const;

    // Timestamp get
    xsensmvn::Timestamp getTimeStamps() const;

    /* --------------------------- *
     *  IXsensMVNControl Interface *
     * --------------------------- */

    bool startAcquisition() override;
    bool stopAcquisition() override;

    bool calibrate(const std::string& calibrationType = {}) override;
    bool abortCalibration() override;

    // Body-dimensions set/get
    bool setBodyDimensions(const std::map<std::string, double>& bodyDimensions) override;
    bool getBodyDimensions(std::map<std::string, double>& dimensions) const override;
    bool getBodyDimension(const std::string bodyName, double& dimension) const override;
};

#endif // XSENS_MVN_DRIVER_H
