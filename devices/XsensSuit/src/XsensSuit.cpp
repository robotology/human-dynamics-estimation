/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "XsensSuit.h"
#include "XSensMVNDriver.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

#include <map>
#include <mutex>
#include <string>

using namespace wearable::devices;

const std::string logPrefix = "XsensSuit :";

class XsensSuit::XsensSuitImpl
{
public:
    class XsensFreeBodyAccelerationSensor;
    class XsensPositionSensor;
    class XsensOrientationSensor;
    class XsensPoseSensor;
    class XsensMagnetometer;
    class XsensVirtualLinkKinSensor;
    class XsensVirtualSphericalJointKinSensor;

    std::unique_ptr<xsensmvn::XSensMVNDriver> driver;
    XsensSuitImpl()
        : driver(nullptr)
    {}

    const std::map<std::string, xsensmvn::CalibrationQuality> calibrationQualities{
        {"Unknown", xsensmvn::CalibrationQuality::UNKNOWN},
        {"Good", xsensmvn::CalibrationQuality::GOOD},
        {"Acceptable", xsensmvn::CalibrationQuality::ACCEPTABLE},
        {"Poor", xsensmvn::CalibrationQuality::POOR},
        {"Failed", xsensmvn::CalibrationQuality::FAILED}};

    const std::map<xsensmvn::DriverStatus, wearable::sensor::SensorStatus> driverToSensorStatusMap{
        {xsensmvn::DriverStatus::Disconnected, sensor::SensorStatus::Error},
        {xsensmvn::DriverStatus::Unknown, sensor::SensorStatus::Unknown},
        {xsensmvn::DriverStatus::Recording, sensor::SensorStatus::Ok},
        {xsensmvn::DriverStatus::Calibrating, sensor::SensorStatus::Calibrating},
        {xsensmvn::DriverStatus::CalibratedAndReadyToRecord,
         sensor::SensorStatus::WaitingForFirstRead},
        {xsensmvn::DriverStatus::Connected, sensor::SensorStatus::WaitingForFirstRead},
        {xsensmvn::DriverStatus::Scanning, sensor::SensorStatus::Error}};

    template <typename T>
    struct driverToDeviceSensors
    {
        std::shared_ptr<T> xsSensor;
        size_t driverIndex;
    };

    std::map<std::string, driverToDeviceSensors<XsensFreeBodyAccelerationSensor>>
        freeBodyAccerlerationSensorsMap;
    std::map<std::string, driverToDeviceSensors<XsensPositionSensor>> positionSensorsMap;
    std::map<std::string, driverToDeviceSensors<XsensOrientationSensor>> orientationSensorsMap;
    std::map<std::string, driverToDeviceSensors<XsensPoseSensor>> poseSensorsMap;
    std::map<std::string, driverToDeviceSensors<XsensMagnetometer>> magnetometersMap;
    std::map<std::string, driverToDeviceSensors<XsensVirtualLinkKinSensor>>
        virtualLinkKinSensorsMap;
    std::map<std::string, driverToDeviceSensors<XsensVirtualSphericalJointKinSensor>>
        virtualSphericalJointKinSensorsMap;

    std::unique_ptr<yarp::os::Network> network = nullptr;

    // ------------------------
    // Custom utility functions
    // ------------------------
    void setAllSensorStates(wearable::sensor::SensorStatus aStatus);

    const WearableName wearableName = "XsensSuit::";
};

// ===================================================
// Xsens implementation of IFreeBodyAccelerationSensor
// ===================================================
class XsensSuit::XsensSuitImpl::XsensFreeBodyAccelerationSensor
    : public wearable::sensor::IFreeBodyAccelerationSensor
{
public:
    // ------------------------
    // Constructor / Destructor
    // ------------------------
    XsensFreeBodyAccelerationSensor(
        XsensSuit::XsensSuitImpl* xsSuitImpl,
        const wearable::sensor::SensorName aName = {},
        const wearable::sensor::SensorStatus aStatus = wearable::sensor::SensorStatus::Unknown)
        : IFreeBodyAccelerationSensor(aName, aStatus)
        , m_suitImpl(xsSuitImpl)
    {}

    ~XsensFreeBodyAccelerationSensor() override = default;

    // -------------------------------------
    // IFreeBodyAccelerationSensor interface
    // -------------------------------------
    bool getFreeBodyAcceleration(wearable::Vector3& fba) const override
    {
        if (m_suitImpl->freeBodyAccerlerationSensorsMap.find(this->m_name)
            == m_suitImpl->freeBodyAccerlerationSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            fba.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const xsensmvn::SensorData sensorData = m_suitImpl->driver->getSensorDataSample().data.at(
            m_suitImpl->freeBodyAccerlerationSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + sensorData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + sensorData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        fba = sensorData.freeBodyAcceleration;
        return true;
    }

    // ------------------------
    // Custom utility functions
    // ------------------------
    inline void setStatus(const wearable::sensor::SensorStatus aStatus) { m_status = aStatus; }

private:
    // ---------
    // Variables
    // ---------
    const XsensSuit::XsensSuitImpl* m_suitImpl = nullptr;
};

// =======================================
// Xsens implementation of IPositionSensor
// =======================================
class XsensSuit::XsensSuitImpl::XsensPositionSensor : public wearable::sensor::IPositionSensor
{
public:
    // ------------------------
    // Constructor / Destructor
    // ------------------------
    XsensPositionSensor(
        XsensSuit::XsensSuitImpl* xsSuitImpl,
        const wearable::sensor::SensorName aName = {},
        const wearable::sensor::SensorStatus aStatus = wearable::sensor::SensorStatus::Unknown)
        : IPositionSensor(aName, aStatus)
        , m_suitImpl(xsSuitImpl)
    {}

    ~XsensPositionSensor() override = default;

    // -------------------------
    // IPositionSensor interface
    // -------------------------
    bool getPosition(wearable::Vector3& pos) const override
    {
        if (m_suitImpl->positionSensorsMap.find(this->m_name)
            == m_suitImpl->positionSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            pos.fill(0.0);
            return false;
        }
        // Retrieve data sample directly form XSens Driver
        const xsensmvn::SensorData sensorData = m_suitImpl->driver->getSensorDataSample().data.at(
            m_suitImpl->positionSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + sensorData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + sensorData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        pos = sensorData.position;
        return true;
    }

    // ------------------------
    // Custom utility functions
    // ------------------------
    inline void setStatus(const wearable::sensor::SensorStatus aStatus) { m_status = aStatus; }

private:
    XsensSuit::XsensSuitImpl* m_suitImpl = nullptr;
};

// ==========================================
// Xsens implementation of IOrientationSensor
// ==========================================
class XsensSuit::XsensSuitImpl::XsensOrientationSensor : public wearable::sensor::IOrientationSensor
{
public:
    // ------------------------
    // Constructor / Destructor
    // ------------------------
    XsensOrientationSensor(
        XsensSuit::XsensSuitImpl* xsSuitImpl,
        const wearable::sensor::SensorName aName = {},
        const wearable::sensor::SensorStatus aStatus = wearable::sensor::SensorStatus::Unknown)
        : IOrientationSensor(aName, aStatus)
        , m_suitImpl(xsSuitImpl)
    {}

    ~XsensOrientationSensor() override = default;

    // -------------------------
    // IOrientationSensor interface
    // -------------------------
    bool getOrientationAsQuaternion(wearable::Quaternion& quat) const override
    {
        if (m_suitImpl->orientationSensorsMap.find(this->m_name)
            == m_suitImpl->orientationSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            quat.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const xsensmvn::SensorData sensorData = m_suitImpl->driver->getSensorDataSample().data.at(
            m_suitImpl->orientationSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + sensorData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + sensorData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        quat = sensorData.orientation;
        return true;
    }

    // ------------------------
    // Custom utility functions
    // ------------------------
    inline void setStatus(const wearable::sensor::SensorStatus aStatus) { m_status = aStatus; }

private:
    XsensSuit::XsensSuitImpl* m_suitImpl = nullptr;
};

// ===================================
// Xsens implementation of IPoseSensor
// ===================================
class XsensSuit::XsensSuitImpl::XsensPoseSensor : public wearable::sensor::IPoseSensor
{
public:
    // ------------------------
    // Constructor / Destructor
    // ------------------------
    XsensPoseSensor(
        XsensSuit::XsensSuitImpl* xsSuitImpl,
        const wearable::sensor::SensorName aName = {},
        const wearable::sensor::SensorStatus aStatus = wearable::sensor::SensorStatus::Unknown)
        : IPoseSensor(aName, aStatus)
        , m_suitImpl(xsSuitImpl)
    {}

    ~XsensPoseSensor() override = default;

    // -------------------------
    // IOrientationSensor interface
    // -------------------------
    bool getPose(Quaternion& orientation, Vector3& position) const override
    {
        if (m_suitImpl->poseSensorsMap.find(this->m_name) == m_suitImpl->poseSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            orientation.fill(0.0);
            position.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const xsensmvn::SensorData sensorData = m_suitImpl->driver->getSensorDataSample().data.at(
            m_suitImpl->poseSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + sensorData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + sensorData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        orientation = sensorData.orientation;
        position = sensorData.position;
        return true;
    }

    // ------------------------
    // Custom utility functions
    // ------------------------
    inline void setStatus(const wearable::sensor::SensorStatus aStatus) { m_status = aStatus; }

private:
    XsensSuit::XsensSuitImpl* m_suitImpl = nullptr;
};

// =====================================
// Xsens implementation of IMagnetometer
// =====================================
class XsensSuit::XsensSuitImpl::XsensMagnetometer : public wearable::sensor::IMagnetometer
{
public:
    // ------------------------
    // Constructor / Destructor
    // ------------------------
    XsensMagnetometer(
        XsensSuit::XsensSuitImpl* xsSuitImpl,
        const wearable::sensor::SensorName aName = {},
        const wearable::sensor::SensorStatus aStatus = wearable::sensor::SensorStatus::Unknown)
        : IMagnetometer(aName, aStatus)
        , m_suitImpl(xsSuitImpl)
    {}

    ~XsensMagnetometer() override = default;

    // -------------------------
    // IMagnetometer interface
    // -------------------------
    bool getMagneticField(wearable::Vector3& mf) const override
    {
        if (m_suitImpl->magnetometersMap.find(this->m_name) == m_suitImpl->magnetometersMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            mf.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const xsensmvn::SensorData sensorData = m_suitImpl->driver->getSensorDataSample().data.at(
            m_suitImpl->magnetometersMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + sensorData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + sensorData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        mf = sensorData.magneticField;
        return true;
    }

    // ------------------------
    // Custom utility functions
    // ------------------------
    inline void setStatus(const wearable::sensor::SensorStatus aStatus) { m_status = aStatus; }

private:
    XsensSuit::XsensSuitImpl* m_suitImpl = nullptr;
};

// =============================================
// Xsens implementation of IVirtualLinkKinsensor
// =============================================
class XsensSuit::XsensSuitImpl::XsensVirtualLinkKinSensor
    : public wearable::sensor::IVirtualLinkKinSensor
{
public:
    // ------------------------
    // Constructor / Destructor
    // ------------------------
    XsensVirtualLinkKinSensor(
        XsensSuit::XsensSuitImpl* xsSuitImpl,
        const wearable::sensor::SensorName aName = {},
        const wearable::sensor::SensorStatus aStatus = wearable::sensor::SensorStatus::Unknown)
        : IVirtualLinkKinSensor(aName, aStatus)
        , m_suitImpl(xsSuitImpl)
    {}

    ~XsensVirtualLinkKinSensor() override = default;

    // -------------------------------
    // IVirtualLinkKinSensor interface
    // -------------------------------
    bool getLinkAcceleration(Vector3& linear, Vector3& angular) const override
    {
        if (m_suitImpl->virtualLinkKinSensorsMap.find(this->m_name)
            == m_suitImpl->virtualLinkKinSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            linear.fill(0.0);
            angular.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const xsensmvn::LinkData linkData = m_suitImpl->driver->getLinkDataSample().data.at(
            m_suitImpl->virtualLinkKinSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + linkData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + linkData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        linear = linkData.linearAcceleration;
        angular = linkData.angularAcceleration;
        return true;
    }

    bool getLinkPose(Vector3& position, Quaternion& orientation) const override
    {
        if (m_suitImpl->virtualLinkKinSensorsMap.find(this->m_name)
            == m_suitImpl->virtualLinkKinSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            position.fill(0.0);
            orientation.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const auto linkData = m_suitImpl->driver->getLinkDataSample().data.at(
            m_suitImpl->virtualLinkKinSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + linkData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + linkData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        position = linkData.position;
        orientation = linkData.orientation;
        return true;
    }

    bool getLinkVelocity(Vector3& linear, Vector3& angular) const override
    {
        if (m_suitImpl->virtualLinkKinSensorsMap.find(this->m_name)
            == m_suitImpl->virtualLinkKinSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            linear.fill(0.0);
            angular.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const auto linkData = m_suitImpl->driver->getLinkDataSample().data.at(
            m_suitImpl->virtualLinkKinSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + linkData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + linkData.name
                     << "Wearable Sensor name " << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        linear = linkData.linearVelocity;
        angular = linkData.angularVelocity;
        return true;
    }

    // ------------------------
    // Custom utility functions
    // ------------------------
    inline void setStatus(const wearable::sensor::SensorStatus aStatus) { m_status = aStatus; }

private:
    XsensSuit::XsensSuitImpl* m_suitImpl = nullptr;
};

// ==============================================
// Xsens implementation of IVirtualJointKinSensor
// ==============================================
class XsensSuit::XsensSuitImpl::XsensVirtualSphericalJointKinSensor
    : public wearable::sensor::IVirtualSphericalJointKinSensor
{
public:
    // ------------------------
    // Constructor / Destructor
    // ------------------------
    XsensVirtualSphericalJointKinSensor(
        XsensSuit::XsensSuitImpl* xsSuitImpl,
        const wearable::sensor::SensorName aName = {},
        const wearable::sensor::SensorStatus aStatus = wearable::sensor::SensorStatus::Unknown)
        : IVirtualSphericalJointKinSensor(aName, aStatus)
        , m_suitImpl(xsSuitImpl)
    {}

    ~XsensVirtualSphericalJointKinSensor() override = default;

    // -----------------------------------------
    // IVirtualSphericalJointKinSensor interface
    // -----------------------------------------
    bool getJointAnglesAsRPY(Vector3& angleAsRPY) const override
    {
        if (m_suitImpl->virtualSphericalJointKinSensorsMap.find(this->m_name)
            == m_suitImpl->virtualSphericalJointKinSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            angleAsRPY.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const xsensmvn::JointData jointData = m_suitImpl->driver->getJointDataSample().data.at(
            m_suitImpl->virtualSphericalJointKinSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + jointData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + jointData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        angleAsRPY = jointData.angles;
        return true;
    }

    bool getJointVelocities(Vector3& velocities) const override
    {
        if (m_suitImpl->virtualSphericalJointKinSensorsMap.find(this->m_name)
            == m_suitImpl->virtualSphericalJointKinSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            velocities.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const auto jointData = m_suitImpl->driver->getJointDataSample().data.at(
            m_suitImpl->virtualSphericalJointKinSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + jointData.name) {
            yError() << logPrefix << "Names mismatch Driver name:" << prefix + jointData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        velocities = jointData.velocities;
        return true;
    }

    bool getJointAccelerations(Vector3& accelerations) const override
    {
        if (m_suitImpl->virtualSphericalJointKinSensorsMap.find(this->m_name)
            == m_suitImpl->virtualSphericalJointKinSensorsMap.end()) {
            yError() << logPrefix << "Sensor" << this->m_name << "NOT found";
            accelerations.fill(0.0);
            return false;
        }

        // Retrieve data sample directly form XSens Driver
        const auto jointData = m_suitImpl->driver->getJointDataSample().data.at(
            m_suitImpl->virtualSphericalJointKinSensorsMap.at(this->m_name).driverIndex);

        // TODO: This should be guaranteed, runtime check should be removed
        // Check if suit sensor and driver data sample have the same name
        auto prefix = m_suitImpl->wearableName + this->getPrefix();
        if (this->m_name != prefix + jointData.name) {
            yError() << logPrefix << "Names mismatch Driver name " << prefix + jointData.name
                     << "Wearable Sensor name" << this->m_name;
            return false;
        }

        // Fill argument with retrieved data
        accelerations = jointData.accelerations;
        return true;
    }

    // ------------------------
    // Custom utility functions
    // ------------------------
    inline void setStatus(const wearable::sensor::SensorStatus aStatus) { m_status = aStatus; }

private:
    XsensSuit::XsensSuitImpl* m_suitImpl = nullptr;
};

// ==========================================
// XsensSuit utility functions implementation
// ==========================================
void XsensSuit::XsensSuitImpl::setAllSensorStates(wearable::sensor::SensorStatus aStatus)
{
    for (const auto& fbas : freeBodyAccerlerationSensorsMap) {
        fbas.second.xsSensor->setStatus(aStatus);
    }
    for (const auto& ps : positionSensorsMap) {
        ps.second.xsSensor->setStatus(aStatus);
    }
    for (const auto& os : orientationSensorsMap) {
        os.second.xsSensor->setStatus(aStatus);
    }
    for (const auto& ps : poseSensorsMap) {
        ps.second.xsSensor->setStatus(aStatus);
    }
    for (const auto& m : magnetometersMap) {
        m.second.xsSensor->setStatus(aStatus);
    }
    for (const auto& vlks : virtualLinkKinSensorsMap) {
        vlks.second.xsSensor->setStatus(aStatus);
    }
    for (const auto& vsjks : virtualSphericalJointKinSensorsMap) {
        vsjks.second.xsSensor->setStatus(aStatus);
    }
}

// =================================================
// XsensSuit constructor / destructor implementation
// =================================================
XsensSuit::XsensSuit()
    : pImpl{new XsensSuitImpl()}
{}

XsensSuit::~XsensSuit() = default;

// ======================
// DeviceDriver interface
// ======================
bool XsensSuit::open(yarp::os::Searchable& config)
{
    yInfo() << logPrefix << " Starting to configure";
    // Read from config file the Xsens rundeps folder
    if (!config.check("xsens-rundeps-dir")) {
        yError() << logPrefix << "REQUIRED parameter <xsens-rundeps-dir> NOT found";
        return false;
    }
    const std::string rundepsFolder = config.find("xsens-rundeps-dir").asString();

    // Read from config file the Xsens suit configuration to use.
    if (!config.check("suit-config")) {
        yError() << logPrefix << "REQUIRED parameter <suit-config> NOT found";
        return false;
    }
    const std::string suitConfiguration = config.find("suit-config").asString();

    // Read from config file the Xsens acquisition scenario to use.
    // Since it is optional, if not found it is safe to use an empty string
    std::string acquisitionScenario;
    if (!config.check("acquisition-scenario")) {
        yWarning() << logPrefix << "OPTIONAL parameter <acquisition-scenario> NOT found";
        acquisitionScenario = "";
    }
    else {
        acquisitionScenario = config.find("acquisition-scenario").asString();
    }

    // Read from config file the calibration routine to be used as default.
    // Since it is optional, if not found it is safe to use an empty string
    std::string defaultCalibrationType = "Npose";
    if (!config.check("default-calibration-type")) {
        yWarning() << logPrefix << "OPTIONAL parameter <default-calibration-type> NOT found";
        yInfo() << logPrefix << "Using Npose as default";
    }
    else {
        defaultCalibrationType = config.find("default-calibration-type").asString();
    }

    // Read from config file the minimum required calibration quality.
    // If not provided using POOR
    xsensmvn::CalibrationQuality minCalibrationQualityRequired;
    if (!config.check("minimum-calibration-quality-required")) {
        yWarning() << logPrefix
                   << "OPTIONAL parameter <minimum-calibration-quality-required> NOT found";
        yWarning() << logPrefix << "Using POOR as minimum required calibration quality";
        minCalibrationQualityRequired = pImpl->calibrationQualities.at("Poor");
    }
    else {
        std::string tmpLabel = config.find("minimum-calibration-quality-required").asString();
        if (pImpl->calibrationQualities.find(tmpLabel) != pImpl->calibrationQualities.end()) {
            minCalibrationQualityRequired = pImpl->calibrationQualities.at(tmpLabel);
        }
        else {
            yWarning() << logPrefix
                       << "OPTIONAL parameter <minimum-calibration-quality-required> INVALID";
            yWarning() << logPrefix << "Using POOR as minimum required calibration quality";
            minCalibrationQualityRequired = pImpl->calibrationQualities.at("Poor");
        }
    }

    // Read from config file the scan-for-suit timeout.
    // If not provided ENABLING endless scan mode
    int scanTimeout = -1;
    if (!config.check("scan-timeout")) {
        yWarning() << logPrefix << "OPTIONAL parameter <scan-timeout> NOT found";
        yWarning() << logPrefix << "Endless scan mode ENABLED";
    }
    else {
        scanTimeout = config.find("scan-timeout").asInt32();
    }

    // Read from config file the sampling-rate.
    // If not provided USING highest available one
    int samplingRate = -1;
    if (!config.check("sampling-rate")) {
        yWarning() << logPrefix << "OPTIONAL parameter <sampling-rate> NOT found";
        yWarning() << logPrefix << "Using highest supported sampling rate";
    }
    else {
        samplingRate = config.find("sampling-rate").asInt32();
    }

    // Get subject-specific body dimensions from the configuration file and push them to
    // subjectBodyDimensions
    xsensmvn::bodyDimensions subjectBodyDimensions;
    yarp::os::Bottle bodyDimensionSet = config.findGroup("body-dimensions", "");
    if (bodyDimensionSet.isNull()) {
        yWarning() << logPrefix << "OPTIONAL parameter group <body-dimensions> NOT found";
        yWarning() << logPrefix
                   << "USING default body dimensions, this may affect estimation quality";
    }
    else {
        for (size_t i = 1; i < bodyDimensionSet.size(); ++i) {
            if (bodyDimensionSet.get(i).asList()->get(1).isFloat64()
                && bodyDimensionSet.get(i).asList()->get(1).asFloat64() != -1) {
                subjectBodyDimensions.insert({bodyDimensionSet.get(i).asList()->get(0).asString(),
                                              bodyDimensionSet.get(i).asList()->get(1).asFloat64()});
            }
        }
    }

    // Read from config file the selected output stream configuration.
    // If not provided USING default configuration, Joints: OFF, Links: ON, Sensors: ON
    xsensmvn::DriverDataStreamConfig outputStreamConfig;
    yarp::os::Bottle streamGroup = config.findGroup("output-stream-configuration", "");
    if (streamGroup.isNull()) {
        yWarning() << logPrefix
                   << "OPTIONAL parameters group <output-stream-configuration> NOT found";
        yWarning() << logPrefix
                   << "USING default configuration, Joints: OFF, Links: ON, Sensors: ON";
    }
    outputStreamConfig.enableJointData =
        streamGroup.check("enable-joint-data", yarp::os::Value(false)).asBool();
    outputStreamConfig.enableLinkData =
        streamGroup.check("enable-link-data", yarp::os::Value(true)).asBool();
    outputStreamConfig.enableSensorData =
        streamGroup.check("enable-sensor-data", yarp::os::Value(true)).asBool();

    // Check for mvn recording flag
    bool saveMVNRecording;
    if (!config.check("saveMVNRecording")) {
        yWarning() << logPrefix
                   << "OPTIONAL parameter <saveMVNRecording> NOT found, setting it to false.";
        saveMVNRecording = false;
    }
    else {
        saveMVNRecording = config.find("saveMVNRecording").asBool();
        yInfo() << logPrefix << "<saveMVNRecording> parameter set to " << saveMVNRecording;
    }

    // Check for saving current calibration flag
    bool saveCurrentCalibration;
    if (!config.check("saveCurrentCalibration")) {
        yWarning() << logPrefix
                   << "OPTIONAL parameter <saveCurrentCalibration> NOT found, setting it to false.";
        saveCurrentCalibration = false;
    }
    else {
        saveCurrentCalibration = config.find("saveCurrentCalibration").asBool();
        yInfo() << logPrefix << "<saveCurrentCalibration> parameter set to "
                << saveCurrentCalibration;
    }

    xsensmvn::DriverConfiguration driverConfig{rundepsFolder,
                                               suitConfiguration,
                                               acquisitionScenario,
                                               defaultCalibrationType,
                                               minCalibrationQualityRequired,
                                               scanTimeout,
                                               samplingRate,
                                               subjectBodyDimensions,
                                               outputStreamConfig,
                                               saveMVNRecording,
                                               saveCurrentCalibration};

    pImpl->driver.reset(new xsensmvn::XSensMVNDriver(driverConfig));

    if (!pImpl->driver->configureAndConnect()) {
        yError() << logPrefix << "Unable to configure the driver and connect to the suit";
        return false;
    }

    std::string fbasPrefix = getWearableName() + sensor::IFreeBodyAccelerationSensor::getPrefix();
    std::string posPrefix = getWearableName() + sensor::IPositionSensor::getPrefix();
    std::string orientPrefix = getWearableName() + sensor::IOrientationSensor::getPrefix();
    std::string posePrefix = getWearableName() + sensor::IPoseSensor::getPrefix();
    std::string magPrefix = getWearableName() + sensor::IMagnetometer::getPrefix();
    std::string vlksPrefix = getWearableName() + sensor::IVirtualLinkKinSensor::getPrefix();
    std::string vsjksPrefix =
        getWearableName() + sensor::IVirtualSphericalJointKinSensor::getPrefix();

    if (pImpl->driver->getDriverConfiguration().dataStreamConfiguration.enableSensorData) {
        // Get the names of the sensors from the driver
        std::vector<std::string> sensorNames = pImpl->driver->getSuitSensorLabels();

        for (size_t s = 0; s < sensorNames.size(); ++s) {
            // Create the new sensors
            auto fba = std::make_shared<XsensSuitImpl::XsensFreeBodyAccelerationSensor>(
                pImpl.get(), fbasPrefix + sensorNames[s]);
            auto pos = std::make_shared<XsensSuitImpl::XsensPositionSensor>(
                pImpl.get(), posPrefix + sensorNames[s]);
            auto orient = std::make_shared<XsensSuitImpl::XsensOrientationSensor>(
                pImpl.get(), orientPrefix + sensorNames[s]);
            auto pose = std::make_shared<XsensSuitImpl::XsensPoseSensor>(
                pImpl.get(), posePrefix + sensorNames[s]);
            auto mag = std::make_shared<XsensSuitImpl::XsensMagnetometer>(
                pImpl.get(), magPrefix + sensorNames[s]);

            pImpl->freeBodyAccerlerationSensorsMap.emplace(
                fbasPrefix + sensorNames[s],
                XsensSuitImpl::driverToDeviceSensors<
                    XsensSuitImpl::XsensFreeBodyAccelerationSensor>{fba, s});

            pImpl->positionSensorsMap.emplace(
                posPrefix + sensorNames[s],
                XsensSuitImpl::driverToDeviceSensors<XsensSuitImpl::XsensPositionSensor>{pos, s});

            pImpl->orientationSensorsMap.emplace(
                orientPrefix + sensorNames[s],
                XsensSuitImpl::driverToDeviceSensors<XsensSuitImpl::XsensOrientationSensor>{orient,
                                                                                            s});

            pImpl->poseSensorsMap.emplace(
                posePrefix + sensorNames[s],
                XsensSuitImpl::driverToDeviceSensors<XsensSuitImpl::XsensPoseSensor>{pose, s});

            pImpl->magnetometersMap.emplace(
                magPrefix + sensorNames[s],
                XsensSuitImpl::driverToDeviceSensors<XsensSuitImpl::XsensMagnetometer>{mag, s});
        }
    }

    if (pImpl->driver->getDriverConfiguration().dataStreamConfiguration.enableLinkData) {
        // Get the names of the links from the driver
        std::vector<std::string> linkNames = pImpl->driver->getSuitLinkLabels();

        for (size_t s = 0; s < linkNames.size(); ++s) {
            // Create the new sensor
            auto sensor = std::make_shared<XsensSuitImpl::XsensVirtualLinkKinSensor>(
                pImpl.get(), vlksPrefix + linkNames[s]);
            // Insert it in the output structure
            pImpl->virtualLinkKinSensorsMap.emplace(
                vlksPrefix + linkNames[s],
                XsensSuitImpl::driverToDeviceSensors<XsensSuitImpl::XsensVirtualLinkKinSensor>{
                    sensor, s});
        }
    }

    if (pImpl->driver->getDriverConfiguration().dataStreamConfiguration.enableJointData) {
        // Get the names of the joints from the driver
        std::vector<std::string> jointNames = pImpl->driver->getSuitJointLabels();

        for (size_t s = 0; s < jointNames.size(); ++s) {
            // Create the new sensor
            auto sensor = std::make_shared<XsensSuitImpl::XsensVirtualSphericalJointKinSensor>(
                pImpl.get(), vsjksPrefix + jointNames[s]);
            // Insert it in the output structure
            pImpl->virtualSphericalJointKinSensorsMap.emplace(
                vsjksPrefix + jointNames[s],
                XsensSuitImpl::driverToDeviceSensors<
                    XsensSuitImpl::XsensVirtualSphericalJointKinSensor>{sensor, s});
        }
    }

    // =================================
    // CHECK YARP NETWORK INITIALIZATION
    // =================================

    pImpl->network = std::make_unique<yarp::os::Network>();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << logPrefix << "YARP server wasn't found active.";
        return false;
    }

    return true;
}

bool XsensSuit::close()
{
    pImpl->driver->stopAcquisition();
    pImpl->driver->terminate();
    return true;
}

// =========================
// IPreciselyTimed interface
// =========================
yarp::os::Stamp XsensSuit::getLastInputStamp()
{
    // Stamp count should be always zero
    return yarp::os::Stamp(0, pImpl->driver->getTimeStamps().systemTime);
}

// ==========================
// IXsensMVNControl interface
// ==========================

bool XsensSuit::setBodyDimensions(const std::map<std::string, double>& dimensions)
{
    if (!pImpl->driver) {
        return false;
    }

    return pImpl->driver->setBodyDimensions(dimensions);
}

bool XsensSuit::getBodyDimensions(std::map<std::string, double>& dimensions) const
{
    if (!pImpl->driver) {
        return false;
    }

    return pImpl->driver->getBodyDimensions(dimensions);
}

bool XsensSuit::getBodyDimension(const std::string bodyName, double& dimension) const
{
    if (!pImpl->driver) {
        return false;
    }

    return pImpl->driver->getBodyDimension(bodyName, dimension);
}

// Calibration methods
bool XsensSuit::calibrate(const std::string& calibrationType)
{

    if (!pImpl->driver) {
        return false;
    }

    pImpl->setAllSensorStates(sensor::SensorStatus::Calibrating);

    bool calibrationSuccess = pImpl->driver->calibrate(calibrationType);

    if (calibrationSuccess) {
        pImpl->setAllSensorStates(sensor::SensorStatus::WaitingForFirstRead);
    }
    else {
        pImpl->setAllSensorStates(sensor::SensorStatus::Error);
    }

    return calibrationSuccess;
}

bool XsensSuit::abortCalibration()
{
    if (!pImpl->driver) {
        return false;
    }

    bool abortCalibrationSuccess = pImpl->driver->abortCalibration();

    if (abortCalibrationSuccess) {
        pImpl->setAllSensorStates(sensor::SensorStatus::Unknown);
    }
    else {
        pImpl->setAllSensorStates(sensor::SensorStatus::Error);
    }

    return abortCalibrationSuccess;
}

// Acquisition methods
bool XsensSuit::startAcquisition()
{
    if (!pImpl->driver) {
        return false;
    }

    bool startAcquisitionSuccess = pImpl->driver->startAcquisition();

    if (startAcquisitionSuccess) {
        pImpl->setAllSensorStates(sensor::SensorStatus::Ok);
    }
    else {
        pImpl->setAllSensorStates(sensor::SensorStatus::WaitingForFirstRead);
    }

    return startAcquisitionSuccess;
}

bool XsensSuit::stopAcquisition()
{
    if (!pImpl->driver) {
        return false;
    }

    bool stopAcquisitionSuccess = pImpl->driver->stopAcquisition();
    if (stopAcquisitionSuccess) {
        pImpl->setAllSensorStates(sensor::SensorStatus::WaitingForFirstRead);
    }
    else {
        pImpl->setAllSensorStates(sensor::SensorStatus::Ok);
    }

    return stopAcquisitionSuccess;
}

// ===============
// IWear interface
// ===============

// ---------------
// Generic Methods
// ---------------

wearable::WearableName XsensSuit::getWearableName() const
{
    return pImpl->wearableName;
}

wearable::WearStatus XsensSuit::getStatus() const
{
    return pImpl->driverToSensorStatusMap.at(pImpl->driver->getStatus());
}

wearable::TimeStamp XsensSuit::getTimeStamp() const
{
    // Stamp count should be always zero
    return {pImpl->driver->getTimeStamps().systemTime, 0};
}

// ---------------------------
// Implemented Sensors Methods
// ---------------------------

wearable::SensorPtr<const wearable::sensor::ISensor>
XsensSuit::getSensor(const wearable::sensor::SensorName name) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> sensors = getAllSensors();
    for (const auto& s : sensors) {
        if (s->getSensorName() == name) {
            return s;
        }
    }
    yWarning() << logPrefix << "User specified name <" << name << "> not found";
    return nullptr;
}

wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
XsensSuit::getSensors(const wearable::sensor::SensorType aType) const
{
    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor> outVec;
    switch (aType) {
        case sensor::SensorType::FreeBodyAccelerationSensor: {
            outVec.reserve(pImpl->freeBodyAccerlerationSensorsMap.size());
            for (const auto& fbas : pImpl->freeBodyAccerlerationSensorsMap) {
                outVec.push_back(
                    static_cast<std::shared_ptr<sensor::ISensor>>(fbas.second.xsSensor));
            }
            break;
        }
        case sensor::SensorType::PositionSensor: {
            outVec.reserve(pImpl->positionSensorsMap.size());
            for (const auto& ps : pImpl->positionSensorsMap) {
                outVec.push_back(static_cast<std::shared_ptr<sensor::ISensor>>(ps.second.xsSensor));
            }
            break;
        }
        case sensor::SensorType::OrientationSensor: {
            outVec.reserve(pImpl->orientationSensorsMap.size());
            for (const auto& os : pImpl->orientationSensorsMap) {
                outVec.push_back(static_cast<std::shared_ptr<sensor::ISensor>>(os.second.xsSensor));
            }
            break;
        }
        case sensor::SensorType::PoseSensor: {
            outVec.reserve(pImpl->poseSensorsMap.size());
            for (const auto& ps : pImpl->poseSensorsMap) {
                outVec.push_back(static_cast<std::shared_ptr<sensor::ISensor>>(ps.second.xsSensor));
            }
            break;
        }
        case sensor::SensorType::Magnetometer: {
            outVec.reserve(pImpl->magnetometersMap.size());
            for (const auto& m : pImpl->magnetometersMap) {
                outVec.push_back(static_cast<std::shared_ptr<sensor::ISensor>>(m.second.xsSensor));
            }
            break;
        }
        case sensor::SensorType::VirtualLinkKinSensor: {
            outVec.reserve(pImpl->virtualLinkKinSensorsMap.size());
            for (const auto& vlks : pImpl->virtualLinkKinSensorsMap) {
                outVec.push_back(
                    static_cast<std::shared_ptr<sensor::ISensor>>(vlks.second.xsSensor));
            }
            break;
        }
        case sensor::SensorType::VirtualSphericalJointKinSensor: {
            outVec.reserve(pImpl->virtualSphericalJointKinSensorsMap.size());
            for (const auto& vsjks : pImpl->virtualSphericalJointKinSensorsMap) {
                outVec.push_back(
                    static_cast<std::shared_ptr<sensor::ISensor>>(vsjks.second.xsSensor));
            }

            break;
        }
        default: {
            yWarning() << logPrefix << "Selected sensor type (" << static_cast<int>(aType)
                       << ") is not supported by XsensSuit";
            return {};
        }
    }

    return outVec;
}

wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
XsensSuit::getFreeBodyAccelerationSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->freeBodyAccerlerationSensorsMap.find(static_cast<std::string>(name))
        == pImpl->freeBodyAccerlerationSensorsMap.end()) {
        yError() << logPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared pointer to the required sensor
    return static_cast<std::shared_ptr<sensor::IFreeBodyAccelerationSensor>>(
        pImpl->freeBodyAccerlerationSensorsMap.at(static_cast<std::string>(name)).xsSensor);
}

wearable::SensorPtr<const wearable::sensor::IPositionSensor>
XsensSuit::getPositionSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->positionSensorsMap.find(static_cast<std::string>(name))
        == pImpl->positionSensorsMap.end()) {
        yError() << logPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared pointer to the required sensor
    return static_cast<std::shared_ptr<sensor::IPositionSensor>>(
        pImpl->positionSensorsMap.at(static_cast<std::string>(name)).xsSensor);
}

wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
XsensSuit::getOrientationSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->orientationSensorsMap.find(static_cast<std::string>(name))
        == pImpl->orientationSensorsMap.end()) {
        yError() << logPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared pointer to the required sensor
    return static_cast<std::shared_ptr<sensor::IOrientationSensor>>(
        pImpl->orientationSensorsMap.at(static_cast<std::string>(name)).xsSensor);
}

wearable::SensorPtr<const wearable::sensor::IPoseSensor>
XsensSuit::getPoseSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->poseSensorsMap.find(static_cast<std::string>(name)) == pImpl->poseSensorsMap.end()) {
        yError() << logPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared pointer to the required sensor
    return static_cast<std::shared_ptr<sensor::IPoseSensor>>(
        pImpl->poseSensorsMap.at(static_cast<std::string>(name)).xsSensor);
}

wearable::SensorPtr<const wearable::sensor::IMagnetometer>
XsensSuit::getMagnetometer(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->magnetometersMap.find(static_cast<std::string>(name))
        == pImpl->magnetometersMap.end()) {
        yError() << logPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared pointer to the required sensor
    return static_cast<std::shared_ptr<sensor::IMagnetometer>>(
        pImpl->magnetometersMap.at(static_cast<std::string>(name)).xsSensor);
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
XsensSuit::getVirtualLinkKinSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->virtualLinkKinSensorsMap.find(static_cast<std::string>(name))
        == pImpl->virtualLinkKinSensorsMap.end()) {
        yError() << logPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared pointer to the required sensor
    return static_cast<std::shared_ptr<sensor::IVirtualLinkKinSensor>>(
        pImpl->virtualLinkKinSensorsMap.at(static_cast<std::string>(name)).xsSensor);
}

wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
XsensSuit::getVirtualSphericalJointKinSensor(const wearable::sensor::SensorName name) const
{
    // Check if user-provided name corresponds to an available sensor
    if (pImpl->virtualSphericalJointKinSensorsMap.find(static_cast<std::string>(name))
        == pImpl->virtualSphericalJointKinSensorsMap.end()) {
        yError() << logPrefix << "Invalid sensor name";
        return nullptr;
    }

    // Return a shared pointer to the required sensor
    return static_cast<std::shared_ptr<sensor::IVirtualSphericalJointKinSensor>>(
        pImpl->virtualSphericalJointKinSensorsMap.at(static_cast<std::string>(name)).xsSensor);
}
