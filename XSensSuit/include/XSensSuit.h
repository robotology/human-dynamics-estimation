/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_XSENSSUIT
#define WEAR_XSENSSUIT

#include "IWear/IWear.h"

namespace wearable {
    namespace suit {
        class XSensSuit;
    }
} // namespace wearable

class wearable::suit::XSensSuit final : public wearable::IWear
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    XSensSuit();
    ~XSensSuit() override;

    XSensSuit(const XSensSuit& other) = delete;
    XSensSuit(XSensSuit&& other) = delete;
    XSensSuit& operator=(const XSensSuit& other) = delete;
    XSensSuit& operator=(XSensSuit&& other) = delete;

    // =======
    // GENERIC
    // =======

    wearable::SensorPtr<const wearable::sensor::ISensor>
    getSensor(const wearable::sensor::SensorName name) const override;

    wearable::VectorOfSensorPtr<const wearable::sensor::ISensor>
    getSensors(const wearable::sensor::SensorType) const override;

    // ==============
    // SINGLE SENSORS
    // ==============

    wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IMagnetometer>
    getMagnetometer(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
    getOrientationSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IPoseSensor>
    getPoseSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IPositionSensor>
    getPositionSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const wearable::sensor::SensorName name) const override;

    wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const wearable::sensor::SensorName name) const override;
};

#endif // WEAR_XSENSSUIT
