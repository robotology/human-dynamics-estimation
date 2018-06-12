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

namespace wear {
    namespace suit {
        class XSensSuit;
    }
} // namespace wear

class wear::suit::XSensSuit final : public wear::IWear
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

    wear::SensorPtr<const wear::sensor::ISensor>
    getSensor(const wear::sensor::SensorName name) const override;

    wear::VectorOfSensorPtr<const wear::sensor::ISensor>
    getSensors(const wear::sensor::SensorType) const override;

    // ==============
    // SINGLE SENSORS
    // ==============

    wear::SensorPtr<const wear::sensor::IFreeBodyAccelerationSensor>
    getFreeBodyAccelerationSensor(const wear::sensor::SensorName name) const override;

    wear::SensorPtr<const wear::sensor::IMagnetometer>
    getMagnetometer(const wear::sensor::SensorName name) const override;

    wear::SensorPtr<const wear::sensor::IOrientationSensor>
    getOrientationSensor(const wear::sensor::SensorName name) const override;

    wear::SensorPtr<const wear::sensor::IPoseSensor>
    getPoseSensor(const wear::sensor::SensorName name) const override;

    wear::SensorPtr<const wear::sensor::IPositionSensor>
    getPositionSensor(const wear::sensor::SensorName name) const override;

    wear::SensorPtr<const wear::sensor::IVirtualLinkKinSensor>
    getVirtualLinkKinSensor(const wear::sensor::SensorName name) const override;

    wear::SensorPtr<const wear::sensor::IVirtualSphericalJointKinSensor>
    getVirtualSphericalJointKinSensor(const wear::sensor::SensorName name) const override;
};

#endif // WEAR_XSENSSUIT
