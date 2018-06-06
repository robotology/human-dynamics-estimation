/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_IVIRTUALLINKKIN
#define WEAR_IVIRTUALLINKKIN

#include "ISensor.h"
#include <string>

namespace wear {
    namespace sensor {
        class IVirtualLinkKin;
    }
} // namespace wear

class wear::sensor::IVirtualLinkKin : public wear::sensor::ISensor
{
protected:
    using LinkName = std::string;
    LinkName m_name;

public:
    virtual ~IVirtualLinkKin() = default;

    virtual bool getPose(wear::Vector3& position, wear::Quaternion& orientation) const = 0;
    virtual bool getVelocity(wear::Vector3& linear, wear::Vector3& angular) const = 0;
    virtual bool getAcceleration(wear::Vector3& linear, wear::Vector3& angular) const = 0;
    virtual bool getOrientation(wear::Quaternion& orientation) const = 0;
    virtual bool getPosition(wear::Vector3& position) const = 0;
    virtual bool getLinearVelocity(wear::Vector3& linear) const = 0;
    virtual bool getLinearAcceleration(wear::Vector3& linear) const = 0;
    virtual bool getAngularVelocity(wear::Vector3& linear) const = 0;
    virtual bool getAngularAcceleration(wear::Vector3& linear) const = 0;

    virtual LinkName getLinkName() const;
};

wear::sensor::IVirtualLinkKin::LinkName wear::sensor::IVirtualLinkKin::getLinkName() const
{
    return m_name;
}

#endif // WEAR_IVIRTUALLINKKIN
