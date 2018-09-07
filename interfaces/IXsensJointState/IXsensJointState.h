/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_INTERFACES_IXSENSJOINTSTATE
#define HDE_INTERFACES_IXSENSJOINTSTATE

#include <array>
#include <string>
#include <vector>

namespace hde {
    namespace interfaces {
        class IXsensJointState;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IXsensJointState
{
public:
    virtual ~IXsensJointState() = default;

    virtual std::vector<std::string> getJointNames() const = 0;
    virtual size_t getNumberOfJoints() const = 0;

    virtual std::array<double, 3> getBasePosition() const = 0;
    virtual std::array<double, 4> getBaseOrientation() const = 0;

    virtual std::array<double, 3> getBaseVelocity() const = 0;
};

#endif // HDE_INTERFACES_IXSENSJOINTSTATE
