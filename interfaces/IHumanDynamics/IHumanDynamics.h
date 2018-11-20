/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_INTERFACES_IHUMANDYNAMICS
#define HDE_INTERFACES_IHUMANDYNAMICS

#include <string>
#include <vector>

namespace hde {
    namespace interfaces {
        class IHumanDynamics;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IHumanDynamics
{
public:
    virtual ~IHumanDynamics() = default;

    virtual std::vector<std::string> getJointNames() const = 0;
    virtual size_t getNumberOfJoints() const = 0;

    virtual std::vector<double> getJointTorques() const = 0;
};

#endif // HDE_INTERFACES_IHUMANDYNAMICS
