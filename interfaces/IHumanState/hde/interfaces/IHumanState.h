// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_INTERFACES_IHUMANSTATE
#define HDE_INTERFACES_IHUMANSTATE

#include <array>
#include <string>
#include <vector>

namespace hde {
    namespace interfaces {
        class IHumanState;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IHumanState
{
public:
    virtual ~IHumanState() = default;

    virtual std::vector<std::string> getJointNames() const = 0;
    virtual std::string getBaseName() const = 0;
    virtual size_t getNumberOfJoints() const = 0;

    virtual std::vector<double> getJointPositions() const = 0;
    virtual std::vector<double> getJointVelocities() const = 0;

    virtual std::array<double, 3> getBasePosition() const = 0;
    virtual std::array<double, 4> getBaseOrientation() const = 0;

    virtual std::array<double, 6> getBaseVelocity() const = 0;

    virtual std::array<double, 3> getCoMPosition() const = 0;
    virtual std::array<double, 3> getCoMVelocity() const = 0;
};

#endif // HDE_INTERFACES_IHUMANSTATE
