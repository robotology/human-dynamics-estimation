/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

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
    virtual std::array<double, 3> getCoMBiasAcceleration() const = 0;
    virtual std::array<double, 3> getCoMProperAcceleration() const = 0;

    virtual std::vector<std::string> getAccelerometerNames() const = 0;
    virtual std::vector<std::array<double, 3>> getProperLinAccelerations() const = 0;
    virtual std::vector<std::array<double, 3>> getProperAngAccelerations() const = 0;
    virtual std::vector<std::array<double, 6>> getProperAccelerations() const = 0;
};

#endif // HDE_INTERFACES_IHUMANSTATE
