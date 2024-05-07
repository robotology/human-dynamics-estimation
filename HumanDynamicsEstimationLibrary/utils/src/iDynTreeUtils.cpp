// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "hde/utils/iDynTreeUtils.hpp"

// std
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>

// YARP
#include <yarp/os/LogStream.h>

using namespace hde::utils::idyntree;

const std::string LogPrefixUtils = "HumanDynamicsEstimation::Utils:: ";

// ================
// ROTATION HELPERS
// ================

iDynTree::Matrix3x3 rotation::skewSymmetric(const iDynTree::Matrix3x3& input)
{
    iDynTree::Matrix3x3 output;
    iDynTree::toEigen(output) =
        0.5 * (iDynTree::toEigen(input) - iDynTree::toEigen(input).transpose());
    return output;
}

iDynTree::Vector3 rotation::skewVee(const iDynTree::Matrix3x3& input)
{
    iDynTree::Vector3 output;
    iDynTree::Matrix3x3 skewSymmetric;

    skewSymmetric = rotation::skewSymmetric(input);
    iDynTree::toEigen(output) = iDynTree::unskew(iDynTree::toEigen(skewSymmetric));

    return output;
}

rotation::RotationDistance::RotationDistance(const RotationDistance& rotationDistance)
    : rotation1(rotationDistance.rotation1)
    , rotation2(rotationDistance.rotation2)
{}

rotation::RotationDistance::RotationDistance(const iDynTree::Rotation& rotation1,
                                             const iDynTree::Rotation& rotation2)
    : rotation1(rotation1)
    , rotation2(rotation2)
{}

rotation::RotationDistance::RotationDistance(const iDynTree::Transform& transform1,
                                             const iDynTree::Transform& transform2)
    : rotation1(transform1.getRotation())
    , rotation2(transform2.getRotation())
{}

iDynTree::Rotation rotation::RotationDistance::asRotation() const
{
    iDynTree::Rotation distanceRotationMatrix = rotation1 * rotation2.inverse();
    return distanceRotationMatrix;
}

iDynTree::Vector3 rotation::RotationDistance::asRPY() const
{
    return this->asRotation().asRPY();
}

iDynTree::Vector4 rotation::RotationDistance::asQuaternion() const
{
    return this->asRotation().asQuaternion();
}

double rotation::RotationDistance::asEuclideanDistanceOfEulerAngles() const
{
    iDynTree::Vector3 rpy1;
    iDynTree::Vector3 rpy2;

    rpy1 = rotation1.asRPY();
    rpy2 = rotation2.asRPY();

    double euclideanDistance = 0;
    double angleDiff;
    for (unsigned int i = 0; i < 3; i++) {
        angleDiff = abs(rpy1.getVal(i) - rpy2.getVal(i));
        euclideanDistance += std::max(angleDiff, M_PI - angleDiff);
    }
    euclideanDistance = sqrt(euclideanDistance);

    return euclideanDistance;
}

iDynTree::Vector3 rotation::RotationDistance::asSkewVee() const
{
    iDynTree::Vector3 output;
    output = rotation::skewVee(this->asRotation());

    return output;
}

double rotation::RotationDistance::asTrace() const
{
    double output;
    iDynTree::Matrix3x3 matrix;

    iDynTree::toEigen(matrix) =
        iDynTree::toEigen(iDynTree::Rotation::Identity()) - iDynTree::toEigen(this->asRotation());
    output = matrix.getVal(0, 0) + matrix.getVal(1, 1) + matrix.getVal(2, 2);

    return output;
}

// =============
// STATE HELPERS
// =============

state::State::State()
{
    initializeState(0);
}

state::State::State(const int ndof)
{
    if (ndof < 0)
        initializeState(0);
    else
        initializeState(ndof);
}

bool state::State::initializeState(const int ndof)
{
    if (ndof < 0)
        return false;
    s.resize(ndof);
    dot_s.resize(ndof);
    dot_dot_s.resize(ndof);

    zero();
    return true;
}

void state::State::zero()
{
    s.zero();
    dot_s.zero();
    dot_dot_s.zero();
    W_p_B.zero();
    dot_W_p_B.zero();
    dot_dot_W_p_B.zero();
    W_R_B = iDynTree::Rotation::Identity();
    omega_B.zero();
    dot_omega_B.zero();
}

state::Integrator::Integrator()
    : Integrator(0.0)
{}

state::Integrator::Integrator(const unsigned int nJoints, const InterpolationType& interpolator)
    : nJoints(nJoints)
    , interpolator(interpolator)
{
    resetJointLimits();
    resetState();
}

state::Integrator::Integrator(const State& initialState, const InterpolationType& interpolator)
    : interpolator(interpolator)
{
    if (!(initialState.s.size() == initialState.dot_s.size())) {
        yError() << LogPrefixUtils << "invalid state. s and dot_s should have the same size";
    }

    nJoints = initialState.s.size();

    resetJointLimits();
    resetState();

    oldState = initialState;
}

state::Integrator::Integrator(const iDynTree::VectorDynSize& s,
                              const iDynTree::VectorDynSize& dot_s,
                              const iDynTree::Vector3& W_p_B,
                              const iDynTree::Vector3& dot_W_p_B,
                              const iDynTree::Rotation& W_R_B,
                              const iDynTree::Vector3& omega_B,
                              const InterpolationType& interpolator)
{
    State initialState;
    initialState.s = s;
    initialState.dot_s = dot_s;
    initialState.W_p_B = W_p_B;
    initialState.dot_W_p_B = dot_W_p_B;
    initialState.W_R_B = W_R_B;
    initialState.omega_B = omega_B;

    Integrator(initialState, interpolator);
}

void state::Integrator::setInterpolatorType(const InterpolationType& _interpolator)
{
    interpolator = _interpolator;
}

void state::Integrator::setNJoints(const unsigned int _nJoints)
{
    nJoints = _nJoints;
    oldState.s.resize(nJoints);
    oldState.dot_s.resize(nJoints);
    oldState.dot_dot_s.resize(nJoints);
    oldState.s.zero();
    oldState.dot_s.zero();
    oldState.dot_dot_s.zero();

    resetState();
    resetJointLimits();
}

void state::Integrator::resetState()
{
    oldState.s.resize(nJoints);
    oldState.dot_s.resize(nJoints);

    oldState.s.zero();
    oldState.dot_s.zero();
    oldState.W_p_B.zero();
    oldState.dot_W_p_B.zero();
    oldState.W_R_B = iDynTree::Rotation::Identity();
    oldState.omega_B.zero();
}

void state::Integrator::resetJointLimits()
{
    jointLimits.lowerLimits.resize(nJoints);
    jointLimits.upperLimits.resize(nJoints);
    jointLimits.active = false;
}

void state::Integrator::setState(const State& newState)
{
    if (!(newState.s.size() == nJoints) || !(newState.dot_s.size() == nJoints)) {
        yError() << LogPrefixUtils << "invalid state. s and dot_s should have size " << nJoints;
        return;
    }

    oldState = newState;
}

void state::Integrator::setState(const iDynTree::VectorDynSize& s,
                                 const iDynTree::VectorDynSize& dot_s,
                                 const iDynTree::Vector3& W_p_B,
                                 const iDynTree::Vector3& dot_W_p_B,
                                 const iDynTree::Rotation& W_R_B,
                                 const iDynTree::Vector3& omega_B)
{
    State newState;
    newState.s = s;
    newState.dot_s = dot_s;
    newState.W_p_B = W_p_B;
    newState.dot_W_p_B = dot_W_p_B;
    newState.W_R_B = W_R_B;
    newState.omega_B = omega_B;

    setState(newState);
}

void state::Integrator::setJointLimits(const iDynTree::VectorDynSize& lowerLimits,
                                       const iDynTree::VectorDynSize& upperLimits,
                                       const bool active)
{
    if (!((lowerLimits.size() == nJoints) && (upperLimits.size() == nJoints))) {
        yError() << LogPrefixUtils
                 << "invalid joint limits. lowerLimits and upperLimits should have size "
                 << nJoints;
        return;
    }

    jointLimits.upperLimits = upperLimits;
    jointLimits.lowerLimits = lowerLimits;
    jointLimits.active = active;
}

void state::Integrator::getState(State& _state) const
{
    getState(_state.s, _state.dot_s, _state.W_p_B, _state.dot_W_p_B, _state.W_R_B, _state.omega_B);
}

void state::Integrator::getState(iDynTree::VectorDynSize& s,
                                 iDynTree::VectorDynSize& dot_s,
                                 iDynTree::Vector3& W_p_B,
                                 iDynTree::Vector3& dot_W_p_B,
                                 iDynTree::Rotation& W_R_B,
                                 iDynTree::Vector3& omega_B) const
{
    s.resize(nJoints);
    dot_s.resize(nJoints);

    s = oldState.s;
    dot_s = oldState.dot_s;
    W_p_B = oldState.W_p_B;
    dot_W_p_B = oldState.dot_W_p_B;
    W_R_B = oldState.W_R_B;
    omega_B = oldState.omega_B;
}

void state::Integrator::getJointState(iDynTree::VectorDynSize& s,
                                      iDynTree::VectorDynSize& dot_s) const
{
    s.resize(nJoints);
    dot_s.resize(nJoints);
    s = oldState.s;
    dot_s = oldState.dot_s;
}

void state::Integrator::getBaseState(iDynTree::Vector3& W_p_B,
                                     iDynTree::Vector3& dot_W_p_B,
                                     iDynTree::Rotation& W_R_B,
                                     iDynTree::Vector3& omega_B) const
{
    W_p_B = oldState.W_p_B;
    dot_W_p_B = oldState.dot_W_p_B;
    W_R_B = oldState.W_R_B;
    omega_B = oldState.omega_B;
}

void state::Integrator::getJointConfiguration(iDynTree::VectorDynSize& s) const
{
    s.resize(nJoints);
    s = oldState.s;
}

void state::Integrator::getBasePose(iDynTree::Vector3& W_p_B, iDynTree::Rotation& W_R_B) const
{
    W_p_B = oldState.W_p_B;
    W_R_B = oldState.W_R_B;
}

void state::Integrator::getBasePose(iDynTree::Transform& W_T_B) const
{
    iDynTree::Position basePosition;
    iDynTree::toEigen(basePosition) = iDynTree::toEigen(oldState.W_p_B);
    W_T_B.setPosition(basePosition);
    W_T_B.setRotation(oldState.W_R_B);
}

void state::Integrator::integrate(const iDynTree::VectorDynSize& new_dot_s, const double dt)
{
    if (!(new_dot_s.size() == nJoints)) {
        yError() << LogPrefixUtils << "invalid dot_s. dot_s expected to have size " << nJoints;
        return;
    }

    switch (interpolator) {
        case InterpolationType::rectangular:
            for (int i = 0; i < new_dot_s.size(); i++) {
                oldState.s.setVal(i, oldState.s.getVal(i) + new_dot_s.getVal(i) * dt);
                if (jointLimits.active) {
                    oldState.s.setVal(i,
                                      saturate(oldState.s.getVal(i),
                                               jointLimits.lowerLimits.getVal(i),
                                               jointLimits.upperLimits.getVal(i)));
                }
            }
            break;
        case InterpolationType::trapezoidal:
            for (int i = 0; i < new_dot_s.size(); i++) {
                oldState.s.setVal(i,
                                  oldState.s.getVal(i)
                                      + (oldState.dot_s.getVal(i) + new_dot_s.getVal(i)) * dt / 2);
                if (jointLimits.active) {
                    oldState.s.setVal(i,
                                      saturate(oldState.s.getVal(i),
                                               jointLimits.lowerLimits.getVal(i),
                                               jointLimits.upperLimits.getVal(i)));
                }
            }
            break;
        default:
            yError() << LogPrefixUtils << "invalid integrator type";
            return;
    }

    oldState.dot_s = new_dot_s;
}

void state::Integrator::integrate(const iDynTree::VectorDynSize& new_dot_s,
                                  const iDynTree::Vector3& new_dot_W_p_B,
                                  const iDynTree::Vector3& new_omega_B,
                                  const double dt,
                                  bool &resetFlag)
{
    if(resetFlag) {
        resetFlag = false;
        std::cout << "Reset old state integrator" << std::endl;
        oldState.zero();
    }
    
    // integrate joints configuration
    integrate(new_dot_s, dt);

    // integrate base position
    switch (interpolator) {
        case InterpolationType::rectangular:
            for (int i = 0; i < 3; i++) {
                oldState.W_p_B.setVal(i, oldState.W_p_B.getVal(i) + new_dot_W_p_B.getVal(i) * dt);
            }
            break;
        case InterpolationType::trapezoidal:
            for (int i = 0; i < 3; i++) {
                oldState.W_p_B.setVal(i,
                                      oldState.W_p_B.getVal(i)
                                          + (oldState.dot_W_p_B.getVal(i) + new_dot_W_p_B.getVal(i))
                                                * dt / 2);
            }
            break;
        default:
            yError() << LogPrefixUtils << "invalid integrator type";
            return;
    }

    oldState.dot_W_p_B = new_dot_W_p_B;

    // integrate base orientation
    iDynTree::Matrix3x3 dot_W_R_B;
    iDynTree::toEigen(dot_W_R_B) =
        iDynTree::skew(iDynTree::toEigen(new_omega_B)) * iDynTree::toEigen(oldState.W_R_B);
    // TODO add correction for rotation matrix integration
    //
    iDynTree::toEigen(oldState.W_R_B) =
        iDynTree::toEigen(oldState.W_R_B) + iDynTree::toEigen(dot_W_R_B) * dt;
    // orthonormalization using quaternion
    iDynTree::Vector4 quaternion = oldState.W_R_B.asQuaternion();
    oldState.W_R_B.fromQuaternion(quaternion);
    oldState.dot_omega_B = new_omega_B;
}

double
state::Integrator::saturate(const double val, const double lowerLimit, const double upperLimit)
{
    if (val > upperLimit)
        return upperLimit;
    else if (val < lowerLimit)
        return lowerLimit;
    else
        return val;
}
