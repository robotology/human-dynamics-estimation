/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "Utils.hpp"

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

rotation::RotationDistance::RotationDistance(const iDynTree::Rotation rotation1,
                                             const iDynTree::Rotation rotation2)
    : rotation1(rotation1)
    , rotation2(rotation2)
{}

rotation::RotationDistance::RotationDistance(const iDynTree::Transform transform1,
                                             const iDynTree::Transform transform2)
    : rotation1(transform1.getRotation())
    , rotation2(transform2.getRotation())
{}

iDynTree::Rotation rotation::RotationDistance::asRotation()
{
    iDynTree::Rotation distanceRotationMatrix = rotation1 * rotation2.inverse();
    return distanceRotationMatrix;
}

iDynTree::Vector3 rotation::RotationDistance::asRPY()
{
    return this->asRotation().asRPY();
}

iDynTree::Vector4 rotation::RotationDistance::asQuaternion()
{
    return this->asRotation().asQuaternion();
}

double rotation::RotationDistance::asEuclideanDistanceOfEulerAngles()
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

iDynTree::Vector3 rotation::RotationDistance::asSkewVee()
{
    iDynTree::Vector3 output;
    output = rotation::skewVee(this->asRotation());

    return output;
}

double rotation::RotationDistance::asTrace()
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

state::Integrator::Integrator()
    : Integrator(0.0)
{}

state::Integrator::Integrator(unsigned int nJoints, interpolationType interpolator)
    : nJoints(nJoints)
    , interpolator(interpolator)
{
    resetJointLimits();
    resetState();
}

state::Integrator::Integrator(state initialState, interpolationType interpolator)
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

state::Integrator::Integrator(iDynTree::VectorDynSize s,
                              iDynTree::VectorDynSize dot_s,
                              iDynTree::Vector3 W_p_B,
                              iDynTree::Vector3 dot_W_p_B,
                              iDynTree::Rotation W_R_B,
                              iDynTree::Vector3 omega_B,
                              interpolationType interpolator)
{
    state initialState;
    initialState.s = s;
    initialState.dot_s = dot_s;
    initialState.W_p_B = W_p_B;
    initialState.dot_W_p_B = dot_W_p_B;
    initialState.W_R_B = W_R_B;
    initialState.omega_B = omega_B;

    Integrator(initialState, interpolator);
}

void state::Integrator::setInterpolatorType(interpolationType _interpolator)
{
    interpolator = _interpolator;
}

void state::Integrator::setNJoints(unsigned int _nJoints)
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

void state::Integrator::setState(state newState)
{
    if (!(newState.s.size() == nJoints) || !(newState.dot_s.size() == nJoints)) {
        yError() << LogPrefixUtils << "invalid state. s and dot_s should have size " << nJoints;
    }

    oldState = newState;
}

void state::Integrator::setState(iDynTree::VectorDynSize s,
                                 iDynTree::VectorDynSize dot_s,
                                 iDynTree::Vector3 W_p_B,
                                 iDynTree::Vector3 dot_W_p_B,
                                 iDynTree::Rotation W_R_B,
                                 iDynTree::Vector3 omega_B)
{
    state newState;
    newState.s = s;
    newState.dot_s = dot_s;
    newState.W_p_B = W_p_B;
    newState.dot_W_p_B = dot_W_p_B;
    newState.W_R_B = W_R_B;
    newState.omega_B = omega_B;

    setState(newState);
}

void state::Integrator::setJointLimits(iDynTree::VectorDynSize lowerLimits,
                                       iDynTree::VectorDynSize upperLimits,
                                       bool active)
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

void state::Integrator::getState(state& _state)
{
    getState(_state.s, _state.dot_s, _state.W_p_B, _state.dot_W_p_B, _state.W_R_B, _state.omega_B);
}

void state::Integrator::getState(iDynTree::VectorDynSize& s,
                                 iDynTree::VectorDynSize& dot_s,
                                 iDynTree::Vector3& W_p_B,
                                 iDynTree::Vector3& dot_W_p_B,
                                 iDynTree::Rotation& W_R_B,
                                 iDynTree::Vector3& omega_B)
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

void state::Integrator::getJointState(iDynTree::VectorDynSize& s, iDynTree::VectorDynSize& dot_s)
{
    s.resize(nJoints);
    dot_s.resize(nJoints);
    s = oldState.s;
    dot_s = oldState.dot_s;
}

void state::Integrator::getBaseState(iDynTree::Vector3& W_p_B,
                                     iDynTree::Vector3& dot_W_p_B,
                                     iDynTree::Rotation& W_R_B,
                                     iDynTree::Vector3& omega_B)
{
    W_p_B = oldState.W_p_B;
    dot_W_p_B = oldState.dot_W_p_B;
    W_R_B = oldState.W_R_B;
    omega_B = oldState.omega_B;
}

void state::Integrator::getJointConfiguration(iDynTree::VectorDynSize& s)
{
    s.resize(nJoints);
    s = oldState.s;
}

void state::Integrator::getBasePose(iDynTree::Vector3& W_p_B, iDynTree::Rotation& W_R_B)
{
    W_p_B = oldState.W_p_B;
    W_R_B = oldState.W_R_B;
}

void state::Integrator::getBasePose(iDynTree::Transform& W_T_B)
{
    iDynTree::Position basePosition;
    iDynTree::toEigen(basePosition) = iDynTree::toEigen(oldState.W_p_B);
    W_T_B.setPosition(basePosition);
    W_T_B.setRotation(oldState.W_R_B);
}

void state::Integrator::integrate(iDynTree::VectorDynSize new_dot_s, double dt)
{
    if (!(new_dot_s.size() == nJoints)) {
        yError() << LogPrefixUtils << "invalid dot_s. dot_s expected to have size " << nJoints;
    }

    if (interpolator == rectangular) {
        for (int i = 0; i < new_dot_s.size(); i++) {
            oldState.s.setVal(i, oldState.s.getVal(i) + new_dot_s.getVal(i) * dt);
            if (jointLimits.active) {
                oldState.s.setVal(i,
                                  saturate(oldState.s.getVal(i),
                                           jointLimits.lowerLimits.getVal(i),
                                           jointLimits.upperLimits.getVal(i)));
            }
        }
    }
    else if (interpolator == trapezoidal) {
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
    }
    else {
        yError() << LogPrefixUtils << "invalid integrator type " << interpolator;
    }
    oldState.dot_s = new_dot_s;
}

void state::Integrator::integrate(iDynTree::VectorDynSize new_dot_s,
                                  iDynTree::Vector3 new_dot_W_p_B,
                                  iDynTree::Vector3 new_omega_B,
                                  double dt)
{
    // integrate joints configuration
    integrate(new_dot_s, dt);

    // integrate base position
    if (interpolator == rectangular) {
        for (int i = 0; i < 3; i++) {
            oldState.W_p_B.setVal(i, oldState.W_p_B.getVal(i) + new_dot_W_p_B.getVal(i) * dt);
        }
    }
    else if (interpolator == trapezoidal) {
        for (int i = 0; i < 3; i++) {
            oldState.W_p_B.setVal(i,
                                  oldState.W_p_B.getVal(i)
                                      + (oldState.dot_W_p_B.getVal(i) + new_dot_W_p_B.getVal(i))
                                            * dt / 2);
        }
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

double state::Integrator::saturate(double val, double lowerLimit, double upperLimit)
{
    if (val > upperLimit)
        return upperLimit;
    else if (val < lowerLimit)
        return lowerLimit;
    else
        return val;
}
