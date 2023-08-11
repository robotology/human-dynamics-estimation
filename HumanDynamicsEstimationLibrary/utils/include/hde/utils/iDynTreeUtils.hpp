// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HUMANDYNAMICSESTIMATION_UTILS_HPP
#define HUMANDYNAMICSESTIMATION_UTILS_HPP

// iDynTree
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>

#include <vector>

/**
 * Helper for iDynTree library.
 */
namespace hde::utils::idyntree {

    namespace rotation {
        /**
         * Transform a 3x3 matrix into a skew-symmetric matrix.
         * @param input is a 3x3 matrix;
         * @return a 3x3 skew-symmetric matrix
         */
        iDynTree::Matrix3x3 skewSymmetric(const iDynTree::Matrix3x3& input);

        iDynTree::Vector3 skewVee(const iDynTree::Matrix3x3& input);

        class RotationDistance;
    } // namespace rotation

    namespace state {
        class State;
        class Integrator;
    } // namespace state
} // namespace hde::utils::idyntree

class hde::utils::idyntree::rotation::RotationDistance
{
private:
    iDynTree::Rotation rotation1;
    iDynTree::Rotation rotation2;

public:
    RotationDistance() = default;
    RotationDistance(const RotationDistance& rotationDistance);
    RotationDistance(const iDynTree::Rotation& rotation1, const iDynTree::Rotation& rotation2);
    RotationDistance(const iDynTree::Transform& transform1, const iDynTree::Transform& transform2);

    iDynTree::Rotation asRotation() const;
    iDynTree::Vector3 asRPY() const;
    iDynTree::Vector4 asQuaternion() const;
    iDynTree::Vector3 asSkewVee() const;

    double asEuclideanDistanceOfEulerAngles() const;
    double asTrace() const;
};

class hde::utils::idyntree::state::State
{
    public:

    State();
    State(const int ndof);

    bool initializeState(const int ndof);
    void zero();

    iDynTree::VectorDynSize s;
    iDynTree::VectorDynSize dot_s;
    iDynTree::VectorDynSize dot_dot_s;
    iDynTree::Vector3 W_p_B;
    iDynTree::Vector3 dot_W_p_B;
    iDynTree::Vector3 dot_dot_W_p_B;
    iDynTree::Rotation W_R_B;
    iDynTree::Vector3 omega_B;
    iDynTree::Vector3 dot_omega_B;
};

class hde::utils::idyntree::state::Integrator
{
public:
    enum class InterpolationType
    {
        rectangular,
        trapezoidal
    };

    Integrator();
    Integrator(const unsigned int nJoints, const InterpolationType& interpolator = InterpolationType::rectangular);
    Integrator(const State& initialState, const InterpolationType& interpolator = InterpolationType::rectangular);
    Integrator(const iDynTree::VectorDynSize& s,
               const iDynTree::VectorDynSize& dot_s,
               const iDynTree::Vector3& W_p_B,
               const iDynTree::Vector3& dot_W_p_B,
               const iDynTree::Rotation& W_R_B,
               const iDynTree::Vector3& omega_B,
               const InterpolationType& interpolator = InterpolationType::rectangular);

    void setInterpolatorType(const InterpolationType& interpolator);
    void setNJoints(const unsigned int nJoints);

    void setState(const State& state);
    void setState(const iDynTree::VectorDynSize& s,
                  const iDynTree::VectorDynSize& dot_s,
                  const iDynTree::Vector3& W_p_B,
                  const iDynTree::Vector3& dot_W_p_B,
                  const iDynTree::Rotation& W_R_B,
                  const iDynTree::Vector3& omega_B);

    void setJointLimits(const iDynTree::VectorDynSize& lowerLimits,
                        const iDynTree::VectorDynSize& upperLimits,
                        const bool active = true);

    void getState(State& state) const;
    void getState(iDynTree::VectorDynSize& s,
                  iDynTree::VectorDynSize& dot_s,
                  iDynTree::Vector3& W_p_B,
                  iDynTree::Vector3& dot_W_p_B,
                  iDynTree::Rotation& W_R_B,
                  iDynTree::Vector3& omega_B) const;
    void getJointState(iDynTree::VectorDynSize& s, iDynTree::VectorDynSize& dot_s) const;
    void getBaseState(iDynTree::Vector3& W_p_B,
                      iDynTree::Vector3& dot_W_p_B,
                      iDynTree::Rotation& W_R_B,
                      iDynTree::Vector3& omega_B) const;
    void getJointConfiguration(iDynTree::VectorDynSize& s) const;
    void getBasePose(iDynTree::Vector3& W_p_B, iDynTree::Rotation& W_R_B) const;
    void getBasePose(iDynTree::Transform& W_T_B) const;

    void integrate(const iDynTree::VectorDynSize& dot_s,  const double dt);
    void integrate(const iDynTree::VectorDynSize& dot_s,
                   const iDynTree::Vector3& dot_W_p_B,
                   const iDynTree::Vector3& omega_B,
                   const double dt);

private:
    unsigned int nJoints;
    State oldState;
    InterpolationType interpolator;

    struct
    {
        bool active;
        iDynTree::VectorDynSize lowerLimits;
        iDynTree::VectorDynSize upperLimits;
    } jointLimits;

    void resetState();
    void resetJointLimits();
    double saturate(const double val, const double lowerLimit, const double upperLimit);
};

#endif
