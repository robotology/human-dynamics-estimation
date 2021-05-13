/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

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

        class rotationDistance;
    } // namespace rotation

    namespace state {
        struct state
        {
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
        class integrator;
    } // namespace state

    class rotation::rotationDistance
    {
    private:
        iDynTree::Rotation rotation1;
        iDynTree::Rotation rotation2;

    public:
        rotationDistance();
        rotationDistance(const rotationDistance& rotationDistance);
        rotationDistance(const iDynTree::Rotation rotation1, const iDynTree::Rotation rotation2);
        rotationDistance(const iDynTree::Transform transform1,
                         const iDynTree::Transform transform2);

        iDynTree::Rotation asRotation();
        iDynTree::Vector3 asRPY();
        iDynTree::Vector4 asQuaternion();
        iDynTree::Vector3 asSkewVee();

        double asEuclideanDistanceOfEulerAngles();
        double asTrace();
    };

    class state::integrator
    {
    public:
        typedef enum
        {
            rectangular,
            trapezoidal
        } interpolationType;

        integrator();
        integrator(unsigned int nJoints, interpolationType interpolator = rectangular);
        integrator(state initialState, interpolationType interpolator = rectangular);
        integrator(iDynTree::VectorDynSize s,
                   iDynTree::VectorDynSize dot_s,
                   iDynTree::Vector3 W_p_B,
                   iDynTree::Vector3 dot_W_p_B,
                   iDynTree::Rotation W_R_B,
                   iDynTree::Vector3 omega_B,
                   interpolationType interpolator = rectangular);

        void setInterpolatorType(interpolationType interpolator);
        void setNJoints(unsigned int nJoints);

        void setState(state state);
        void setState(iDynTree::VectorDynSize s,
                      iDynTree::VectorDynSize dot_s,
                      iDynTree::Vector3 W_p_B,
                      iDynTree::Vector3 dot_W_p_B,
                      iDynTree::Rotation W_R_B,
                      iDynTree::Vector3 omega_B);

        void setJointLimits(iDynTree::VectorDynSize lowerLimits,
                            iDynTree::VectorDynSize upperLimits,
                            bool active = true);

        void getState(state& state);
        void getState(iDynTree::VectorDynSize& s,
                      iDynTree::VectorDynSize& dot_s,
                      iDynTree::Vector3& W_p_B,
                      iDynTree::Vector3& dot_W_p_B,
                      iDynTree::Rotation& W_R_B,
                      iDynTree::Vector3& omega_B);
        void getJointState(iDynTree::VectorDynSize& s, iDynTree::VectorDynSize& dot_s);
        void getBaseState(iDynTree::Vector3& W_p_B,
                          iDynTree::Vector3& dot_W_p_B,
                          iDynTree::Rotation& W_R_B,
                          iDynTree::Vector3& omega_B);
        void getJointConfiguration(iDynTree::VectorDynSize& s);
        void getBasePose(iDynTree::Vector3& W_p_B, iDynTree::Rotation& W_R_B);
        void getBasePose(iDynTree::Transform& W_T_B);

        void integrate(iDynTree::VectorDynSize dot_s, double dt);
        void integrate(iDynTree::VectorDynSize dot_s,
                       iDynTree::Vector3 dot_W_p_B,
                       iDynTree::Vector3 omega_B,
                       double dt);

    private:
        unsigned int nJoints;
        state oldState;
        interpolationType interpolator;

        typedef struct
        {
            bool active;
            iDynTree::VectorDynSize lowerLimits;
            iDynTree::VectorDynSize upperLimits;
        } JointLimits;
        JointLimits jointLimits;

        void resetState();
        void resetJointLimits();
        double saturate(double val, double lowerLimit, double upperLimit);
    };

} // namespace hde::utils::idyntree

#endif
