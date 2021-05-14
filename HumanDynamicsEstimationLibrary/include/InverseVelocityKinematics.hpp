/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef INVERSEVELOCITYKINEMATICS_HPP
#define INVERSEVELOCITYKINEMATICS_HPP

#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SparseCholesky>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>

#include <map>
#include <memory>
#include <yarp/os/LogStream.h>

namespace hde {
    class InverseVelocityKinematics;
} // namespace hde

class hde::InverseVelocityKinematics
{
protected:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    enum class InverseVelocityKinematicsResolutionMode
    {
        QP,
        moorePenrose,
        completeOrthogonalDecomposition,
        leastSquare,
        choleskyDecomposition,
        sparseCholeskyDecomposition,
        robustCholeskyDecomposition,
        sparseRobustCholeskyDecomposition,
    };

    InverseVelocityKinematics();
    ~InverseVelocityKinematics();

    bool setModel(const iDynTree::Model& model);
    bool setFloatingBaseOnFrameNamed(const std::string& floatingBaseFrameName);
    bool setResolutionMode(const InverseVelocityKinematicsResolutionMode& resolutionMode);
    bool setResolutionMode(const std::string& resolutionModeName);
    void setRegularization(const double regularizationWeight);

    bool addTarget(const std::string& linkName,
                   const iDynTree::Vector3& linearVelocity,
                   const iDynTree::Vector3& angularVelocity,
                   const double linearWeight = 1.0,
                   const double angularWeight = 1.0);
    bool addTarget(const std::string& linkName,
                   const iDynTree::Twist& twist,
                   const double linearWeight = 1.0,
                   const double angularWeight = 1.0);
    bool addLinearVelocityTarget(const std::string& linkName,
                                 const iDynTree::Vector3& linearVelocity,
                                 const double linearWeight = 1.0);
    bool addLinearVelocityTarget(const std::string& linkName,
                                 const iDynTree::Twist& twist,
                                 const double linearWeight = 1.0);
    bool addAngularVelocityTarget(const std::string& linkName,
                                  const iDynTree::Vector3& angularVelocity,
                                  const double angularWeight = 1.0);
    bool addAngularVelocityTarget(const std::string& linkName,
                                  const iDynTree::Twist& twist,
                                  const double angularWeight = 1.0);

    // TODO
    // addFrameVelocityConstraint
    // addFrameLinearVelocityConstraint
    // addFrameAngularVelocityConstraint

    bool setJointConfiguration(const std::string& jointName, const double jointConfiguration);
    bool setJointsConfiguration(const iDynTree::VectorDynSize& jointsConfiguration);
    bool setBasePose(const iDynTree::Transform& baseTransform);
    bool setBasePose(const iDynTree::Vector3& basePosition, const iDynTree::Rotation& baseRotation);
    bool setConfiguration(const iDynTree::Transform& baseTransform,
                          const iDynTree::VectorDynSize& jointsConfiguration);
    bool setConfiguration(const iDynTree::Vector3& basePosition,
                          const iDynTree::Rotation& baseRotation,
                          const iDynTree::VectorDynSize& jointsConfiguration);

    // TODO
    bool setCustomBaseVelocityLimit(const iDynTree::VectorDynSize& lowerBound,
                                    const iDynTree::VectorDynSize& upperBound);
    // bool setBaseLinearVelocityLimit();
    // bool setBaseAngularVelocityLimit();
    // bool setJointVelocityLimit(std::string jointName, double jointLimit);
    bool setCustomJointsVelocityLimit(const std::vector<iDynTree::JointIndex>& jointsIndexList,
                                      const iDynTree::VectorDynSize& jointsLimitList);
    bool setCustomConstraintsJointsValues(const std::vector<iDynTree::JointIndex>& jointsIndexList,
                                          const iDynTree::VectorDynSize& upperBoundary,
                                          const iDynTree::VectorDynSize& lowerBoundary,
                                          const iDynTree::MatrixDynSize& customConstraintMatrix,
                                          const double k_u,
                                          const double k_l);

    bool setGeneralJointVelocityConstraints(const double jointVelocityLimit);

    bool setGeneralJointsUpperLowerConstraints(const iDynTree::VectorDynSize& jointUpperLimits,
                                               const iDynTree::VectorDynSize& jointLowerLimits);

    bool updateTarget(const std::string& linkName,
                      const iDynTree::Vector3& linearVelocity,
                      const iDynTree::Vector3& angularVelocity,
                      const double linearWeight = 1.0,
                      const double angularWeight = 1.0);
    bool updateTarget(const std::string& linkName,
                      const iDynTree::Twist& twist,
                      const double linearWeight = 1.0,
                      const double angularWeight = 1.0);
    bool updateLinearVelocityTarget(const std::string& linkName,
                                    const iDynTree::Vector3& linearVelocity,
                                    const double linearWeight = 1.0);
    bool updateAngularVelocityTarget(const std::string& linkName,
                                     const iDynTree::Vector3& angularVelocity,
                                     const double angularWeight = 1.0);

    bool getVelocitySolution(iDynTree::Twist& baseVelocity,
                             iDynTree::VectorDynSize& jointsVelocity) const;
    bool getJointsVelocitySolution(iDynTree::VectorDynSize& jointsVelocity) const;
    bool getBaseVelocitySolution(iDynTree::Twist& baseVelocity) const;
    bool getBaseVelocitySolution(iDynTree::Vector3& linearVelocity,
                                 iDynTree::Vector3& angularVelocity) const;

    bool solve();
    void clearProblem();
};

#endif // INVERSEVELOCITYKINEMATICS_HPP
