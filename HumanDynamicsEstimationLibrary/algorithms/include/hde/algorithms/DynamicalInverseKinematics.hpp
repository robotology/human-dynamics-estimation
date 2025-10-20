// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef DYNAMICALINVERSEKINEMATICS_HPP
#define DYNAMICALINVERSEKINEMATICS_HPP

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/Model.h>

#include <map>
#include <memory>

namespace hde::algorithms {
    class DynamicalInverseKinematics;
} // namespace hde

class hde::algorithms::DynamicalInverseKinematics
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:

    DynamicalInverseKinematics();
    ~DynamicalInverseKinematics();

    bool setModel(const iDynTree::Model& model);
    bool setFloatingBaseOnFrameNamed(const std::string& floatingBaseFrameName);
    bool setInverseVelocityKinematicsResolutionMode(const std::string& resolutionModeName);
    void setInverseVelocityKinematicsRegularization(const double regularizationWeight);

    bool setLinearJointConfigurationLimits(const std::vector<iDynTree::JointIndex>& jointsIndexList,
                                           const iDynTree::VectorDynSize& upperBoundary,
                                           const iDynTree::VectorDynSize& lowerBoundary,
                                           const iDynTree::MatrixDynSize& customConstraintMatrix);
    bool setConstraintParametersJointValues(const double& k_u, const double& k_l);
    bool setAllJointsVelocityLimit(const double limit);
    bool setBaseVelocityLimit(const iDynTree::VectorDynSize& lowerLimit,
                              const iDynTree::VectorDynSize& upperLimit);
    bool setJointVelocityLimit(const iDynTree::JointIndex jointIndex, const double limit);

    bool addPoseAndVelocityTarget(const std::string& linkName,
                                  const iDynTree::Transform& transform,
                                  const iDynTree::Twist& twist,
                                  const std::array<bool, 3> positionTargetActive,
                                  const std::array<bool, 3> orientationTargetActive,
                                  const double positionFeedbackGain = 1.0,
                                  const double orientationFeedbackGain = 1.0,
                                  const double linearVelocityFeedforwardGain = 1.0,
                                  const double angularVelocityFeedforwardGain = 1.0,
                                  const double linearVelocityWeight = 1.0,
                                  const double angularVelocityWeight = 1.0);
    bool addPoseTarget(const std::string& linkName,
                       const iDynTree::Transform& transform,
                       const std::array<bool, 3> positionTargetActive,
                       const std::array<bool, 3> orientationTargetActive,
                       const double positionFeedbackGain = 1.0,
                       const double orientationFeedbackGain = 1.0,
                       const double linearVelocityWeight = 1.0,
                       const double angularVelocityWeight = 1.0);
    bool addPoseAndVelocityTarget(const std::string& linkName,
                                  const iDynTree::Vector3& position,
                                  const iDynTree::Rotation& orientation,
                                  const iDynTree::Vector3& linearVelocity,
                                  const iDynTree::Vector3& angularVelocity,
                                  const std::array<bool, 3> positionTargetActive,
                                  const std::array<bool, 3> orientationTargetActive,
                                  const double positionFeedbackGain = 1.0,
                                  const double orientationFeedbackGain = 1.0,
                                  const double linearVelocityFeedforwardGain = 1.0,
                                  const double angularVelocityFeedforwardGain = 1.0,
                                  const double linearVelocityWeight = 1.0,
                                  const double angularVelocityWeight = 1.0);
    bool addPoseTarget(const std::string& linkName,
                       const iDynTree::Vector3& position,
                       const iDynTree::Rotation& orientation,
                       const std::array<bool, 3> positionTargetActive,
                       const std::array<bool, 3> orientationTargetActive,
                       const double positionFeedbackGain = 1.0,
                       const double orientationFeedbackGain = 1.0,
                       const double linearVelocityWeight = 1.0,
                       const double angularVelocityWeight = 1.0);
    bool addPositionAndVelocityTarget(const std::string& linkName,
                                      const iDynTree::Vector3& position,
                                      const iDynTree::Vector3& linearVelocity,
                                      const std::array<bool, 3> positionTargetActive,
                                      const double positionFeedbackGain = 1.0,
                                      const double linearVelocityFeedforwardGain = 1.0,
                                      const double linearVelocityWeight = 1.0);
    bool addPositionTarget(const std::string& linkName,
                           const iDynTree::Vector3& position,
                           const std::array<bool, 3> positionTargetActive,
                           const double positionFeedbackGain = 1.0,
                           const double linearVelocityWeight = 1.0);
    bool addOrientationAndVelocityTarget(const std::string& linkName,
                                         const iDynTree::Rotation& orientation,
                                         const iDynTree::Vector3& angularVelocity,
                                         const std::array<bool, 3> orientationTargetActive,
                                         const double orientationFeedbackGain = 1.0,
                                         const double angularVelocityFeedforwardGain = 1.0,
                                         const double angularVelocityWeight = 1.0);
    bool addOrientationTarget(const std::string& linkName,
                              const iDynTree::Rotation& orientation,
                              const std::array<bool, 3> orientationTargetActive,
                              const double orientationFeedbackGain = 1.0,
                              const double angularVelocityWeight = 1.0);

    bool updateTarget(const std::string& linkName,
                      const iDynTree::Transform& transform,
                      const iDynTree::Twist& twist,
                      const double positionFeedbackGain,
                      const double orientationFeedbackGain,
                      const double linearVelocityFeedforwardGain,
                      const double angularVelocityFeedforwardGain,
                      const double linearVelocityWeight,
                      const double angularVelocityWeight);
    bool updateTarget(const std::string& linkName,
                      const iDynTree::Vector3& position,
                      const iDynTree::Rotation& orientation,
                      const iDynTree::Vector3& linearVelocity,
                      const iDynTree::Vector3& angularVelocity,
                      const double positionFeedbackGain,
                      const double orientationFeedbackGain,
                      const double linearVelocityFeedforwardGain,
                      const double angularVelocityFeedforwardGain,
                      const double linearVelocityWeight,
                      const double angularVelocityWeight);
    bool updateTargetPoseAndVelocity(const std::string& linkName,
                                     const iDynTree::Transform& transform,
                                     const iDynTree::Twist& twist);
    bool updateTargetPoseAndVelocity(const std::string& linkName,
                                     const iDynTree::Vector3& position,
                                     const iDynTree::Rotation& orientation,
                                     const iDynTree::Vector3& linearVelocity,
                                     const iDynTree::Vector3& angularVelocity);
    bool updateTargetPose(const std::string& linkName,
                          const iDynTree::Transform& transform);
    bool updateTargetPose(const std::string& linkName,
                          const iDynTree::Vector3& position,
                          const iDynTree::Rotation& orientation);
    bool updateTargetPosition(const std::string& linkName,
                              const iDynTree::Vector3& position);
    bool updateTargetPositionAndVelocity(const std::string& linkName,
                                         const iDynTree::Vector3& position,
                                         const iDynTree::Vector3& linearVelocity);
    bool updateTargetOrientation(const std::string& linkName,
                                 const iDynTree::Rotation& orientation);
    bool updateTargetOrientationAndVelocity(const std::string& linkName,
                                            const iDynTree::Rotation& orientation,
                                            const iDynTree::Vector3& angularVelocity);
    bool updateTargetLinearVelocity(const std::string& linkName,
                                    const iDynTree::Vector3& linearVelocity);
    bool updateTargetAngularVelocity(const std::string& linkName,
                                     const iDynTree::Vector3& angularVelocity);
    bool updateTargetGains(const std::string& linkName,
                           const double positionFeedbackGain,
                           const double orientationFeedbackGain,
                           const double linearVelocityFeedforwardGain,
                           const double angularVelocityFeedforwardGain);
    bool updateTargetWeights(const std::string& linkName,
                            const double linearVelocityWeight,
                            const double angularVelocityWeight);
    bool updatePositionFeedbackGain(const std::string& linkName, const double positionFeedbackGain);
    bool updateOrientationFeedbackGain(const std::string& linkName, const double orientationFeedbackGain);
    bool updateLinearVelocityFeedforwardGain(const std::string& linkName, const double linearVelocityFeedforwardGain);
    bool updateAngularVelocityFeedforwardGain(const std::string& linkName, const double angularVelocityFeedforwardGain);
    bool updateLinearVelocityWeight(const std::string& linkName, const double linearVelocityWeight);
    bool updateAngularVelocityWeight(const std::string& linkName, const double angularVelocityWeight);

    bool updatePositionTargetAxis(const std::string& linkName,
                                  const std::array<bool, 3> positionTargetActive);
    bool updateOrientationTargetAxis(const std::string& linkName,
                                     const std::array<bool, 3> orientationTargetActive);

    bool setJointConfiguration(const std::string& jointName, const double jointConfiguration);
    bool setJointsConfiguration(const iDynTree::VectorDynSize& jointsConfiguration);
    bool setBasePose(const iDynTree::Transform& baseTransform);
    bool setBasePose(const iDynTree::Vector3& basePosition, const iDynTree::Rotation& baseRotation);
    bool setConfiguration(const iDynTree::Transform& baseTransform,
                          const iDynTree::VectorDynSize& jointsConfiguration);
    bool setConfiguration(const iDynTree::Vector3& basePosition,
                          const iDynTree::Rotation& baseRotation,
                          const iDynTree::VectorDynSize& jointsConfiguration);

    bool getJointsConfigurationSolution(iDynTree::VectorDynSize& jointsConfiguration) const;
    bool getBasePoseSolution(iDynTree::Transform& baseTransform) const;
    bool getBasePoseSolution(iDynTree::Vector3& basePosition, iDynTree::Rotation& baseRotation) const;
    bool getConfigurationSolution(iDynTree::Transform& baseTransform,
                                  iDynTree::VectorDynSize& jointsConfiguration) const;
    bool getConfigurationSolution(iDynTree::Vector3& basePosition,
                                  iDynTree::Rotation& baseRotation,
                                  iDynTree::VectorDynSize& jointsConfiguration) const;
    bool getVelocitySolution(iDynTree::Twist& baseVelocity,
                             iDynTree::VectorDynSize& jointsVelocity) const;
    bool getJointsVelocitySolution(iDynTree::VectorDynSize& jointsVelocity) const;
    bool getBaseVelocitySolution(iDynTree::Twist& baseVelocity) const;
    bool getBaseVelocitySolution(iDynTree::Vector3& linearVelocity,
                                 iDynTree::Vector3& angularVelocity) const;

    void clearProblem();

    bool solve(const double dt, const bool reset = false);

    double getTargetsMeanOrientationErrorNorm() const;
    double getTargetsMeanPositionErrorNorm() const;
};

#endif // DYNAMICALINVERSEKINEMATICS_HPP
