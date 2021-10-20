/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "hde/algorithms/DynamicalInverseKinematics.hpp"

#include <hde/utils/iDynTreeUtils.hpp>
#include <hde/algorithms/InverseVelocityKinematics.hpp>


// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// idyntree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>

// yarp
#include <yarp/os/LogStream.h>

using namespace hde::algorithms;

class DynamicalInverseKinematics::impl
{

public:

    impl();

    class InverseKinematicsTarget;
    typedef std::map<int, InverseKinematicsTarget> TargetsMap;
    TargetsMap targets;

    InverseVelocityKinematics inverseVelocityKinematics;
    hde::utils::idyntree::state::Integrator stateIntegrator;

    iDynTree::Model model;
    iDynTree::KinDynComputations dynamics;
    size_t dofs;
     
    hde::utils::idyntree::state::State state;
    
    iDynTree::Vector3 worldGravity;

    bool isInverseVelocityKinematicsInitialized;
    bool isInverseKinematicsInitializd;

    bool addTarget(const InverseKinematicsTarget& target);

    void updateTargetPosition(const TargetsMap::iterator& target,
                              const iDynTree::Vector3& newPosition,
                              const double newPositionTargetWeight);
    void updateTargetOrientation(const TargetsMap::iterator& target,
                              const iDynTree::Rotation& newOrientation,
                              const double newOrientationTargetWeight);
    void updateTargetLinearVelocity(const TargetsMap::iterator& target,
                                    const iDynTree::Vector3& newLinearVelocity,
                                    const double newLinearVelocityWeight);
    void updateTargetAngularVelocity(const TargetsMap::iterator& target,
                                     const iDynTree::Vector3& newAngularVelocity,
                                     const double newAngularVelocityWeight);

    TargetsMap::iterator getTargetRefIfItExists(const std::string& targetFrameName);

    bool initializeInverseVelocityKinematics();
    bool updateConfiguration();
};

// ===========================
// INVERSE KINEAMATICS TARGETS
// ===========================

class DynamicalInverseKinematics::impl::InverseKinematicsTarget
{
public:
    enum TargetType
    {
        TargetTypePosition,
        TargetTypeOrientation,
        TargetTypePose,
    };

    iDynTree::Transform transform;
    iDynTree::Twist twist;
    TargetType type;
    std::string frameName;

    double positionTargetWeight;
    double orientationTargetWeight;
    double linearVelocityWeight;
    double angularVelocityWeight;

    InverseKinematicsTarget(const std::string& frameName, const TargetType& configuration);

    static InverseKinematicsTarget PositionAndVelocityTarget(const std::string& frameName,
                                                             const iDynTree::Vector3& position,
                                                             const iDynTree::Vector3& linearVelocity,
                                                             const double positionTargetWeight = 1.0,
                                                             const double linearVeloictyWeight = 1.0);
    static InverseKinematicsTarget PositionTarget(const std::string& frameName,
                                                  const iDynTree::Vector3& position,
                                                  const double positionTargetWeight = 1.0);
    static InverseKinematicsTarget OrientationAndVelocityTarget(const std::string& frameName,
                                                                const iDynTree::Rotation& rotation,
                                                                const iDynTree::Vector3& angularVelocity,
                                                                const double orientationTargetWeight = 1.0,
                                                                const double angularVelocityWeight = 1.0);
    static InverseKinematicsTarget OrientationTarget(const std::string& frameName,
                                                     const iDynTree::Rotation& rotation,
                                                     const double orientationTargetWeight = 1.0);
    static InverseKinematicsTarget PoseAndVelocityTarget(const std::string& frameName,
                                                         const iDynTree::Transform& transform,
                                                         const iDynTree::Twist& twist,
                                                         const double positionTargetWeight = 1.0,
                                                         const double orientationTargetWeight = 1.0,
                                                         const double linearVelocityWeight = 1.0,
                                                         const double angularVelocityWeight = 1.0);
    static InverseKinematicsTarget PoseTarget(const std::string& frameName,
                                              const iDynTree::Transform& transform,
                                              const double positionTargetWeight = 1.0,
                                              const double orientationTargetWeight = 1.0);
    static InverseKinematicsTarget PoseAndVelocityTarget(const std::string& frameName,
                                                         const iDynTree::Vector3& position,
                                                         const iDynTree::Rotation& orientation,
                                                         const iDynTree::Vector3& linearVelocity,
                                                         const iDynTree::Vector3& angularVelocity,
                                                         const double positionTargetWeight = 1.0,
                                                         const double orientationTargetWeight = 1.0,
                                                         const double linearVelocityWeight = 1.0,
                                                         const double angularVelocityWeight = 1.0);
    static InverseKinematicsTarget PoseTarget(const std::string& frameName,
                                              const iDynTree::Vector3& position,
                                              const iDynTree::Rotation& orientation,
                                              const double positionTargetWeight = 1.0,
                                              const double orientationTargetWeight = 1.0);
    
    

    TargetType getTargetType() const;
    std::string getFrameName() const;

    bool hasPositionTarget() const;
    bool hasOrientationTarget() const;

    iDynTree::Vector3 getPosition() const;
    void setPosition(const iDynTree::Vector3& newPosition);
    iDynTree::Vector3 getLinearVelocity() const;
    void setLinearVelocity(const iDynTree::Vector3& newLinearVelocity);
    iDynTree::Rotation getOrientation() const;
    void setOrientation(const iDynTree::Rotation& newOrientation);
    iDynTree::Vector3 getAngularVelocity() const;
    void setAngularVelocity(const iDynTree::Vector3& newAngularVelocity);
    iDynTree::Transform getTransform() const;
    void setTransform(const iDynTree::Transform& newTransform);
    iDynTree::Twist getTwist() const;
    void setTwist(const iDynTree::Twist& newTwist);

    double getPositionTargetWeight() const;
    void setPositionTargetWeight(const double newPositionTargetWeight);
    double getOrientationTargetWeight() const;
    void setOrientationTargetWeight(const double newOrientationTargetWeight);
    double getLinearVelocityWeight() const;
    void setLinearVelocityWeight(const double newLinearVelocityWeight);
    double getAngularVelocityWeight() const;
    void setAngularVelocityWeight(const double newAngularVelocityWeight);
};


DynamicalInverseKinematics::impl::InverseKinematicsTarget::InverseKinematicsTarget(
    const std::string& frameName,
    const TargetType& type)
    : type(type)
    , frameName(frameName)
    , positionTargetWeight(1.0)
    , orientationTargetWeight(1.0)
    , linearVelocityWeight(1.0)
    , angularVelocityWeight(1.0)
{
    setTransform(iDynTree::Transform::Identity());
    iDynTree::Twist twist;
    twist.zero();
    setTwist(twist);
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PositionAndVelocityTarget(
    const std::string& frameName,
    const iDynTree::Vector3& position,
    const iDynTree::Vector3& linearVelocity,
    const double positionTargetWeight,
    const double linearVelocityWeight)
{
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePosition);
    inverseKinematicsTarget.setPosition(position);
    inverseKinematicsTarget.setPositionTargetWeight(positionTargetWeight);
    inverseKinematicsTarget.setLinearVelocity(linearVelocity);
    inverseKinematicsTarget.setLinearVelocityWeight(linearVelocityWeight);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PositionTarget(
    const std::string& frameName,
    const iDynTree::Vector3& position,
    const double positionTargetWeight)
{
    iDynTree::Vector3 linearVelocity;
    linearVelocity.zero();
    return PositionAndVelocityTarget(frameName, position, linearVelocity, positionTargetWeight, 1.0);
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::OrientationAndVelocityTarget(
    const std::string& frameName,
    const iDynTree::Rotation& rotation,
    const iDynTree::Vector3& angularVelocity,
    const double orientationTargetWeight,
    const double angularVelocityWeight)
{
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePosition);
    inverseKinematicsTarget.setOrientation(rotation);
    inverseKinematicsTarget.setOrientationTargetWeight(orientationTargetWeight);
    inverseKinematicsTarget.setAngularVelocity(angularVelocity);
    inverseKinematicsTarget.setAngularVelocityWeight(angularVelocityWeight);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::OrientationTarget(
    const std::string& frameName,
    const iDynTree::Rotation& rotation,
    const double orientationTargetWeight)
{
    iDynTree::Vector3 angularVelocity;
    angularVelocity.zero();
    return OrientationAndVelocityTarget(frameName, rotation, angularVelocity, orientationTargetWeight, 1.0);
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseAndVelocityTarget(
    const std::string& frameName,
    const iDynTree::Transform& transform,
    const iDynTree::Twist& twist,
    const double positionTargetWeight,
    double orientationTargetWeight,
    double linearVelocityWeight,
    double angularVelocityWeight)
{
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePosition);
    inverseKinematicsTarget.setTransform(transform);
    inverseKinematicsTarget.setPositionTargetWeight(positionTargetWeight);
    inverseKinematicsTarget.setOrientationTargetWeight(orientationTargetWeight);
    inverseKinematicsTarget.setTwist(twist);
    inverseKinematicsTarget.setLinearVelocityWeight(linearVelocityWeight);
    inverseKinematicsTarget.setAngularVelocityWeight(angularVelocityWeight);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseTarget(
    const std::string& frameName,
    const iDynTree::Transform& transform,
    const double positionTargetWeight,
    const double orientationTargetWeight)
{
    iDynTree::Twist twist;
    twist.zero();
    return PoseAndVelocityTarget(frameName, transform, twist, positionTargetWeight, orientationTargetWeight, 1.0, 1.0);
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseAndVelocityTarget(
    const std::string& frameName,
    const iDynTree::Vector3& position,
    const iDynTree::Rotation& orientation,
    const iDynTree::Vector3& linearVelocity,
    const iDynTree::Vector3& angularVelocity,
    const double positionTargetWeight,
    double orientationTargetWeight,
    double linearVelocityWeight,
    double angularVelocityWeight)
{
    iDynTree::Twist twist(linearVelocity, angularVelocity);
    iDynTree::Transform transform(orientation, iDynTree::Position(position));

    return PoseAndVelocityTarget(frameName, transform, twist, positionTargetWeight, orientationTargetWeight, linearVelocityWeight, angularVelocityWeight);
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseTarget(
    const std::string& frameName,
    const iDynTree::Vector3& position,
    const iDynTree::Rotation& orientation,
    const double positionTargetWeight,
    double orientationTargetWeight)
{
    iDynTree::Transform transform(orientation, iDynTree::Position(position));

    return PoseTarget(frameName, transform, positionTargetWeight, orientationTargetWeight);
}


DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType
DynamicalInverseKinematics::impl::InverseKinematicsTarget::getTargetType() const
{
    return type;
}

std::string DynamicalInverseKinematics::impl::InverseKinematicsTarget::getFrameName() const
{
    return frameName;
}

bool DynamicalInverseKinematics::impl::InverseKinematicsTarget::hasPositionTarget() const
{
    return (type == TargetTypePosition) || (type == TargetTypePose);
}

bool DynamicalInverseKinematics::impl::InverseKinematicsTarget::hasOrientationTarget() const
{
    return (type == TargetTypeOrientation) || (type == TargetTypePose);
}


iDynTree::Vector3 DynamicalInverseKinematics::impl::InverseKinematicsTarget::getPosition() const
{
    return transform.getPosition();
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setPosition(
    const iDynTree::Vector3& newPosition)
{
    transform.setPosition(iDynTree::Position(newPosition));
}

iDynTree::Vector3 DynamicalInverseKinematics::impl::InverseKinematicsTarget::getLinearVelocity() const
{
    return twist.getLinearVec3();
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setLinearVelocity(
    const iDynTree::Vector3& newLinearVelocity)
{
    twist.setLinearVec3(newLinearVelocity);
}

iDynTree::Rotation DynamicalInverseKinematics::impl::InverseKinematicsTarget::getOrientation() const
{
    return transform.getRotation();
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setOrientation(
    const iDynTree::Rotation& newOrientation)
{
    transform.setRotation(newOrientation);
}

iDynTree::Vector3 DynamicalInverseKinematics::impl::InverseKinematicsTarget::getAngularVelocity() const
{
    return twist.getAngularVec3();
}

iDynTree::Transform DynamicalInverseKinematics::impl::InverseKinematicsTarget::getTransform() const
{
    return transform;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setTransform(
    const iDynTree::Transform& newTransform)
{
    transform = newTransform;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setAngularVelocity(
    const iDynTree::Vector3& newAngularVelocity)
{
    twist.setAngularVec3(newAngularVelocity);
}

iDynTree::Twist DynamicalInverseKinematics::impl::InverseKinematicsTarget::getTwist() const
{
    return twist;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setTwist(const iDynTree::Twist& newTwist)
{
    twist = newTwist;
}


double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getPositionTargetWeight() const
{
    return positionTargetWeight;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setPositionTargetWeight(
    const double newPositionTargetWeight)
{
    positionTargetWeight = newPositionTargetWeight;
}

double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getOrientationTargetWeight() const
{
    return orientationTargetWeight;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setOrientationTargetWeight(
    const double newOrientationTargetWeight)
{
    orientationTargetWeight = newOrientationTargetWeight;
}

double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getLinearVelocityWeight() const
{
    return linearVelocityWeight;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setLinearVelocityWeight(
    const double newLinearVelocityWeight)
{
    linearVelocityWeight = newLinearVelocityWeight;
}

double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getAngularVelocityWeight() const
{
    return angularVelocityWeight;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setAngularVelocityWeight(
    const double newAngularVelocityWeight)
{
    angularVelocityWeight = newAngularVelocityWeight;
}

// ====
// IMPL
// ====

DynamicalInverseKinematics::impl::impl()
: dofs(0)
, isInverseVelocityKinematicsInitialized(false)
, isInverseKinematicsInitializd(false)
{
    // These variables are touched only once.
    worldGravity.zero();
}

bool DynamicalInverseKinematics::impl::initializeInverseVelocityKinematics()
{
    inverseVelocityKinematics.setModel(dynamics.getRobotModel());
    inverseVelocityKinematics.setFloatingBaseOnFrameNamed(dynamics.getFloatingBase());

    isInverseVelocityKinematicsInitialized = true;

    return true;
}

DynamicalInverseKinematics::impl::TargetsMap::iterator
DynamicalInverseKinematics::impl::getTargetRefIfItExists(const std::string& targetFrameName)
{
    int frameIndex = dynamics.getFrameIndex(targetFrameName);
    if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
        return targets.end();

    // Find the target (if this fails, it will return m_targets.end())
    return targets.find(frameIndex);
}

bool DynamicalInverseKinematics::impl::updateConfiguration()
{
    iDynTree::Twist baseVelocity;

    inverseVelocityKinematics.getVelocitySolution(state.dot_W_p_B, state.omega_B, state.dot_s);

    return dynamics.setRobotState(iDynTree::Transform(state.W_R_B, iDynTree::Position(state.W_p_B)),
                                  state.s,
                                  iDynTree::Twist(state.dot_W_p_B, state.omega_B),
                                  state.dot_s,
                                  worldGravity);
}

bool DynamicalInverseKinematics::impl::addTarget(const InverseKinematicsTarget& target)
{
    int frameIndex = dynamics.getFrameIndex(target.getFrameName());
    if (frameIndex < 0)
        return false;

    std::pair<TargetsMap::iterator, bool> result =
        targets.insert(TargetsMap::value_type(frameIndex, target));

    isInverseKinematicsInitializd = false;
    return result.second;
}

void DynamicalInverseKinematics::impl::updateTargetPosition(
    const TargetsMap::iterator& target,
    const iDynTree::Vector3& newPosition,
    const double newPositionTargetWeight)
{
    target->second.setPosition(newPosition);
    target->second.setPositionTargetWeight(newPositionTargetWeight);
}

void DynamicalInverseKinematics::impl::updateTargetOrientation(
    const TargetsMap::iterator& target,
    const iDynTree::Rotation& newOrientation,
    const double newOrientationTargetWeight)
{
    target->second.setOrientation(newOrientation);
    target->second.setOrientationTargetWeight(newOrientationTargetWeight);
}

void DynamicalInverseKinematics::impl::updateTargetLinearVelocity(
    const TargetsMap::iterator& target,
    const iDynTree::Vector3& newLinearVelocity,
    const double newLinearVelocityWeight)
{
    target->second.setLinearVelocity(newLinearVelocity);
    target->second.setLinearVelocityWeight(newLinearVelocityWeight);
}

void DynamicalInverseKinematics::impl::updateTargetAngularVelocity(
    const TargetsMap::iterator& target,
    const iDynTree::Vector3& newAngularVelocity,
    const double newAngularVelocityWeight)
{
    target->second.setAngularVelocity(newAngularVelocity);
    target->second.setAngularVelocityWeight(newAngularVelocityWeight);
}

// ============================
// DYNAMICAL INVERSE KINEMATICS
// ============================

DynamicalInverseKinematics::DynamicalInverseKinematics()
: pImpl{new impl()}
{}

DynamicalInverseKinematics::~DynamicalInverseKinematics() {}


bool DynamicalInverseKinematics::setModel(const iDynTree::Model& model)
{
    pImpl->dofs = model.getNrOfDOFs();
    pImpl->model = model;

    bool result = pImpl->dynamics.loadRobotModel(model);
    if (!result || !pImpl->dynamics.isValid()) {
        std::cerr << "[ERROR] Error loading robot model" << std::endl;
        return false;
    }

    clearProblem();

    return true;
}

bool DynamicalInverseKinematics::setFloatingBaseOnFrameNamed(
    const std::string& floatingBaseFrameName)
{
    return pImpl->dynamics.setFloatingBase(floatingBaseFrameName);
}

bool DynamicalInverseKinematics::setInverseVelocityKinematicsResolutionMode(const std::string& resolutionModeName)
{
    return pImpl->inverseVelocityKinematics.setResolutionMode(resolutionModeName);
}

void DynamicalInverseKinematics::setInverseVelocityKinematicsRegularization(const double regularizationWeight)
{
    pImpl->inverseVelocityKinematics.setRegularization(regularizationWeight);
}


bool DynamicalInverseKinematics::addPoseAndVelocityTarget(const std::string& linkName,
                            const iDynTree::Transform& transform,
                            const iDynTree::Twist& twist,
                            const double positionTargetWeight,
                            const double rotationTargetWeight,
                            const double linearVelocityWeight,
                            const double angularVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseAndVelocityTarget(
        linkName, transform, twist, positionTargetWeight, rotationTargetWeight, linearVelocityWeight, angularVelocityWeight));
}

bool DynamicalInverseKinematics::addPoseTarget(const std::string& linkName,
                const iDynTree::Transform& transform,
                const double positionTargetWeight,
                const double rotationTargetWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseTarget(
        linkName, transform, positionTargetWeight, rotationTargetWeight));
}

bool DynamicalInverseKinematics::addPoseAndVelocityTarget(const std::string& linkName,
                            const iDynTree::Vector3& position,
                            const iDynTree::Rotation& rotation,
                            const iDynTree::Vector3& linearVelocity,
                            const iDynTree::Vector3& angularVelocity,
                            const double positionTargetWeight,
                            const double rotationTargetWeight,
                            const double linearVelocityWeight,
                            const double angularVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseAndVelocityTarget(
        linkName, position, rotation, linearVelocity, angularVelocity, positionTargetWeight, rotationTargetWeight, linearVelocityWeight, angularVelocityWeight));
}

bool DynamicalInverseKinematics::addPoseTarget(const std::string& linkName,
                const iDynTree::Vector3& position,
                const iDynTree::Rotation& rotation,
                const double positionTargetWeight,
                const double rotationTargetWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseTarget(
        linkName, position, rotation, positionTargetWeight, rotationTargetWeight));
}

bool DynamicalInverseKinematics::addPositionAndVelocityTarget(const std::string& linkName,
                                const iDynTree::Vector3& position,
                                const iDynTree::Vector3& linearVelocity,
                                const double positionTargetWeight,
                                const double linearVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PositionAndVelocityTarget(
        linkName, position, linearVelocity, positionTargetWeight, linearVelocityWeight));
}

bool DynamicalInverseKinematics::addPositionTarget(const std::string& linkName,
                    const iDynTree::Vector3& position,
                    const double positionTargetWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PositionTarget(
        linkName, position, positionTargetWeight));
}

bool DynamicalInverseKinematics::addOrientationAndVelocityTarget(const std::string& linkName,
                                    const iDynTree::Rotation& orientation,
                                    const iDynTree::Vector3& angularVelocity,
                                    const double orientationTargetWeight,
                                    const double angularVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::OrientationAndVelocityTarget(
        linkName, orientation, angularVelocity, orientationTargetWeight, angularVelocityWeight));
}

bool DynamicalInverseKinematics::addOrientationTarget(const std::string& linkName,
                        const iDynTree::Rotation& orientation,
                        const double orientationTargetWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::OrientationTarget(
        linkName, orientation, orientationTargetWeight));
}


bool DynamicalInverseKinematics::setJointConfiguration(const std::string& jointName,
                                                       const double jointConfiguration)
{
    iDynTree::JointIndex jointIndex = pImpl->dynamics.model().getJointIndex(jointName);
    if (jointIndex == iDynTree::JOINT_INVALID_INDEX)
        return false;
    pImpl->state.s(jointIndex) = jointConfiguration;
    pImpl->updateConfiguration();
    return true;
}

bool DynamicalInverseKinematics::setJointsConfiguration(const iDynTree::VectorDynSize& jointsConfiguration)
{
    if (pImpl->state.s.size() == jointsConfiguration.size()) {
        pImpl->state.s = jointsConfiguration;
        pImpl->updateConfiguration();
        return true;
    }
    else {
        return false;
    }
}

bool DynamicalInverseKinematics::setBasePose(const iDynTree::Transform& baseTransform)
{
    pImpl->state.W_p_B = baseTransform.getPosition();
    pImpl->state.W_R_B = baseTransform.getRotation();
    pImpl->updateConfiguration();
    return true;
}

bool DynamicalInverseKinematics::setBasePose(const iDynTree::Vector3& basePosition,
                                            const iDynTree::Rotation& baseRotation)
{
    pImpl->state.W_p_B = basePosition;
    pImpl->state.W_R_B = baseRotation;
    pImpl->updateConfiguration();
    return true;
}

bool DynamicalInverseKinematics::setConfiguration(const iDynTree::Transform& baseTransform,
                                                 const iDynTree::VectorDynSize& jointsConfiguration)
{
    if (setJointsConfiguration(jointsConfiguration) && setBasePose(baseTransform)) {
        pImpl->updateConfiguration();
        return true;
    }
    else {
        return false;
    }
}

bool DynamicalInverseKinematics::setConfiguration(const iDynTree::Vector3& basePosition,
                                                 const iDynTree::Rotation& baseRotation,
                                                 const iDynTree::VectorDynSize& jointsConfiguration)
{
    if (setJointsConfiguration(jointsConfiguration) && setBasePose(basePosition, baseRotation)) {
        pImpl->updateConfiguration();
        return true;
    }
    else {
        return false;
    }
}

bool updateTarget(const std::string& linkName,
                const iDynTree::Vector3& newPosition,
                const iDynTree::Rotation& newOrientation,
                const iDynTree::Vector3& newLinearVelocity,
                const iDynTree::Vector3& newAngularVelocity,
                const double newPositionTargetWeight = 1.0,
                const double newOrientationTargetWeight = 1.0,
                const double newLinearVelocityWeight = 1.0,
                const double newAngularVelocityWeight = 1.0);
bool updateTarget(const std::string& linkName,
                const iDynTree::Transform& newTransform,
                const iDynTree::Twist& newTwist,
                const double newPositionTargetWeight = 1.0,
                const double newOrientationTargetWeight = 1.0,
                const double newLinearVelocityWeight = 1.0,
                const double newAngularVelocityWeight = 1.0);
void updateTargetPosition(const std::string& linkName,
                        const iDynTree::Vector3& newPosition,
                        const double newPositionTargetWeight = 1.0);
void updateTargetOrientation(const std::string& linkName,
                            const iDynTree::Rotation& newOrientation,
                            const double newOrientationTargetWeight = 1.0);
void updateTargetLinearVelocity(const std::string& linkName,
                            const iDynTree::Vector3& newLinearVelocity,
                            const double newLinearVelocityWeight = 1.0);
void updateTargetAngularVelocity(const std::string& linkName,
                                const iDynTree::Vector3& newAngularVelocity,
                                const double newAngularVelocityWeight = 1.0);

bool DynamicalInverseKinematics::updateTarget(const std::string& linkName,
                                             const iDynTree::Vector3& newPosition,
                                             const iDynTree::Rotation& newOrientation,
                                             const iDynTree::Vector3& newLinearVelocity,
                                             const iDynTree::Vector3& newAngularVelocity,
                                             const double newPositionTargetWeight,
                                             const double newOrientationTargetWeight,
                                             const double newLinearVelocityWeight,
                                             const double newAngularVelocityWeight)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);

    if (target == pImpl->targets.end()) {
        std::stringstream ss;
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }
    
    pImpl->updateTargetPosition(target, newPosition, newPositionTargetWeight);
    pImpl->updateTargetOrientation(target, newOrientation, newOrientationTargetWeight);
    pImpl->updateTargetLinearVelocity(target, newLinearVelocity, newLinearVelocityWeight);
    pImpl->updateTargetAngularVelocity(target, newAngularVelocity, newAngularVelocityWeight);
    return true;
}

bool DynamicalInverseKinematics::updateTarget(const std::string& linkName,
                                              const iDynTree::Transform& newTransform,
                                              const iDynTree::Twist& newTwist,
                                              const double newPositionTargetWeight,
                                              const double newOrientationTargetWeight,
                                              const double newLinearVelocityWeight,
                                              const double newAngularVelocityWeight)
{
    return updateTarget(
        linkName, newTransform.getPosition(), newTransform.getRotation(), newTwist.getLinearVec3(), newTwist.getAngularVec3(), newPositionTargetWeight, newOrientationTargetWeight, newLinearVelocityWeight, newAngularVelocityWeight);
}

bool DynamicalInverseKinematics::updateTargetPosition(const std::string& linkName,
                                                      const iDynTree::Vector3& newPosition,
                                                      const double newPositionTargetWeight)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);

    if (target == pImpl->targets.end()) {
        std::stringstream ss;
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateTargetPosition(target, newPosition, newPositionTargetWeight);
    return true;
}

bool DynamicalInverseKinematics::updateTargetOrientation(const std::string& linkName,
                                                         const iDynTree::Rotation& newOrientation,
                                                         const double newOrientationTargetWeight)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);

    if (target == pImpl->targets.end()) {
        std::stringstream ss;
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateTargetOrientation(target, newOrientation, newOrientationTargetWeight);
    return true;
}

bool DynamicalInverseKinematics::updateTargetLinearVelocity(const std::string& linkName,
                                                            const iDynTree::Vector3& newLinearVelocity,
                                                            const double newLinearVelocityWeight)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);

    if (target == pImpl->targets.end()) {
        std::stringstream ss;
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateTargetLinearVelocity(target, newLinearVelocity, newLinearVelocityWeight);
    return true;
}

bool DynamicalInverseKinematics::updateTargetAngularVelocity(
    const std::string& linkName,
    const iDynTree::Vector3& newAngularVelocity,
    const double newAngularVelocityWeight)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);

    if (target == pImpl->targets.end()) {
        std::stringstream ss;
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateTargetAngularVelocity(target, newAngularVelocity, newAngularVelocityWeight);
    return true;
}


bool DynamicalInverseKinematics::getJointsConfigurationSolution(iDynTree::VectorDynSize& jointsConfiguration) const
{
    if (jointsConfiguration.size() == pImpl->state.s.size()) {
        jointsConfiguration = pImpl->state.s;
        return true;
    }
    else {
        return false;
    }
}

bool DynamicalInverseKinematics::getBasePoseSolution(iDynTree::Transform& baseTransform) const
{
    baseTransform = iDynTree::Transform(pImpl->state.W_R_B, iDynTree::Position(pImpl->state.W_p_B));
    return true;
}

bool DynamicalInverseKinematics::getBasePoseSolution(iDynTree::Vector3& basePosition, iDynTree::Rotation& baseRotation) const
{
    basePosition = pImpl->state.W_p_B;
    baseRotation = pImpl->state.W_R_B;
    return true;
}

bool DynamicalInverseKinematics::getConfigurationSolution(iDynTree::Transform& baseTransform,
                                                          iDynTree::VectorDynSize& jointsConfiguration) const
{
    return getJointsConfigurationSolution(jointsConfiguration) && getBasePoseSolution(baseTransform);
}
bool DynamicalInverseKinematics::getConfigurationSolution(iDynTree::Vector3& basePosition,
                                                          iDynTree::Rotation& baseRotation,
                                                          iDynTree::VectorDynSize& jointsConfiguration) const
{
    return getJointsConfigurationSolution(jointsConfiguration) && getBasePoseSolution(basePosition, baseRotation);
}

bool DynamicalInverseKinematics::getVelocitySolution(iDynTree::Twist& baseVelocity,
                                                     iDynTree::VectorDynSize& jointsVelocity) const
{
    return pImpl->inverseVelocityKinematics.getVelocitySolution(baseVelocity, jointsVelocity);
}

bool DynamicalInverseKinematics::getJointsVelocitySolution(
    iDynTree::VectorDynSize& jointsVelocity) const
{
    return pImpl->inverseVelocityKinematics.getJointsVelocitySolution(jointsVelocity);
}

bool DynamicalInverseKinematics::getBaseVelocitySolution(iDynTree::Twist& baseVelocity) const
{
    return pImpl->inverseVelocityKinematics.getBaseVelocitySolution(baseVelocity);
}

bool DynamicalInverseKinematics::getBaseVelocitySolution(iDynTree::Vector3& linearVelocity,
                                                         iDynTree::Vector3& angularVelocity) const
{
    return pImpl->inverseVelocityKinematics.getBaseVelocitySolution(linearVelocity, angularVelocity);
}

void DynamicalInverseKinematics::clearProblem()
{

    pImpl->state.initializeState(pImpl->dofs);
    
    pImpl->inverseVelocityKinematics.clearProblem();

    pImpl->isInverseKinematicsInitializd = false;
}
