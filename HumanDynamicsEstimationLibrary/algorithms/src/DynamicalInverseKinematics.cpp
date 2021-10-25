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
    TargetsMap m_targets;

    InverseVelocityKinematics m_inverseVelocityKinematics;
    hde::utils::idyntree::state::Integrator m_stateIntegrator;

    iDynTree::Model m_model;
    iDynTree::KinDynComputations m_dynamics;
    size_t m_dofs;
     
    hde::utils::idyntree::state::State m_state;

    struct Limits
    {
        iDynTree::VectorDynSize jointPositionUpperLimit;
        iDynTree::VectorDynSize jointPositionLowerLimit;
        iDynTree::VectorDynSize jointVelocityUpperLimit;
        iDynTree::VectorDynSize jointVelocityLowerLimit;
        iDynTree::VectorDynSize baseVelocityUpperLimit;
        iDynTree::VectorDynSize baseVelocityLowerLimit;

        void initializeLimits(const double ndof)
        {
            jointPositionUpperLimit.resize(ndof);
            jointPositionUpperLimit.zero();
            jointPositionLowerLimit.resize(ndof);
            jointPositionLowerLimit.zero();
            jointVelocityUpperLimit.resize(ndof);
            jointVelocityUpperLimit.zero();
            jointVelocityLowerLimit.resize(ndof);
            jointVelocityLowerLimit.zero();
            baseVelocityUpperLimit.resize(6);
            baseVelocityUpperLimit.zero();
            baseVelocityLowerLimit.resize(6);
            baseVelocityLowerLimit.zero();
        };
    };
    Limits m_limits;

    struct LinearJointConfigurationLimits
    {
        iDynTree::MatrixDynSize constraintMatrix; 
        std::vector<iDynTree::JointIndex> constraintVariablesIndex; 
        iDynTree::VectorDynSize constraintUpperBound; 
        iDynTree::VectorDynSize constraintLowerBound; 
        double k_u, k_l;
    };
    LinearJointConfigurationLimits m_linearLimits;
    
    iDynTree::Vector3 m_worldGravity;

    bool m_isInverseKinematicsInitializd;
    bool m_isModelLoaded;

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

    bool solveProblem(const double dt);

    bool computeDesiredLinkVelocities();

    bool getJointPositionLimitsFromModel();

    bool initialize();
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

    iDynTree::Transform targetTransform;
    iDynTree::Twist targetTwist;
    TargetType type;
    std::string frameName;

    double positionTargetWeight;
    double orientationTargetWeight;
    double linearVelocityWeight;
    double angularVelocityWeight;

    int errorSize;

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
    
    
    void setTargetType(const TargetType targetType);
    TargetType getTargetType() const;
    std::string getFrameName() const;

    bool hasPositionTarget() const;
    bool hasOrientationTarget() const;

    iDynTree::VectorDynSize errorBuffer;
    int getErrorSize() const;
    bool computeError(const iDynTree::Transform transform, iDynTree::VectorDynSize& error);
    bool computeError(const iDynTree::Vector3 position, iDynTree::VectorDynSize& error);
    bool computeError(const iDynTree::Rotation orientation, iDynTree::VectorDynSize& error);

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
    : frameName(frameName)
    , positionTargetWeight(1.0)
    , orientationTargetWeight(1.0)
    , linearVelocityWeight(1.0)
    , angularVelocityWeight(1.0)
{
    setTargetType(type);
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
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypeOrientation);
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
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePose);
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

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setTargetType(const DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType targetType)
{
    type = targetType;
    switch (type)
    {
    case DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType::TargetTypePosition:
        errorSize = 3;
        break;
    case DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType::TargetTypeOrientation:
        errorSize = 3;
        break;
    case DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType::TargetTypePose:
        errorSize = 6;
        break;
    }
}

int DynamicalInverseKinematics::impl::InverseKinematicsTarget::getErrorSize() const
{
    return errorSize;
}

bool DynamicalInverseKinematics::impl::InverseKinematicsTarget::computeError(const iDynTree::Transform transform, iDynTree::VectorDynSize& error)
{
    if ( (type != TargetTypePose) || (error.size() != errorSize))
    {
        return false;
    }

    iDynTree::toEigen(error).head(3) = iDynTree::toEigen(transform.getPosition()) - iDynTree::toEigen(targetTransform.getPosition());
    iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(hde::utils::idyntree::rotation::skewVee(transform.getRotation() * targetTransform.getRotation().inverse()));

    return true;
}

bool DynamicalInverseKinematics::impl::InverseKinematicsTarget::computeError(const iDynTree::Vector3 position, iDynTree::VectorDynSize& error)
{
    if ( (type != TargetTypePosition) || (error.size() != errorSize))
    {
        return false;
    }

    iDynTree::toEigen(error) = iDynTree::toEigen(position) - iDynTree::toEigen(targetTransform.getPosition());
   
    return true;
}


bool DynamicalInverseKinematics::impl::InverseKinematicsTarget::computeError(const iDynTree::Rotation orientation, iDynTree::VectorDynSize& error)
{
    if ( (type != TargetTypeOrientation) || (error.size() != errorSize))
    {
        return false;
    }

    iDynTree::toEigen(error) = iDynTree::toEigen(hde::utils::idyntree::rotation::skewVee(orientation * targetTransform.getRotation().inverse()));

    return true;
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
    return targetTransform.getPosition();
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setPosition(
    const iDynTree::Vector3& newPosition)
{
    targetTransform.setPosition(iDynTree::Position(newPosition));
}

iDynTree::Vector3 DynamicalInverseKinematics::impl::InverseKinematicsTarget::getLinearVelocity() const
{
    return targetTwist.getLinearVec3();
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setLinearVelocity(
    const iDynTree::Vector3& newLinearVelocity)
{
    targetTwist.setLinearVec3(newLinearVelocity);
}

iDynTree::Rotation DynamicalInverseKinematics::impl::InverseKinematicsTarget::getOrientation() const
{
    return targetTransform.getRotation();
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setOrientation(
    const iDynTree::Rotation& newOrientation)
{
    targetTransform.setRotation(newOrientation);
}

iDynTree::Vector3 DynamicalInverseKinematics::impl::InverseKinematicsTarget::getAngularVelocity() const
{
    return targetTwist.getAngularVec3();
}

iDynTree::Transform DynamicalInverseKinematics::impl::InverseKinematicsTarget::getTransform() const
{
    return targetTransform;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setTransform(
    const iDynTree::Transform& newTransform)
{
    targetTransform = newTransform;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setAngularVelocity(
    const iDynTree::Vector3& newAngularVelocity)
{
    targetTwist.setAngularVec3(newAngularVelocity);
}

iDynTree::Twist DynamicalInverseKinematics::impl::InverseKinematicsTarget::getTwist() const
{
    return targetTwist;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setTwist(const iDynTree::Twist& newTwist)
{
    targetTwist = newTwist;
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
: m_dofs(0)
, m_isInverseKinematicsInitializd(false)
, m_isModelLoaded(false)
{
    // These variables are touched only once.
    m_worldGravity.zero();
}

bool DynamicalInverseKinematics::impl::initialize()
{
    // Initialize integrator
    m_stateIntegrator.setInterpolatorType(hde::utils::idyntree::state::Integrator::InterpolationType::trapezoidal);
    m_stateIntegrator.setNJoints(m_model.getNrOfDOFs());
    m_stateIntegrator.setJointLimits(m_limits.jointPositionLowerLimit, m_limits.jointPositionUpperLimit);
    

    // Initialize Inverse Velocity Kinematics
    m_inverseVelocityKinematics.setModel(m_dynamics.getRobotModel());
    m_inverseVelocityKinematics.setFloatingBaseOnFrameNamed(m_dynamics.getFloatingBase());

    // Add the targets for the inverse velocity kinematics solver
    for (auto const& it : m_targets)
    {
        auto target = it.second;
        switch (target.getTargetType()) {
            case InverseKinematicsTarget::TargetType::TargetTypePosition:
                if (!m_inverseVelocityKinematics.addLinearVelocityTarget(target.getFrameName(), target.getLinearVelocity(), 1.0))
                    return false;
                break;
            case InverseKinematicsTarget::TargetType::TargetTypeOrientation:
                if (!m_inverseVelocityKinematics.addAngularVelocityTarget(target.getFrameName(), target.getAngularVelocity(), 1.0))
                    return false;
                break;
            case InverseKinematicsTarget::TargetType::TargetTypePose:
                if (!m_inverseVelocityKinematics.addTarget(target.getFrameName(), target.getLinearVelocity(), target.getAngularVelocity(), 1.0, 1.0))
                    return false;
                break;
        }
    }
    if (!m_inverseVelocityKinematics.setGeneralJointVelocityConstraints(100.0)) // to be removed
        return false; 
    if (!m_inverseVelocityKinematics.setGeneralJointsUpperLowerConstraints(m_limits.jointPositionUpperLimit,
                                                                           m_limits.jointPositionLowerLimit))
        return false;
    
    if (m_limits.jointVelocityUpperLimit.size() > 0)
    {
        std::vector<iDynTree::JointIndex> jointsList;
        jointsList.clear();
        for (size_t i = 0; i < m_dofs; i++)
            jointsList.push_back(i);
        if (!m_inverseVelocityKinematics.setCustomJointsVelocityLimit(jointsList, m_limits.jointVelocityUpperLimit))
            return false;
    }
    if (m_linearLimits.constraintVariablesIndex.size() > 0)
    {
        if (!m_inverseVelocityKinematics.setCustomConstraintsJointsValues(m_linearLimits.constraintVariablesIndex,
                                                                      m_linearLimits.constraintUpperBound,
                                                                      m_linearLimits.constraintLowerBound,
                                                                      m_linearLimits.constraintMatrix,
                                                                      m_linearLimits.k_u,
                                                                      m_linearLimits.k_l))
            return false;
    }

    m_isInverseKinematicsInitializd = true;

    return true;
}

DynamicalInverseKinematics::impl::TargetsMap::iterator
DynamicalInverseKinematics::impl::getTargetRefIfItExists(const std::string& targetFrameName)
{
    int frameIndex = m_dynamics.getFrameIndex(targetFrameName);
    if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
        return m_targets.end();

    // Find the target (if this fails, it will return m_targets.end())
    return m_targets.find(frameIndex);
}

bool DynamicalInverseKinematics::impl::addTarget(const InverseKinematicsTarget& target)
{
    int frameIndex = m_dynamics.getFrameIndex(target.getFrameName());
    if (frameIndex < 0)
        return false;

    std::pair<TargetsMap::iterator, bool> result =
        m_targets.insert(TargetsMap::value_type(frameIndex, target));

    m_isInverseKinematicsInitializd = false;
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

bool DynamicalInverseKinematics::impl::solveProblem(const double dt)
{
    if (!m_isInverseKinematicsInitializd) {
        if (!initialize())
            return false;
    }

    // update internal configuration inverse velocity kinematics state
    if (!updateConfiguration())
        return false;

    // compute desired link velocities and update inverse veloicty kinematics targets
    if (!computeDesiredLinkVelocities())
        return false;

    // solve inverse velocity kinematics
    if (!m_inverseVelocityKinematics.solve())
        return false;
    if (!(m_inverseVelocityKinematics.getBaseVelocitySolution(m_state.dot_W_p_B, m_state.omega_B) && m_inverseVelocityKinematics.getJointsVelocitySolution(m_state.dot_s)))
        return false;

    m_stateIntegrator.integrate(m_state.dot_s,
                              m_state.dot_W_p_B,
                              m_state.omega_B,
                              dt);
    m_stateIntegrator.getJointConfiguration(m_state.s);
    m_stateIntegrator.getBasePose(m_state.W_p_B, m_state.W_R_B);


    return true;
}

bool DynamicalInverseKinematics::impl::updateConfiguration()
{
    // update internal configuration inverse velocity kinematics state
    if (!m_inverseVelocityKinematics.setConfiguration(m_state.W_p_B, m_state.W_R_B, m_state.s))
        return false;

    if (!m_dynamics.setRobotState(iDynTree::Transform(m_state.W_R_B, iDynTree::Position(m_state.W_p_B)),
                                m_state.s,
                                iDynTree::Twist(m_state.dot_W_p_B, m_state.omega_B),
                                m_state.dot_s,
                                m_worldGravity))
        return false;

    return true;
}

bool DynamicalInverseKinematics::impl::computeDesiredLinkVelocities()
{
    for (auto const& it : m_targets)
    {
        auto target = it.second;

        // initialize buffers
        iDynTree::VectorDynSize errorBuffer;
        iDynTree::Vector3 desiredLinearVelocityBuffer, desiredAngularVelocityBuffer;

        switch (target.getTargetType()) {
            case DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType::TargetTypePosition:
                errorBuffer.resize(3);
                if (!target.computeError(m_dynamics.getWorldTransform(target.getFrameName()).getPosition(), errorBuffer))
                    return false;
                iDynTree::toEigen(desiredLinearVelocityBuffer) = target.getLinearVelocityWeight() * iDynTree::toEigen(target.getLinearVelocity()) 
                                                                 - target.getPositionTargetWeight()  * iDynTree::toEigen(errorBuffer);
                if (!m_inverseVelocityKinematics.updateTargetLinearVelocity(target.getFrameName(), desiredLinearVelocityBuffer, 1.0))
                    return false;
                break;
            case DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType::TargetTypeOrientation:
                errorBuffer.resize(3);
                if (!target.computeError(m_dynamics.getWorldTransform(target.getFrameName()).getRotation(), errorBuffer))
                    return false;
                iDynTree::toEigen(desiredAngularVelocityBuffer) = target.getAngularVelocityWeight() * iDynTree::toEigen(target.getAngularVelocity()) 
                                                                 - target.getOrientationTargetWeight()  * iDynTree::toEigen(errorBuffer);
                if (!m_inverseVelocityKinematics.updateTargetAngularVelocity(target.getFrameName(), desiredAngularVelocityBuffer, 1.0))
                    return false;
                break;
            case DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType::TargetTypePose:
                errorBuffer.resize(6);
                if (!target.computeError(m_dynamics.getWorldTransform(target.getFrameName()), errorBuffer))
                    return false;
                iDynTree::toEigen(desiredLinearVelocityBuffer) = target.getLinearVelocityWeight() * iDynTree::toEigen(target.getLinearVelocity()) 
                                                                 - target.getPositionTargetWeight()  * iDynTree::toEigen(errorBuffer).head(3);
                iDynTree::toEigen(desiredAngularVelocityBuffer) = target.getAngularVelocityWeight() * iDynTree::toEigen(target.getAngularVelocity()) 
                                                                 - target.getOrientationTargetWeight()  * iDynTree::toEigen(errorBuffer).tail(3);
                if (!m_inverseVelocityKinematics.updateTarget(target.getFrameName(), iDynTree::Twist(desiredLinearVelocityBuffer, desiredAngularVelocityBuffer), 1.0, 1.0))
                    return false;
                break;
        }
    }
    return true;
}

bool DynamicalInverseKinematics::impl::getJointPositionLimitsFromModel()
{
    m_limits.jointPositionLowerLimit.resize(m_model.getNrOfDOFs());
    m_limits.jointPositionUpperLimit.resize(m_model.getNrOfDOFs());
    size_t DOFIndex = 0;
    for (size_t jointIndex = 0; jointIndex < m_model.getNrOfJoints(); ++jointIndex) {
        if (m_model.getJoint(jointIndex)->getNrOfDOFs() == 1) {
            m_limits.jointPositionLowerLimit.setVal(DOFIndex, m_model.getJoint(jointIndex)->getMinPosLimit(0));
            m_limits.jointPositionUpperLimit.setVal(DOFIndex, m_model.getJoint(jointIndex)->getMaxPosLimit(0));
            DOFIndex++;
        }
    }

    return true;
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
    pImpl->m_dofs = model.getNrOfDOFs();
    pImpl->m_model = model;

    bool result = pImpl->m_dynamics.loadRobotModel(pImpl->m_model);
    if (!result || !pImpl->m_dynamics.isValid()) {
        pImpl->m_isModelLoaded = false;
        return false;
    }

    clearProblem();
    pImpl->m_isModelLoaded = true;

    return true;
}

bool DynamicalInverseKinematics::setFloatingBaseOnFrameNamed(
    const std::string& floatingBaseFrameName)
{
    if (!pImpl->m_isModelLoaded)
        return false;
    return pImpl->m_dynamics.setFloatingBase(floatingBaseFrameName);
}

bool DynamicalInverseKinematics::setInverseVelocityKinematicsResolutionMode(const std::string& resolutionModeName)
{
    return pImpl->m_inverseVelocityKinematics.setResolutionMode(resolutionModeName);
}

void DynamicalInverseKinematics::setInverseVelocityKinematicsRegularization(const double regularizationWeight)
{
    pImpl->m_inverseVelocityKinematics.setRegularization(regularizationWeight);
}

bool DynamicalInverseKinematics::setLinearJointConfigurationLimits(const std::vector<iDynTree::JointIndex>& jointsIndexList,
                                                                   const iDynTree::VectorDynSize& upperBoundary,
                                                                   const iDynTree::VectorDynSize& lowerBoundary,
                                                                   const iDynTree::MatrixDynSize& customConstraintMatrix,
                                                                   const double k_u,
                                                                   const double k_l)
{
    if (!pImpl->m_isModelLoaded)
        return false;

    
    pImpl->m_linearLimits.constraintVariablesIndex = jointsIndexList;
    pImpl->m_linearLimits.constraintMatrix = customConstraintMatrix;
    pImpl->m_linearLimits.constraintUpperBound = upperBoundary;
    pImpl->m_linearLimits.constraintLowerBound = lowerBoundary;
    pImpl->m_linearLimits.k_u = k_u;
    pImpl->m_linearLimits.k_l = k_l;

    return true;
}

bool DynamicalInverseKinematics::setAllJointsVelocityLimit(const double limit)
{
    if (!pImpl->m_isModelLoaded || limit < 0.0)
        return false;
    
    for (size_t i = 0; i < pImpl->m_dofs; i++)
    {
        pImpl->m_limits.jointVelocityUpperLimit.setVal(i, limit);
        pImpl->m_limits.jointVelocityLowerLimit.setVal(i, -limit);
    }
    return true;
}

bool DynamicalInverseKinematics::setBaseVelocityLimit(const iDynTree::VectorDynSize& lowerLimit,
                                                      const iDynTree::VectorDynSize& upperLimit)
{
    if (!pImpl->m_isModelLoaded)
        return false;

    pImpl->m_limits.baseVelocityUpperLimit = upperLimit;
    pImpl->m_limits.baseVelocityLowerLimit = lowerLimit;
    return true;
}

bool DynamicalInverseKinematics::setJointVelocityLimit(const iDynTree::JointIndex jointIndex, const double limit)
{
    if (!pImpl->m_isModelLoaded || limit < 0.0)
        return false;
    
    pImpl->m_limits.jointVelocityUpperLimit.setVal(jointIndex, limit);
    pImpl->m_limits.jointVelocityLowerLimit.setVal(jointIndex, -limit);

    return true;
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
    if (!pImpl->m_isModelLoaded)
        return false;
    
    iDynTree::JointIndex jointIndex = pImpl->m_dynamics.model().getJointIndex(jointName);
    if (jointIndex == iDynTree::JOINT_INVALID_INDEX)
        return false;
    pImpl->m_state.s(jointIndex) = jointConfiguration;
    pImpl->updateConfiguration();
    return true;
}

bool DynamicalInverseKinematics::setJointsConfiguration(const iDynTree::VectorDynSize& jointsConfiguration)
{
    if (!pImpl->m_isModelLoaded)
        return false;
    
    if (pImpl->m_state.s.size() == jointsConfiguration.size()) {
        pImpl->m_state.s = jointsConfiguration;
        pImpl->updateConfiguration();
        return true;
    }
    else {
        return false;
    }
}

bool DynamicalInverseKinematics::setBasePose(const iDynTree::Transform& baseTransform)
{
    if (!pImpl->m_isModelLoaded)
        return false;
    
    pImpl->m_state.W_p_B = baseTransform.getPosition();
    pImpl->m_state.W_R_B = baseTransform.getRotation();
    pImpl->updateConfiguration();
    return true;
}

bool DynamicalInverseKinematics::setBasePose(const iDynTree::Vector3& basePosition,
                                            const iDynTree::Rotation& baseRotation)
{
    if (!pImpl->m_isModelLoaded)
        return false;
    
    pImpl->m_state.W_p_B = basePosition;
    pImpl->m_state.W_R_B = baseRotation;
    pImpl->updateConfiguration();
    return true;
}

bool DynamicalInverseKinematics::setConfiguration(const iDynTree::Transform& baseTransform,
                                                 const iDynTree::VectorDynSize& jointsConfiguration)
{
    if (!pImpl->m_isModelLoaded)
        return false;
    
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
    if (!pImpl->m_isModelLoaded)
        return false;
    
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

    if (target == pImpl->m_targets.end()) {
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

    if (target == pImpl->m_targets.end()) {
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

    if (target == pImpl->m_targets.end()) {
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

    if (target == pImpl->m_targets.end()) {
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

    if (target == pImpl->m_targets.end()) {
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
    if (jointsConfiguration.size() == pImpl->m_state.s.size()) {
        jointsConfiguration = pImpl->m_state.s;
        return true;
    }
    else {
        return false;
    }
}

bool DynamicalInverseKinematics::getBasePoseSolution(iDynTree::Transform& baseTransform) const
{
    baseTransform = iDynTree::Transform(pImpl->m_state.W_R_B, iDynTree::Position(pImpl->m_state.W_p_B));
    return true;
}

bool DynamicalInverseKinematics::getBasePoseSolution(iDynTree::Vector3& basePosition, iDynTree::Rotation& baseRotation) const
{
    basePosition = pImpl->m_state.W_p_B;
    baseRotation = pImpl->m_state.W_R_B;
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
    getBaseVelocitySolution(baseVelocity);
    getJointsVelocitySolution(jointsVelocity);

    return true;
}

bool DynamicalInverseKinematics::getJointsVelocitySolution(
    iDynTree::VectorDynSize& jointsVelocity) const
{
    jointsVelocity = pImpl->m_state.dot_s;
    return true;
}

bool DynamicalInverseKinematics::getBaseVelocitySolution(iDynTree::Twist& baseVelocity) const
{
    baseVelocity = iDynTree::Twist(pImpl->m_state.dot_W_p_B, pImpl->m_state.omega_B);
    return true;
}

bool DynamicalInverseKinematics::getBaseVelocitySolution(iDynTree::Vector3& linearVelocity,
                                                         iDynTree::Vector3& angularVelocity) const
{
    linearVelocity = pImpl->m_state.dot_W_p_B;
    angularVelocity = pImpl->m_state.omega_B;
    return true;
}

bool DynamicalInverseKinematics::solve(const double dt)
{
    return pImpl->solveProblem(dt);
}

void DynamicalInverseKinematics::clearProblem()
{

    pImpl->m_state.initializeState(pImpl->m_dofs);

    // initialize joint limits
    pImpl->m_limits.initializeLimits(pImpl->m_dofs);
    pImpl->getJointPositionLimitsFromModel();

    pImpl->m_isInverseKinematicsInitializd = false;
}
