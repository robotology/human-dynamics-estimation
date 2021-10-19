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

#include <yarp/os/LogStream.h>

using namespace hde::algorithms;

// ====
// IMPL
// ====

class DynamicalInverseKinematics::impl
{

public:

    impl();

    InverseVelocityKinematics inverseVelocityKinematics;
    hde::utils::idyntree::state::Integrator stateIntegrator;

    iDynTree::Model model;
    iDynTree::KinDynComputations dynamics;
    size_t dofs;

    iDynTree::VectorDynSize jointConfigurationSolution;
    iDynTree::Transform baseTransformSolution;
    iDynTree::Vector3 worldGravity;

    bool isInverseVelocityKinematicsInitialized;

    bool initializeInverseVelocityKinematics();
    bool updateConfiguration();
};

DynamicalInverseKinematics::impl::impl()
: dofs(0)
, isInverseVelocityKinematicsInitialized(false)
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

bool DynamicalInverseKinematics::impl::updateConfiguration()
{
    iDynTree::Twist baseVelocity;
    iDynTree::VectorDynSize jointsVelocity;
    jointsVelocity.resize(jointConfigurationSolution.size());

    inverseVelocityKinematics.getVelocitySolution(baseVelocity, jointsVelocity);

    return dynamics.setRobotState(baseTransformSolution,
                                  jointConfigurationSolution,
                                  baseVelocity,
                                  jointsVelocity,
                                  worldGravity);
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



bool DynamicalInverseKinematics::setJointConfiguration(const std::string& jointName,
                                                       const double jointConfiguration)
{
    iDynTree::JointIndex jointIndex = pImpl->dynamics.model().getJointIndex(jointName);
    if (jointIndex == iDynTree::JOINT_INVALID_INDEX)
        return false;
    pImpl->jointConfigurationSolution(jointIndex) = jointConfiguration;
    pImpl->updateConfiguration();
    return true;
}

bool DynamicalInverseKinematics::setJointsConfiguration(const iDynTree::VectorDynSize& jointsConfiguration)
{
    if (pImpl->jointConfigurationSolution.size() == jointsConfiguration.size()) {
        pImpl->jointConfigurationSolution = jointsConfiguration;
        pImpl->updateConfiguration();
        return true;
    }
    else {
        return false;
    }
}

bool DynamicalInverseKinematics::setBasePose(const iDynTree::Transform& baseTransform)
{
    pImpl->baseTransformSolution = baseTransform;
    pImpl->updateConfiguration();
    return true;
}

bool DynamicalInverseKinematics::setBasePose(const iDynTree::Vector3& basePosition,
                                            const iDynTree::Rotation& baseRotation)
{
    iDynTree::Position _basePosition;
    iDynTree::toEigen(_basePosition) = iDynTree::toEigen(basePosition);
    pImpl->baseTransformSolution.setPosition(_basePosition);
    pImpl->baseTransformSolution.setRotation(baseRotation);
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


bool DynamicalInverseKinematics::getJointsConfigurationSolution(iDynTree::VectorDynSize& jointsConfiguration) const
{
    if (jointsConfiguration.size() == pImpl->jointConfigurationSolution.size()) {
        jointsConfiguration = pImpl->jointConfigurationSolution;
        return true;
    }
    else {
        return false;
    }
}

bool DynamicalInverseKinematics::getBasePoseSolution(iDynTree::Transform& baseTransform) const
{
    baseTransform = pImpl->baseTransformSolution;
    return true;
}

bool DynamicalInverseKinematics::getBasePoseSolution(iDynTree::Vector3& basePosition, iDynTree::Rotation& baseRotation) const
{
    basePosition = pImpl->baseTransformSolution.getPosition();
    baseRotation = pImpl->baseTransformSolution.getRotation();
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
    pImpl->baseTransformSolution.setPosition(iDynTree::Position(0, 0, 0));
    pImpl->baseTransformSolution.setRotation(iDynTree::Rotation::Identity());

    pImpl->jointConfigurationSolution.resize(pImpl->dofs);
    pImpl->jointConfigurationSolution.zero();

    pImpl->inverseVelocityKinematics.clearProblem();
}
