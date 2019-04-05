/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "InverseVelocityKinematics.hpp"

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

// ====
// IMPL
// ====

class InverseVelocityKinematics::impl
{
public:
    iDynTree::Model model;
    iDynTree::KinDynComputations dynamics;
    size_t dofs;

    struct {
            iDynTree::VectorDynSize jointsConfiguration;
            iDynTree::Transform basePose;
            iDynTree::VectorDynSize jointsVelocity;
            iDynTree::Twist baseTwist;
            iDynTree::Vector3 worldGravity;
        } state;

    InverseVelocityKinematicsResolutionMode resolutionMode;

    class VelocityConstraint;
    typedef std::map<int, VelocityConstraint> VelocityMap;
    VelocityMap velocityTarget;
};

// ===================
// VELOCITY CONSTRAINT
// ===================

class InverseVelocityKinematics::impl::VelocityConstraint
{
public:

    enum VelocityConstraintType {
            VelocityConstraintTypeLinearVelocity,
            VelocityConstraintTypeAngularVelocity,
            VelocityConstraintTypeTwist
        };

    iDynTree::Twist twist;
    VelocityConstraintType type;
    std::string frameName;
    double linearVelocityWeight;
    double angularVelocityWeight;

    VelocityConstraint(std::string frameName, VelocityConstraintType type);

    static VelocityConstraint linearVelocityConstraint(std::string frameName, iDynTree::Vector3 linearVelocity, double linearVelocityWeight=1.0);
    static VelocityConstraint angularVelocityConstraint(std::string frameName, iDynTree::Vector3 angularVelocity, double angularVelocityWeight=1.0);
    static VelocityConstraint TwistConstraint(std::string frameName,
                                              iDynTree::Vector3 linearVelocity,
                                              iDynTree::Vector3 angularVelocity,
                                              double linearVelocityWeight=1.0,
                                              double angularVelocityWeight=1.0);
    static VelocityConstraint TwistConstraint(std::string frameName,
                                              iDynTree::Twist twist,
                                              double linearVelocityWeight=1.0,
                                              double angularVelocityWeight=1.0);

    VelocityConstraintType getType();

    bool hasLinearVelocityConstraint();
    bool hasAngularVelocityConstraint();

    iDynTree::Vector3 getLinearVelocity();
    void setLinearVelocity(iDynTree::Vector3 newLinearVelocity);

    iDynTree::Vector3 getAngularVelocity();
    void setAngularVelocity(iDynTree::Vector3 newAngularVelocity);

    iDynTree::Twist getTwist();
    void setTwist(iDynTree::Twist newTwist);

    double getLinearVelocityWeight();
    void setLinearVelocityWeight(double newLinearVelocityWeight);

    double getAngularVelocityWeight();
    void setAngularVelocityWeight(double newAngularVelocityWeight);
};

InverseVelocityKinematics::impl::VelocityConstraint::VelocityConstraint(std::string frameName, VelocityConstraintType type):
    type(type),
    frameName(frameName),
    linearVelocityWeight(1.0),
    angularVelocityWeight(1.0)
{}

InverseVelocityKinematics::impl::VelocityConstraint InverseVelocityKinematics::impl::VelocityConstraint::linearVelocityConstraint(std::string frameName,
                                                                                                                                  iDynTree::Vector3 linearVelocity,
                                                                                                                                  double linearVelocityWeight)
{
    VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeLinearVelocity);
    velocityConstraint.setLinearVelocity(linearVelocity);
    velocityConstraint.setLinearVelocityWeight(linearVelocityWeight);
    return velocityConstraint;
}

InverseVelocityKinematics::impl::VelocityConstraint InverseVelocityKinematics::impl::VelocityConstraint::angularVelocityConstraint(std::string frameName,
                                                                                                                                   iDynTree::Vector3 angularVelocity,
                                                                                                                                   double angularVelocityWeight)
{
    VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeAngularVelocity);
    velocityConstraint.setLinearVelocity(angularVelocity);
    velocityConstraint.setLinearVelocityWeight(angularVelocityWeight);
    return velocityConstraint;
}

InverseVelocityKinematics::impl::VelocityConstraint InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(std::string frameName,
                                                                                                                         iDynTree::Vector3 linearVelocity,
                                                                                                                         iDynTree::Vector3 angularVelocity,
                                                                                                                         double linearVelocityWeight,
                                                                                                                         double angularVelocityWeight)
{
    iDynTree::Twist twist(linearVelocity, angularVelocity);
    return TwistConstraint(frameName, twist, linearVelocityWeight, angularVelocityWeight);
}

InverseVelocityKinematics::impl::VelocityConstraint InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(std::string frameName,
                                                                                                                         iDynTree::Twist twist,
                                                                                                                         double linearVelocityWeight,
                                                                                                                         double angularVelocityWeight)
{
    VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeTwist);
    velocityConstraint.setTwist(twist);
    velocityConstraint.setLinearVelocityWeight(angularVelocityWeight);
    velocityConstraint.setLinearVelocityWeight(angularVelocityWeight);
    return velocityConstraint;
}

InverseVelocityKinematics::impl::VelocityConstraint::VelocityConstraintType InverseVelocityKinematics::impl::VelocityConstraint::getType()
{
    return type;
}

bool InverseVelocityKinematics::impl::VelocityConstraint::hasLinearVelocityConstraint()
{
    return (type == VelocityConstraintTypeLinearVelocity) || (type == VelocityConstraintTypeTwist);
}

bool InverseVelocityKinematics::impl::VelocityConstraint::hasAngularVelocityConstraint()
{
    return (type == VelocityConstraintTypeAngularVelocity) || (type == VelocityConstraintTypeTwist);
}

iDynTree::Vector3 InverseVelocityKinematics::impl::VelocityConstraint::getLinearVelocity()
{
    return twist.getLinearVec3();
}

void InverseVelocityKinematics::impl::VelocityConstraint::setLinearVelocity(iDynTree::Vector3 newLinearVelocity)
{
    twist.setLinearVec3(newLinearVelocity);
}

iDynTree::Vector3 InverseVelocityKinematics::impl::VelocityConstraint::getAngularVelocity()
{
    return twist.getAngularVec3();
}

void InverseVelocityKinematics::impl::VelocityConstraint::setAngularVelocity(iDynTree::Vector3 newAngularVelocity)
{
    twist.setAngularVec3(newAngularVelocity);
}

iDynTree::Twist InverseVelocityKinematics::impl::VelocityConstraint::getTwist()
{
    return twist;
}

void InverseVelocityKinematics::impl::VelocityConstraint::setTwist(iDynTree::Twist newTwist)
{
    twist = newTwist;
}

double InverseVelocityKinematics::impl::VelocityConstraint::getLinearVelocityWeight()
{
    return linearVelocityWeight;
}

void InverseVelocityKinematics::impl::VelocityConstraint::setLinearVelocityWeight(double newLinearVelocityWeight)
{
    linearVelocityWeight = newLinearVelocityWeight;
}

double InverseVelocityKinematics::impl::VelocityConstraint::getAngularVelocityWeight()
{
    return angularVelocityWeight;
}

void InverseVelocityKinematics::impl::VelocityConstraint::setAngularVelocityWeight(double newAngularVelocityWeight)
{
    angularVelocityWeight = newAngularVelocityWeight;
}

// ===========================
// INVERSE VELOCITY KINEMATICS
// ===========================

InverseVelocityKinematics::InverseVelocityKinematics():
    pImpl{new impl()}
{}

InverseVelocityKinematics::~InverseVelocityKinematics()
{}

bool InverseVelocityKinematics::setModel(iDynTree::Model model)
{
    pImpl->dofs = model.getNrOfDOFs();
    pImpl->model = model;

    bool result = pImpl->dynamics.loadRobotModel(model);
    if (!result || !pImpl->dynamics.isValid()) {
        std::cerr << "[ERROR] Error loading robot model" << std::endl;
        return false;
    }

    clearProblem();

    updateRobotConfiguration();

    return true;
}
