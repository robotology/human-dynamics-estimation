/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef INVERSEVELOCITYKINEMATICS_HPP
#define INVERSEVELOCITYKINEMATICS_HPP

#include <iDynTree/Model/Model.h>

#include <map>
#include <memory>
#include <yarp/os/LogStream.h>

class InverseVelocityKinematics
{
protected:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    typedef enum
    {
        moorePenrose,
        completeOrthogonalDecomposition,
        leastSquare,
        choleskyDecomposition,
        sparseCholeskyDecomposition,
        robustCholeskyDecomposition,
        sparseRobustCholeskyDecomposition,
    } InverseVelocityKinematicsResolutionMode;

    InverseVelocityKinematics();
    ~InverseVelocityKinematics();

    bool setModel(iDynTree::Model model);
    bool setFloatingBaseOnFrameNamed(std::string floatingBaseFrameName);
    bool setResolutionMode(InverseVelocityKinematicsResolutionMode resolutionMode);
    bool setResolutionMode(std::string resolutionModeName);
    void setRegularization(double regularizationWeight);

    bool addTarget(std::string linkName,
                   iDynTree::Vector3 linearVelocity,
                   iDynTree::Vector3 angularVelocity,
                   double linearWeight = 1.0,
                   double angularWeight = 1.0);
    bool addTarget(std::string linkName,
                   iDynTree::Twist twist,
                   double linearWeight = 1.0,
                   double angularWeight = 1.0);
    bool addLinearVelocityTarget(std::string linkName,
                                 iDynTree::Vector3 linearVelocity,
                                 double linearWeight = 1.0);
    bool
    addLinearVelocityTarget(std::string linkName, iDynTree::Twist twist, double linearWeight = 1.0);
    bool addAngularVelocityTarget(std::string linkName,
                                  iDynTree::Vector3 angularVelocity,
                                  double angularWeight = 1.0);
    bool addAngularVelocityTarget(std::string linkName,
                                  iDynTree::Twist twist,
                                  double angularWeight = 1.0);

    // TODO
    // addFrameVelocityConstraint
    // addFrameLinearVelocityConstraint
    // addFrameAngularVelocityConstraint

    bool setJointConfiguration(std::string jointName, double jointConfiguration);
    bool setJointsConfiguration(iDynTree::VectorDynSize jointsConfiguration);
    bool setBasePose(iDynTree::Transform baseTransform);
    bool setBasePose(iDynTree::Vector3 basePosition, iDynTree::Rotation baseRotation);
    bool setConfiguration(iDynTree::Transform baseTransform,
                          iDynTree::VectorDynSize jointsConfiguration);
    bool setConfiguration(iDynTree::Vector3 basePosition,
                          iDynTree::Rotation baseRotation,
                          iDynTree::VectorDynSize jointsConfiguration);

    // TODO
    // setBaseVelocityLimit
    // setBaseLinearVelocityLimit
    // setBaseAngularVelocityLimit
    // setJointVelocityLimit
    // setJointsVelocityLimit

    bool updateTarget(std::string linkName,
                      iDynTree::Vector3 linearVelocity,
                      iDynTree::Vector3 angularVelocity,
                      double linearWeight = 1.0,
                      double angularWeight = 1.0);
    bool updateTarget(std::string linkName,
                      iDynTree::Twist twist,
                      double linearWeight = 1.0,
                      double angularWeight = 1.0);
    bool updateLinearVelocityTarget(std::string linkName,
                                    iDynTree::Vector3 linearVelocity,
                                    double linearWeight = 1.0);
    bool updateAngularVelocityTarget(std::string linkName,
                                     iDynTree::Vector3 angularVelocity,
                                     double angularWeight = 1.0);

    bool getVelocitySolution(iDynTree::Twist& baseVelocity,
                             iDynTree::VectorDynSize& jointsVelocity);
    bool getJointsVelocitySolution(iDynTree::VectorDynSize& jointsVelocity);
    bool getBaseVelocitySolution(iDynTree::Twist& baseVelocity);
    bool getBaseVelocitySolution(iDynTree::Vector3& linearVelocity,
                                 iDynTree::Vector3& angularVelocity);

    bool solve();
    void clearProblem();
};

#endif // INVERSEVELOCITYKINEMATICS_HPP
