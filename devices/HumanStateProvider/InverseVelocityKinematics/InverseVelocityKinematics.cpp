/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "InverseVelocityKinematics.hpp"

#include <Eigen/QR>
#include <Eigen/SparseCholesky>
#include <iDynTree/Core/EigenHelpers.h>
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

    struct
    {
        iDynTree::VectorDynSize jointsConfiguration;
        iDynTree::Transform basePose;
        iDynTree::VectorDynSize jointsVelocity;
        iDynTree::Twist baseTwist;
        iDynTree::Vector3 worldGravity;
    } state;

    InverseVelocityKinematicsResolutionMode resolutionMode;

    class VelocityConstraint;
    typedef std::map<int, VelocityConstraint> VelocityMap;
    VelocityMap velocityTargets;

    size_t numberOfTargetVariables;
    double regularizationWeight;

    iDynTree::Twist baseVelocityResult;
    iDynTree::VectorDynSize jointVelocityResult;

    iDynTree::MatrixDynSize fullJacobianBuffer;
    iDynTree::VectorDynSize fullVelocityBuffer;
    iDynTree::VectorDynSize weightVectorBuffer;
    iDynTree::MatrixDynSize regularizationMatrixBuffer;

    bool problemInitialized;

    impl();

    bool updateConfiguration();

    bool addTarget(VelocityConstraint frameConstraint);

    void updateLinearVelocityTarget(VelocityMap::iterator target,
                                    iDynTree::Vector3 newLinearVelocity,
                                    double newLinearVelocityWeight);
    void updateAngularVelocityTarget(VelocityMap::iterator target,
                                     iDynTree::Vector3 newLAngularVelocity,
                                     double newAngularVelocityWeight);

    VelocityMap::iterator getTargetRefIfItExists(const std::string targetFrameName);

    bool solveProblem();

    bool solveWeightedPseudoInverse(iDynTree::MatrixDynSize matrix,
                                    iDynTree::VectorDynSize inputVector,
                                    iDynTree::VectorDynSize& outputVector,
                                    iDynTree::VectorDynSize weightVector,
                                    iDynTree::MatrixDynSize regularizationMatrix);

    void computeTargetSize();
    void computeProblemSizeAndResizeBuffers();

    void prepareFullVelocityVector();
    void prepareFullJacobianMatrix();
    void prepareWeightVector();
};

// ===================
// VELOCITY CONSTRAINT
// ===================

class InverseVelocityKinematics::impl::VelocityConstraint
{
public:
    enum VelocityConstraintType
    {
        VelocityConstraintTypeLinearVelocity,
        VelocityConstraintTypeAngularVelocity,
        VelocityConstraintTypeTwist,
    };

    iDynTree::Twist twist;
    VelocityConstraintType type;
    std::string frameName;
    double linearVelocityWeight;
    double angularVelocityWeight;

    VelocityConstraint(std::string frameName, VelocityConstraintType type);

    static VelocityConstraint linearVelocityConstraint(std::string frameName,
                                                       iDynTree::Vector3 linearVelocity,
                                                       double linearVelocityWeight = 1.0);
    static VelocityConstraint angularVelocityConstraint(std::string frameName,
                                                        iDynTree::Vector3 angularVelocity,
                                                        double angularVelocityWeight = 1.0);
    static VelocityConstraint TwistConstraint(std::string frameName,
                                              iDynTree::Vector3 linearVelocity,
                                              iDynTree::Vector3 angularVelocity,
                                              double linearVelocityWeight = 1.0,
                                              double angularVelocityWeight = 1.0);
    static VelocityConstraint TwistConstraint(std::string frameName,
                                              iDynTree::Twist twist,
                                              double linearVelocityWeight = 1.0,
                                              double angularVelocityWeight = 1.0);

    VelocityConstraintType getType();
    std::string getFrameName();

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

// ====
// IMPL
// ====

InverseVelocityKinematics::impl::impl()
    : dofs(0)
    , resolutionMode(pseudoinverse)
    , numberOfTargetVariables(0)
    , regularizationWeight(1E-8)
    , problemInitialized(false)
{
    // These variables are touched only once.
    state.worldGravity.zero();
}

bool InverseVelocityKinematics::impl::updateConfiguration()
{

    return dynamics.setRobotState(state.basePose,
                                  state.jointsConfiguration,
                                  state.baseTwist,
                                  state.jointsVelocity,
                                  state.worldGravity);
}

bool InverseVelocityKinematics::impl::addTarget(VelocityConstraint frameConstraint)
{
    int frameIndex = dynamics.getFrameIndex(frameConstraint.getFrameName());
    if (frameIndex < 0)
        return false;

    std::pair<VelocityMap::iterator, bool> result =
        velocityTargets.insert(VelocityMap::value_type(frameIndex, frameConstraint));

    problemInitialized = false;
    return result.second;
}

void InverseVelocityKinematics::impl::updateLinearVelocityTarget(
    VelocityMap::iterator target,
    iDynTree::Vector3 newLinearVelocity,
    double newLinearVelocityWeight)
{
    assert(target != velocityTargets.end());
    target->second.setLinearVelocity(newLinearVelocity);
    target->second.setLinearVelocityWeight(newLinearVelocityWeight);
}

void InverseVelocityKinematics::impl::updateAngularVelocityTarget(
    VelocityMap::iterator target,
    iDynTree::Vector3 newLAngularVelocity,
    double newAngularVelocityWeight)
{
    assert(target != velocityTargets.end());
    target->second.setAngularVelocity(newLAngularVelocity);
    target->second.setAngularVelocityWeight(newAngularVelocityWeight);
}

InverseVelocityKinematics::impl::VelocityMap::iterator
InverseVelocityKinematics::impl::getTargetRefIfItExists(const std::string targetFrameName)
{
    int frameIndex = dynamics.getFrameIndex(targetFrameName);
    if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
        return velocityTargets.end();

    // Find the target (if this fails, it will return m_targets.end()
    return velocityTargets.find(frameIndex);
}

bool InverseVelocityKinematics::impl::solveProblem()
{
    if (!problemInitialized) {
        computeProblemSizeAndResizeBuffers();
    }

    prepareFullVelocityVector();
    prepareFullJacobianMatrix();
    prepareWeightVector();

    iDynTree::VectorDynSize nu;
    solveWeightedPseudoInverse(
        fullJacobianBuffer, fullVelocityBuffer, nu, weightVectorBuffer, regularizationMatrixBuffer);

    baseVelocityResult.setVal(0, nu.getVal(0));
    baseVelocityResult.setVal(1, nu.getVal(1));
    baseVelocityResult.setVal(2, nu.getVal(2));
    baseVelocityResult.setVal(3, nu.getVal(3));
    baseVelocityResult.setVal(4, nu.getVal(4));
    baseVelocityResult.setVal(5, nu.getVal(5));

    for (int k = 0; k < dofs; k++) {
        jointVelocityResult.setVal(k, nu.getVal(k + 6));
    }

    double max_velocity_val = 10.0;
    // Add here the joint velocity limits
    for (unsigned i = 0; i < jointVelocityResult.size(); i++) {
        if (jointVelocityResult.getVal(i) > max_velocity_val) {
            yWarning() << "joint velocity out of limit: " << i << " : "
                       << jointVelocityResult.getVal(i);
            iDynTree::iDynTreeEigenMatrixMap fullJacobian = iDynTree::toEigen(fullJacobianBuffer);
            yInfo() << "InverseVelocityKinematics::impl:: jacobian " << fullJacobian.rows()
                    << fullJacobian.cols();
            //            yInfo() << "JACOBIAN:";

            //            for (unsigned i = 0; i < fullJacobianBuffer.rows(); i++) {
            //                for (unsigned j = 0; j < fullJacobianBuffer.cols(); j++) {
            //                    std::cout << fullJacobianBuffer.getVal(i, j);
            //                    if (j < fullJacobianBuffer.cols() - 1) {
            //                        std::cout << " , ";
            //                    }
            //                    else {
            //                        std::cout << " ;" << std::endl;
            //                    }
            //                }
            //            }

            //            yInfo() << "InverseVelocityKinematics::impl:: jacobian buffer"
            //                    << fullJacobianBuffer.toString();
            //            yInfo() << "velocities: " << fullVelocityBuffer.toString();
        }
        else if (jointVelocityResult.getVal(i) < (-1.0 * max_velocity_val)) {
            yWarning() << "joint velocity out of limit: " << i << " : "
                       << jointVelocityResult.getVal(i);
            iDynTree::iDynTreeEigenMatrixMap fullJacobian = iDynTree::toEigen(fullJacobianBuffer);
            //            yInfo() << "InverseVelocityKinematics::impl:: jacobian Eigen values: "
            //                    << fullJacobian.eigenvalues().transpose().rows();
            yInfo() << "velocities: " << fullVelocityBuffer.toString();
        }
    }

    return true;
}

bool InverseVelocityKinematics::impl::solveWeightedPseudoInverse(
    iDynTree::MatrixDynSize matrix,
    iDynTree::VectorDynSize inputVector,
    iDynTree::VectorDynSize& outputVector,
    iDynTree::VectorDynSize weightVector,
    iDynTree::MatrixDynSize regularizationMatrix)
{
    if (inputVector.size() != matrix.rows() || matrix.cols() != regularizationMatrix.rows())
        return false;

    outputVector.resize(matrix.cols());
    yInfo() << "reg: " << regularizationMatrix.getVal(1, 1) << regularizationMatrix.getVal(2, 2)
            << regularizationMatrix.getVal(3, 3) << regularizationMatrix.getVal(4, 4);

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> weightInverse(weightVector.size());
    weightInverse =
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(iDynTree::toEigen(weightVector)).inverse();

    //**************************************************
    // Pseudo inverse: CompleteOrthogonalDecomposition: // 7-9 msec
    //**************************************************
    //    Eigen::CompleteOrthogonalDecomposition<
    //        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
    //        WeightedJacobian = (iDynTree::toEigen(matrix).transpose() *
    //        weightInverse.toDenseMatrix()
    //                                * iDynTree::toEigen(matrix)
    //                            + iDynTree::toEigen(regularizationMatrix))
    //                               .completeOrthogonalDecomposition();
    //    WeightedJacobian.setThreshold(1e-2);

    //    iDynTree::toEigen(outputVector) =
    //        WeightedJacobian.pseudoInverse() * iDynTree::toEigen(matrix).transpose()
    //        * weightInverse.toDenseMatrix() * iDynTree::toEigen(inputVector);
    //    std::cout << "rank: " << WeightedJacobian.rank()
    //              << " threshhold: " << WeightedJacobian.threshold() << std::endl;

    //**************************************************
    // simple inversion: // 5-6 msec
    //**************************************************

    //    iDynTree::toEigen(outputVector) =
    //        (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //             * iDynTree::toEigen(matrix)
    //         + iDynTree::toEigen(regularizationMatrix))
    //            .inverse()
    //        * iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //        * iDynTree::toEigen(inputVector);

    //**************************************************
    // Least Square Solving: // 50-60 msec
    //**************************************************
    //    iDynTree::toEigen(outputVector) =
    //        (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //             * iDynTree::toEigen(matrix)
    //         + iDynTree::toEigen(regularizationMatrix))
    //            .bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
    //            .solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //                   * iDynTree::toEigen(inputVector));
    //     Eigen::ComputeThinU | Eigen::ComputeThinV
    //     Eigen::ComputeFullU | Eigen::ComputeFullV

    //**************************************************
    // Robust Cholesky decomposition (LDL^T): // 3-4 msec
    //**************************************************
    //    iDynTree::toEigen(outputVector) =
    //        (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //             * iDynTree::toEigen(matrix)
    //         + iDynTree::toEigen(regularizationMatrix))
    //            .ldlt()
    //            .solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //                   * iDynTree::toEigen(inputVector));

    //**************************************************
    // Standard Cholesky decomposition (LL^T): // 2-3 msec
    //**************************************************
    //    iDynTree::toEigen(outputVector) =
    //        (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //             * iDynTree::toEigen(matrix)
    //         + iDynTree::toEigen(regularizationMatrix))
    //            .llt()
    //            .solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //                   * iDynTree::toEigen(inputVector));

    //**************************************************
    // Sparse Robust Cholesky decomposition (SLDL^T): // 4-5 msec
    //**************************************************

    //    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    //    solver.compute((iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //                        * iDynTree::toEigen(matrix)
    //                    + iDynTree::toEigen(regularizationMatrix))
    //                       .sparseView());

    //    iDynTree::toEigen(outputVector) =
    //        solver.solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //                     * iDynTree::toEigen(inputVector));

    //**************************************************
    // Sparse Standard Cholesky decomposition (SLDL^T): // 3 msec
    //**************************************************

    //    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
    //    solver.compute((iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //                        * iDynTree::toEigen(matrix)
    //                    + iDynTree::toEigen(regularizationMatrix))
    //                       .sparseView());

    //    iDynTree::toEigen(outputVector) =
    //        solver.solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
    //                     * iDynTree::toEigen(inputVector));

    // play with the gains of the regularization term, add the manipulability term maybe, or
    // adding SVD term.

    return true;
}

void InverseVelocityKinematics::impl::computeTargetSize()
{
    numberOfTargetVariables = 0;
    for (VelocityMap::const_iterator target = velocityTargets.begin();
         target != velocityTargets.end();
         ++target) {
        if (target->second.type == VelocityConstraint::VelocityConstraintTypeTwist) {
            numberOfTargetVariables += 6;
        }
        else {
            numberOfTargetVariables += 3;
        }
    }
}

void InverseVelocityKinematics::impl::computeProblemSizeAndResizeBuffers()
{
    computeTargetSize();

    fullJacobianBuffer.resize(numberOfTargetVariables, 6 + dofs);
    fullJacobianBuffer.zero();
    fullVelocityBuffer.resize(numberOfTargetVariables);
    fullVelocityBuffer.zero();
    weightVectorBuffer.resize(numberOfTargetVariables);
    weightVectorBuffer.zero();

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> identityMatrix(6 + dofs);
    identityMatrix.setIdentity();
    regularizationMatrixBuffer.resize(6 + dofs, 6 + dofs);
    iDynTree::toEigen(regularizationMatrixBuffer) =
        identityMatrix.toDenseMatrix() * regularizationWeight;

    problemInitialized = true;
}

void InverseVelocityKinematics::impl::prepareFullVelocityVector()
{
    unsigned int vectorIndex = 0;

    // TODO this should be done by filling the sub-blocks trough Eigen maps
    for (VelocityMap::const_iterator target = velocityTargets.begin();
         target != velocityTargets.end();
         ++target) {
        if (target->second.type == VelocityConstraint::VelocityConstraintTypeTwist) {
            fullVelocityBuffer.setVal(vectorIndex, target->second.twist.getLinearVec3().getVal(0));
            fullVelocityBuffer.setVal(vectorIndex + 1,
                                      target->second.twist.getLinearVec3().getVal(1));
            fullVelocityBuffer.setVal(vectorIndex + 2,
                                      target->second.twist.getLinearVec3().getVal(2));
            fullVelocityBuffer.setVal(vectorIndex + 3,
                                      target->second.twist.getAngularVec3().getVal(0));
            fullVelocityBuffer.setVal(vectorIndex + 4,
                                      target->second.twist.getAngularVec3().getVal(1));
            fullVelocityBuffer.setVal(vectorIndex + 5,
                                      target->second.twist.getAngularVec3().getVal(2));
            vectorIndex += 6;
        }
        else if (target->second.type == VelocityConstraint::VelocityConstraintTypeLinearVelocity) {
            fullVelocityBuffer.setVal(vectorIndex, target->second.twist.getLinearVec3().getVal(0));
            fullVelocityBuffer.setVal(vectorIndex + 1,
                                      target->second.twist.getLinearVec3().getVal(1));
            fullVelocityBuffer.setVal(vectorIndex + 2,
                                      target->second.twist.getLinearVec3().getVal(2));
            vectorIndex += 3;
        }
        else if (target->second.type == VelocityConstraint::VelocityConstraintTypeAngularVelocity) {
            fullVelocityBuffer.setVal(vectorIndex, target->second.twist.getAngularVec3().getVal(0));
            fullVelocityBuffer.setVal(vectorIndex + 1,
                                      target->second.twist.getAngularVec3().getVal(1));
            fullVelocityBuffer.setVal(vectorIndex + 2,
                                      target->second.twist.getAngularVec3().getVal(2));
            vectorIndex += 3;
        }
    }
}

void InverseVelocityKinematics::impl::prepareFullJacobianMatrix()
{
    unsigned int rowIndex = 0;

    iDynTree::iDynTreeEigenMatrixMap fullJacobian = iDynTree::toEigen(fullJacobianBuffer);
    iDynTree::MatrixDynSize frameJacobian(6, 6 + dofs);

    for (VelocityMap::const_iterator target = velocityTargets.begin();
         target != velocityTargets.end();
         ++target) {

        dynamics.getFrameFreeFloatingJacobian(target->second.frameName, frameJacobian);

        if (target->second.type == VelocityConstraint::VelocityConstraintTypeTwist) {
            fullJacobian.block(rowIndex, 0, 6, 6 + dofs) = iDynTree::toEigen(frameJacobian);
            rowIndex += 6;
        }
        else if (target->second.type == VelocityConstraint::VelocityConstraintTypeLinearVelocity) {
            fullJacobian.block(rowIndex, 0, 3, 6 + dofs) =
                iDynTree::toEigen(frameJacobian).topRows(3);
            rowIndex += 3;
        }
        else if (target->second.type == VelocityConstraint::VelocityConstraintTypeAngularVelocity) {
            fullJacobian.block(rowIndex, 0, 3, 6 + dofs) =
                iDynTree::toEigen(frameJacobian).bottomRows(3);
            rowIndex += 3;
        }
    }
}

void InverseVelocityKinematics::impl::prepareWeightVector()
{
    unsigned int vectorIndex = 0;

    // TODO this should be done by filling the sub-blocks trough Eigen maps
    for (VelocityMap::const_iterator target = velocityTargets.begin();
         target != velocityTargets.end();
         ++target) {
        if (target->second.type == VelocityConstraint::VelocityConstraintTypeTwist) {
            weightVectorBuffer.setVal(vectorIndex, target->second.linearVelocityWeight);
            weightVectorBuffer.setVal(vectorIndex + 1, target->second.linearVelocityWeight);
            weightVectorBuffer.setVal(vectorIndex + 2, target->second.linearVelocityWeight);
            weightVectorBuffer.setVal(vectorIndex + 3, target->second.angularVelocityWeight);
            weightVectorBuffer.setVal(vectorIndex + 4, target->second.angularVelocityWeight);
            weightVectorBuffer.setVal(vectorIndex + 5, target->second.angularVelocityWeight);
            vectorIndex += 6;
        }
        else if (target->second.type == VelocityConstraint::VelocityConstraintTypeLinearVelocity) {
            weightVectorBuffer.setVal(vectorIndex, target->second.linearVelocityWeight);
            weightVectorBuffer.setVal(vectorIndex + 1, target->second.linearVelocityWeight);
            weightVectorBuffer.setVal(vectorIndex + 2, target->second.linearVelocityWeight);
            vectorIndex += 3;
        }
        else if (target->second.type == VelocityConstraint::VelocityConstraintTypeAngularVelocity) {
            weightVectorBuffer.setVal(vectorIndex, target->second.angularVelocityWeight);
            weightVectorBuffer.setVal(vectorIndex + 1, target->second.angularVelocityWeight);
            weightVectorBuffer.setVal(vectorIndex + 2, target->second.angularVelocityWeight);
            vectorIndex += 3;
        }
    }
}

// ===================
// VELOCITY CONSTRAINT
// ===================

InverseVelocityKinematics::impl::VelocityConstraint::VelocityConstraint(std::string frameName,
                                                                        VelocityConstraintType type)
    : type(type)
    , frameName(frameName)
    , linearVelocityWeight(1.0)
    , angularVelocityWeight(1.0)
{}

InverseVelocityKinematics::impl::VelocityConstraint
InverseVelocityKinematics::impl::VelocityConstraint::linearVelocityConstraint(
    std::string frameName,
    iDynTree::Vector3 linearVelocity,
    double linearVelocityWeight)
{
    VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeLinearVelocity);
    velocityConstraint.setLinearVelocity(linearVelocity);
    velocityConstraint.setLinearVelocityWeight(linearVelocityWeight);
    return velocityConstraint;
}

InverseVelocityKinematics::impl::VelocityConstraint
InverseVelocityKinematics::impl::VelocityConstraint::angularVelocityConstraint(
    std::string frameName,
    iDynTree::Vector3 angularVelocity,
    double angularVelocityWeight)
{
    VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeAngularVelocity);
    velocityConstraint.setLinearVelocity(angularVelocity);
    velocityConstraint.setLinearVelocityWeight(angularVelocityWeight);
    return velocityConstraint;
}

InverseVelocityKinematics::impl::VelocityConstraint
InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(
    std::string frameName,
    iDynTree::Vector3 linearVelocity,
    iDynTree::Vector3 angularVelocity,
    double linearVelocityWeight,
    double angularVelocityWeight)
{
    iDynTree::Twist twist(linearVelocity, angularVelocity);
    return TwistConstraint(frameName, twist, linearVelocityWeight, angularVelocityWeight);
}

InverseVelocityKinematics::impl::VelocityConstraint
InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(std::string frameName,
                                                                     iDynTree::Twist twist,
                                                                     double linearVelocityWeight,
                                                                     double angularVelocityWeight)
{
    VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeTwist);
    velocityConstraint.setTwist(twist);
    velocityConstraint.setLinearVelocityWeight(linearVelocityWeight);
    velocityConstraint.setAngularVelocityWeight(angularVelocityWeight);
    return velocityConstraint;
}

InverseVelocityKinematics::impl::VelocityConstraint::VelocityConstraintType
InverseVelocityKinematics::impl::VelocityConstraint::getType()
{
    return type;
}

std::string InverseVelocityKinematics::impl::VelocityConstraint::getFrameName()
{
    return frameName;
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

void InverseVelocityKinematics::impl::VelocityConstraint::setLinearVelocity(
    iDynTree::Vector3 newLinearVelocity)
{
    twist.setLinearVec3(newLinearVelocity);
}

iDynTree::Vector3 InverseVelocityKinematics::impl::VelocityConstraint::getAngularVelocity()
{
    return twist.getAngularVec3();
}

void InverseVelocityKinematics::impl::VelocityConstraint::setAngularVelocity(
    iDynTree::Vector3 newAngularVelocity)
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

void InverseVelocityKinematics::impl::VelocityConstraint::setLinearVelocityWeight(
    double newLinearVelocityWeight)
{
    linearVelocityWeight = newLinearVelocityWeight;
}

double InverseVelocityKinematics::impl::VelocityConstraint::getAngularVelocityWeight()
{
    return angularVelocityWeight;
}

void InverseVelocityKinematics::impl::VelocityConstraint::setAngularVelocityWeight(
    double newAngularVelocityWeight)
{
    angularVelocityWeight = newAngularVelocityWeight;
}

// ===========================
// INVERSE VELOCITY KINEMATICS
// ===========================

InverseVelocityKinematics::InverseVelocityKinematics()
    : pImpl{new impl()}
{}

InverseVelocityKinematics::~InverseVelocityKinematics() {}

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

    pImpl->updateConfiguration();

    return true;
}

bool InverseVelocityKinematics::setFloatingBaseOnFrameNamed(std::string floatingBaseFrameName)
{
    return pImpl->dynamics.setFloatingBase(floatingBaseFrameName);
}

void InverseVelocityKinematics::setResolutionMode(
    InverseVelocityKinematicsResolutionMode resolutionMode)
{
    pImpl->resolutionMode = resolutionMode;
}

void InverseVelocityKinematics::setRegularization(double regularizationWeight)
{
    pImpl->regularizationWeight = regularizationWeight;
}

bool InverseVelocityKinematics::addTarget(std::string linkName,
                                          iDynTree::Vector3 linearVelocity,
                                          iDynTree::Vector3 angularVelocity,
                                          double linearWeight,
                                          double angularWeight)
{
    assert(pImpl);
    return pImpl->addTarget(InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(
        linkName, linearVelocity, angularVelocity, linearWeight, angularWeight));
}

bool InverseVelocityKinematics::addTarget(std::string linkName,
                                          iDynTree::Twist twist,
                                          double linearWeight,
                                          double angularWeight)
{
    assert(pImpl);
    return pImpl->addTarget(InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(
        linkName, twist, linearWeight, angularWeight));
}

bool InverseVelocityKinematics::addLinearVelocityTarget(std::string linkName,
                                                        iDynTree::Vector3 linearVelocity,
                                                        double linearWeight)
{
    return pImpl->addTarget(
        InverseVelocityKinematics::impl::VelocityConstraint::linearVelocityConstraint(
            linkName, linearVelocity, linearWeight));
}

bool InverseVelocityKinematics::addLinearVelocityTarget(std::string linkName,
                                                        iDynTree::Twist twist,
                                                        double linearWeight)
{
    return pImpl->addTarget(
        InverseVelocityKinematics::impl::VelocityConstraint::linearVelocityConstraint(
            linkName, twist.getLinearVec3(), linearWeight));
}

bool InverseVelocityKinematics::addAngularVelocityTarget(std::string linkName,
                                                         iDynTree::Vector3 angularVelocity,
                                                         double angularWeight)
{
    return pImpl->addTarget(
        InverseVelocityKinematics::impl::VelocityConstraint::angularVelocityConstraint(
            linkName, angularVelocity, angularWeight));
}

bool InverseVelocityKinematics::addAngularVelocityTarget(std::string linkName,
                                                         iDynTree::Twist twist,
                                                         double angularWeight)
{
    return pImpl->addTarget(
        InverseVelocityKinematics::impl::VelocityConstraint::angularVelocityConstraint(
            linkName, twist.getAngularVec3(), angularWeight));
}

bool InverseVelocityKinematics::setJointConfiguration(std::string jointName,
                                                      double jointConfiguration)
{
    iDynTree::JointIndex jointIndex = pImpl->dynamics.model().getJointIndex(jointName);
    if (jointIndex == iDynTree::JOINT_INVALID_INDEX)
        return false;
    pImpl->state.jointsConfiguration(jointIndex) = jointConfiguration;
    pImpl->updateConfiguration();
    return true;
}

bool InverseVelocityKinematics::setJointsConfiguration(iDynTree::VectorDynSize jointsConfiguration)
{
    assert(pImpl->state.jointsConfiguration.size() == jointsConfiguration.size());
    pImpl->state.jointsConfiguration = jointsConfiguration;
    pImpl->updateConfiguration();
    return true;
}

bool InverseVelocityKinematics::setBasePose(iDynTree::Transform baseTransform)
{
    pImpl->state.basePose = baseTransform;
    pImpl->updateConfiguration();
    return true;
}

bool InverseVelocityKinematics::setBasePose(iDynTree::Vector3 basePosition,
                                            iDynTree::Rotation baseRotation)
{
    iDynTree::Position _basePosition;
    iDynTree::toEigen(_basePosition) = iDynTree::toEigen(basePosition);
    pImpl->state.basePose.setPosition(_basePosition);
    pImpl->state.basePose.setRotation(baseRotation);
    pImpl->updateConfiguration();
    return true;
}

bool InverseVelocityKinematics::setConfiguration(iDynTree::Transform baseTransform,
                                                 iDynTree::VectorDynSize jointsConfiguration)
{
    assert(setJointsConfiguration(jointsConfiguration) && setBasePose(baseTransform));
    pImpl->updateConfiguration();
    return true;
}

bool InverseVelocityKinematics::setConfiguration(iDynTree::Vector3 basePosition,
                                                 iDynTree::Rotation baseRotation,
                                                 iDynTree::VectorDynSize jointsConfiguration)
{
    assert(setJointsConfiguration(jointsConfiguration) && setBasePose(basePosition, baseRotation));
    pImpl->updateConfiguration();
    return true;
}

bool InverseVelocityKinematics::updateTarget(std::string linkName,
                                             iDynTree::Vector3 linearVelocity,
                                             iDynTree::Vector3 angularVelocity,
                                             double linearWeight,
                                             double angularWeight)
{
    InverseVelocityKinematics::impl::VelocityMap::iterator targetConstr =
        pImpl->getTargetRefIfItExists(linkName);

    if (targetConstr == pImpl->velocityTargets.end()) {
        std::stringstream ss;
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateLinearVelocityTarget(targetConstr, linearVelocity, linearWeight);
    pImpl->updateAngularVelocityTarget(targetConstr, angularVelocity, angularWeight);
    return true;
}

bool InverseVelocityKinematics::updateTarget(std::string linkName,
                                             iDynTree::Twist twist,
                                             double linearWeight,
                                             double angularWeight)
{
    return updateTarget(
        linkName, twist.getLinearVec3(), twist.getAngularVec3(), linearWeight, angularWeight);
}

bool InverseVelocityKinematics::updateLinearVelocityTarget(std::string linkName,
                                                           iDynTree::Vector3 linearVelocity,
                                                           double linearWeight)
{
    InverseVelocityKinematics::impl::VelocityMap::iterator targetConstr =
        pImpl->getTargetRefIfItExists(linkName);

    if (targetConstr == pImpl->velocityTargets.end()) {
        std::stringstream ss;
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateLinearVelocityTarget(targetConstr, linearVelocity, linearWeight);
    return true;
}

bool InverseVelocityKinematics::updateAngularVelocityTarget(std::string linkName,
                                                            iDynTree::Vector3 angularVelocity,
                                                            double angularWeight)
{
    InverseVelocityKinematics::impl::VelocityMap::iterator targetConstr =
        pImpl->getTargetRefIfItExists(linkName);

    if (targetConstr == pImpl->velocityTargets.end()) {
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateAngularVelocityTarget(targetConstr, angularVelocity, angularWeight);
    return true;
}

bool InverseVelocityKinematics::getVelocitySolution(iDynTree::Twist& baseVelocity,
                                                    iDynTree::VectorDynSize& jointsVelocity)
{
    assert(getJointsVelocitySolution(jointsVelocity) && getBaseVelocitySolution(baseVelocity));
    return true;
}

bool InverseVelocityKinematics::getJointsVelocitySolution(iDynTree::VectorDynSize& jointsVelocity)
{
    assert(jointsVelocity.size() == pImpl->jointVelocityResult.size());
    jointsVelocity = pImpl->jointVelocityResult;
    return true;
}

bool InverseVelocityKinematics::getBaseVelocitySolution(iDynTree::Twist& baseVelocity)
{
    baseVelocity = pImpl->baseVelocityResult;
    return true;
}

bool InverseVelocityKinematics::getBaseVelocitySolution(iDynTree::Vector3& linearVelocity,
                                                        iDynTree::Vector3& angularVelocity)
{
    linearVelocity = pImpl->baseVelocityResult.getLinearVec3();
    angularVelocity = pImpl->baseVelocityResult.getAngularVec3();
    return true;
}

bool InverseVelocityKinematics::solve()
{
    return pImpl->solveProblem();
}

void InverseVelocityKinematics::clearProblem()
{
    pImpl->state.jointsConfiguration.resize(pImpl->dofs);
    pImpl->state.jointsConfiguration.zero();

    pImpl->state.jointsVelocity.resize(pImpl->dofs);
    pImpl->state.jointsVelocity.zero();

    pImpl->state.basePose.setPosition(iDynTree::Position(0, 0, 0));
    pImpl->state.basePose.setRotation(iDynTree::Rotation::Identity());

    pImpl->baseVelocityResult.zero();

    pImpl->jointVelocityResult.resize(pImpl->dofs);
    pImpl->jointVelocityResult.zero();

    pImpl->state.baseTwist.zero();

    pImpl->velocityTargets.clear();

    pImpl->problemInitialized = false;
}
