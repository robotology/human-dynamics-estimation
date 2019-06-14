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
// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// ====
// IMPL
// ====

class InverseVelocityKinematics::impl
{
    // qp variables
    iDynTree::VectorDynSize m_u_prime, m_l_prime, m_q_prime;
    iDynTree::MatrixDynSize m_A_prime, m_P_prime;

    iDynTree::VectorDynSize m_u, m_l, m_q;
    iDynTree::MatrixDynSize m_A, m_P;

    std::unique_ptr<OsqpEigen::Solver> m_optimizerSolver; /**< Optimization solver. */

public:
    iDynTree::Model model;
    iDynTree::KinDynComputations dynamics;
    size_t dofs;
    size_t configSpaceSize;

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
    size_t numberOfConstraints;
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

    bool solveIntegrationBasedIK(iDynTree::MatrixDynSize matrix,
                                 iDynTree::VectorDynSize inputVector,
                                 iDynTree::VectorDynSize& outputVector,
                                 iDynTree::VectorDynSize weightVector,
                                 iDynTree::MatrixDynSize regularizationMatrix);

    void computeTargetSize();
    void computeProblemSizeAndResizeBuffers();

    void prepareFullVelocityVector();
    void prepareFullJacobianMatrix();
    void prepareWeightVector();
    void prepareOptimizer();
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
    , resolutionMode(moorePenrose)
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
    target->second.setLinearVelocity(newLinearVelocity);
    target->second.setLinearVelocityWeight(newLinearVelocityWeight);
}

void InverseVelocityKinematics::impl::updateAngularVelocityTarget(
    VelocityMap::iterator target,
    iDynTree::Vector3 newLAngularVelocity,
    double newAngularVelocityWeight)
{
    target->second.setAngularVelocity(newLAngularVelocity);
    target->second.setAngularVelocityWeight(newAngularVelocityWeight);
}

InverseVelocityKinematics::impl::VelocityMap::iterator
InverseVelocityKinematics::impl::getTargetRefIfItExists(const std::string targetFrameName)
{
    int frameIndex = dynamics.getFrameIndex(targetFrameName);
    if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
        return velocityTargets.end();

    // Find the target (if this fails, it will return m_targets.end())
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
    solveIntegrationBasedIK(
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

    //    yInfo() << "IB-IK, normal: ";
    //    for (int k = 0; k < nu.size(); k++) {
    //        std::cout << nu.getVal(k) << " ";
    //    }
    //    std::cout << std::endl;

    return true;
}

bool InverseVelocityKinematics::impl::solveIntegrationBasedIK(
    iDynTree::MatrixDynSize matrix,
    iDynTree::VectorDynSize inputVector,
    iDynTree::VectorDynSize& outputVector,
    iDynTree::VectorDynSize weightVector,
    iDynTree::MatrixDynSize regularizationMatrix)
{
    if (inputVector.size() != matrix.rows() || matrix.cols() != regularizationMatrix.rows()) {
        return false;
    }

    outputVector.resize(matrix.cols());

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> weightInverse(weightVector.size());
    weightInverse =
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(iDynTree::toEigen(weightVector)).inverse();
    //********************* qp implementation
    //    // check changing the matrixxd to sparse matrices if is faster or not!?
    //    unsigned int taskSpaceSize = matrix.rows();
    //    unsigned int configSpaceSize = matrix.cols();
    //    unsigned int NoOfconstraints = configSpaceSize;

    //    // generate P_prime matrix
    //    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(taskSpaceSize, taskSpaceSize);

    //    Eigen::MatrixXd P_prime(taskSpaceSize + configSpaceSize, taskSpaceSize + configSpaceSize);

    //    //    P_prime.topLeftCorner(configSpaceSize, configSpaceSize) =
    //    //        Eigen::MatrixXd::Zero(configSpaceSize, configSpaceSize);
    //    P_prime.topLeftCorner(configSpaceSize, configSpaceSize) =
    //        iDynTree::toEigen(regularizationMatrix).sparseView();
    //    P_prime.topRightCorner(configSpaceSize, taskSpaceSize) =
    //        Eigen::MatrixXd::Zero(configSpaceSize, taskSpaceSize);
    //    P_prime.bottomLeftCorner(taskSpaceSize, configSpaceSize) =
    //        Eigen::MatrixXd::Zero(taskSpaceSize, configSpaceSize);
    //    P_prime.bottomRightCorner(taskSpaceSize, taskSpaceSize) = P;

    //    // generate A_prime matrix for constraints
    //    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(NoOfconstraints, NoOfconstraints);

    //    Eigen::MatrixXd A_prime(NoOfconstraints + taskSpaceSize, configSpaceSize + taskSpaceSize);

    //    A_prime.topLeftCorner(taskSpaceSize, configSpaceSize) =
    //    iDynTree::toEigen(matrix).sparseView(); A_prime.topRightCorner(taskSpaceSize,
    //    taskSpaceSize) =
    //        Eigen::MatrixXd::Identity(taskSpaceSize, taskSpaceSize);
    //    A_prime.bottomLeftCorner(NoOfconstraints, configSpaceSize) = A;
    //    A_prime.bottomRightCorner(NoOfconstraints, taskSpaceSize) =
    //        Eigen::MatrixXd::Zero(NoOfconstraints, taskSpaceSize);
    //    // generate upper limit vector (u_prime) and lower limit vector (l_prime)
    //    Eigen::VectorXd V = iDynTree::toEigen(inputVector);
    //    double jointVelocityLimit = 10.0;

    //    Eigen::VectorXd U = Eigen::VectorXd::Ones(NoOfconstraints, 1) * jointVelocityLimit;
    //    Eigen::VectorXd L = Eigen::VectorXd::Ones(NoOfconstraints, 1) * -1.0 * jointVelocityLimit;

    //    Eigen::VectorXd u_prime(NoOfconstraints + taskSpaceSize);
    //    u_prime.topRows(taskSpaceSize) = V;
    //    u_prime.bottomRows(NoOfconstraints) = U;

    //    Eigen::VectorXd l_prime(NoOfconstraints + taskSpaceSize);
    //    l_prime.topRows(taskSpaceSize) = V;
    //    l_prime.bottomRows(NoOfconstraints) = L;

    //    OsqpEigen::Solver solver;
    //    // solver.settings()->setVerbosity(false);
    //    solver.settings()->setWarmStart(true);
    //    solver.data()->setNumberOfVariables(taskSpaceSize + configSpaceSize);
    //    solver.data()->setNumberOfConstraints(NoOfconstraints + taskSpaceSize);

    //    Eigen::SparseMatrix<double> A_prime_sparse = A_prime.sparseView();
    //    Eigen::SparseMatrix<double> P_prime_sparse = P_prime.sparseView();
    //    Eigen::VectorXd q_prime(taskSpaceSize + configSpaceSize);
    //    q_prime = Eigen::VectorXd::Zero(taskSpaceSize + configSpaceSize, 1);

    //    // set A
    //    if (!solver.data()->setLinearConstraintsMatrix(A_prime_sparse)) {
    //        return 1;
    //    }
    //    // set U
    //    if (!solver.data()->setUpperBound(u_prime)) {
    //        return 1;
    //    }
    //    // set L
    //    if (!solver.data()->setLowerBound(l_prime)) {
    //        return 1;
    //    }
    //    // set P
    //    if (!solver.data()->setHessianMatrix(P_prime_sparse)) {
    //        return 1;
    //    }

    //    // set q
    //    if (!solver.data()->setGradient(q_prime)) {
    //        return 1;
    //    }

    //    if (!solver.initSolver()) {
    //        return 1;
    //    }

    //    // controller QPSolution vector

    yInfo() << "resolution mode: " << resolutionMode;
    if (resolutionMode == QP) {

        if (!m_optimizerSolver->isInitialized()) {
            //            iDynTree::toEigen(m_A_prime).topLeftCorner(numberOfTargetVariables,
            //            configSpaceSize) =
            //                iDynTree::toEigen(matrix).sparseView();
            //            iDynTree::toEigen(m_u_prime).topRows(numberOfTargetVariables) =
            //                iDynTree::toEigen(inputVector);
            //            iDynTree::toEigen(m_l_prime).topRows(numberOfTargetVariables) =
            //                iDynTree::toEigen(inputVector);

            //            // set A
            //            Eigen::SparseMatrix<double> constraintMatrix =
            //                iDynTree::toEigen(m_A_prime).sparseView();
            //            if
            //            (!m_optimizerSolver->data()->setLinearConstraintsMatrix(constraintMatrix))
            //            {
            //                yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
            //                         << "qp solver for [setLinearConstraintsMatrix] returns
            //                         false.";
            //                return 1;
            //            }
            //            yInfo() << "9";

            //            // set U
            //            auto upperBuond = iDynTree::toEigen(m_u_prime);
            //            if (!m_optimizerSolver->data()->setUpperBound(upperBuond)) {
            //                yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
            //                         << "qp solver for [setUpperBound] returns false.";

            //                return 1;
            //            }
            //            yInfo() << "10";

            //            // set L
            //            auto lowerBuond = iDynTree::toEigen(m_l_prime);
            //            if (!m_optimizerSolver->data()->setLowerBound(lowerBuond)) {
            //                yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
            //                         << "qp solver for [setLowerBound] returns false.";

            //                return 1;
            //            }
            //            yInfo() << "11";

            //            // set P
            //            Eigen::SparseMatrix<double> HessianMatrix =
            //            iDynTree::toEigen(m_P_prime).sparseView(); if
            //            (!m_optimizerSolver->data()->setHessianMatrix(HessianMatrix)) {
            //                yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
            //                         << "qp solver for [setHessianMatrix] returns false.";

            //                return 1;
            //            }
            //            yInfo() << "12";

            //            // set q
            //            auto GradientVector = iDynTree::toEigen(m_q_prime);
            //            if (!m_optimizerSolver->data()->setGradient(GradientVector)) {
            //                yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
            //                         << "qp solver for [setGradient] returns false.";
            //                return 1;
            //        }
            yInfo() << "14-2";

            if (!m_optimizerSolver->initSolver()) {
                yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
                         << "qp solver for [initSolver] returns false.";

                return 1;
            }
        }
        else {
            yInfo() << "15";
            iDynTree::toEigen(m_A_prime).topLeftCorner(numberOfTargetVariables, configSpaceSize) =
                iDynTree::toEigen(matrix).sparseView();
            iDynTree::toEigen(m_u_prime).topRows(numberOfTargetVariables) =
                iDynTree::toEigen(inputVector);
            iDynTree::toEigen(m_l_prime).topRows(numberOfTargetVariables) =
                iDynTree::toEigen(inputVector);

            Eigen::SparseMatrix<double> constraintMatrix =
                iDynTree::toEigen(m_A_prime).sparseView();
            yInfo() << "16";
            auto upperBuond = iDynTree::toEigen(m_u_prime);
            auto lowerBuond = iDynTree::toEigen(m_l_prime);
            yInfo() << "17";
            if (!m_optimizerSolver->updateBounds(lowerBuond, upperBuond)) {
                yError() << "[InverseVelocityKinematics::impl::solveIntegrationBasedIK] "
                         << "qp solver for [updateBounds] returns false.";
                return 1;
            }
            yInfo() << "17-1";
            if (!m_optimizerSolver->updateLinearConstraintsMatrix(constraintMatrix)) {
                yError() << "[InverseVelocityKinematics::impl::solveIntegrationBasedIK] "
                         << "qp solver for [updateLinearConstraintsMatrix] returns false.";
                return 1;
            }
            yInfo() << "18";
        }
        yInfo() << "19";

        //        Eigen::VectorXd QPSolution;
        // solve the QP problem   fullVelocityBuffer
        if (!m_optimizerSolver->solve()) {
            yError() << "[InverseVelocityKinematics::impl::solveIntegrationBasedIK] "
                     << "qp solver for [solve] returns false.";

            return 1;
        }

        // get the controller input
        //        QPSolution = m_optimizerSolver->getSolution();

        //    yInfo() << "IB-IK qp: ";
        //    for (unsigned int i = 0; i < configSpaceSize; i++) {
        //        std::cout << QPSolution.coeff(i, 0) << " ";
        //    }
        //    std::cout << std::endl;

        //        iDynTree::toEigen(outputVector) = QPSolution.topRows(configSpaceSize);
        iDynTree::toEigen(outputVector) = m_optimizerSolver->getSolution().topRows(configSpaceSize);
        //        yInfo() << "running qp";

        //**********************************
    }
    else if (resolutionMode == moorePenrose) {
        iDynTree::toEigen(outputVector) =
            (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                 * iDynTree::toEigen(matrix)
             + iDynTree::toEigen(regularizationMatrix))
                .inverse()
            * iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
            * iDynTree::toEigen(inputVector);
    }
    else if (resolutionMode == completeOrthogonalDecomposition) {
        Eigen::CompleteOrthogonalDecomposition<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            WeightedJacobian = (iDynTree::toEigen(matrix).transpose()
                                    * weightInverse.toDenseMatrix() * iDynTree::toEigen(matrix)
                                + iDynTree::toEigen(regularizationMatrix))
                                   .completeOrthogonalDecomposition();
        WeightedJacobian.setThreshold(1e-2);

        iDynTree::toEigen(outputVector) =
            WeightedJacobian.pseudoInverse() * iDynTree::toEigen(matrix).transpose()
            * weightInverse.toDenseMatrix() * iDynTree::toEigen(inputVector);
        std::cout << "rank: " << WeightedJacobian.rank()
                  << " threshhold: " << WeightedJacobian.threshold() << std::endl;
    }
    else if (resolutionMode == leastSquare) {
        iDynTree::toEigen(outputVector) =
            (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                 * iDynTree::toEigen(matrix)
             + iDynTree::toEigen(regularizationMatrix))
                .bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
                .solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                       * iDynTree::toEigen(inputVector));
    }
    else if (resolutionMode == choleskyDecomposition) {
        iDynTree::toEigen(outputVector) =
            (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                 * iDynTree::toEigen(matrix)
             + iDynTree::toEigen(regularizationMatrix))
                .llt()
                .solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                       * iDynTree::toEigen(inputVector));
    }
    else if (resolutionMode == sparseCholeskyDecomposition) {
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
        solver.compute((iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                            * iDynTree::toEigen(matrix)
                        + iDynTree::toEigen(regularizationMatrix))
                           .sparseView());

        iDynTree::toEigen(outputVector) =
            solver.solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                         * iDynTree::toEigen(inputVector));
    }
    else if (resolutionMode == robustCholeskyDecomposition) {
        iDynTree::toEigen(outputVector) =
            (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                 * iDynTree::toEigen(matrix)
             + iDynTree::toEigen(regularizationMatrix))
                .ldlt()
                .solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                       * iDynTree::toEigen(inputVector));
    }
    else if (resolutionMode == sparseRobustCholeskyDecomposition) {
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute((iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                            * iDynTree::toEigen(matrix)
                        + iDynTree::toEigen(regularizationMatrix))
                           .sparseView());

        iDynTree::toEigen(outputVector) =
            solver.solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                         * iDynTree::toEigen(inputVector));
    }

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
    prepareOptimizer();

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
void InverseVelocityKinematics::impl::prepareOptimizer()
{
    //    if (resolutionMode == QP) {
    configSpaceSize = dofs + 6;
    numberOfConstraints = dofs;

    // generate P_prime matrix
    m_P = iDynTree::MatrixDynSize(numberOfTargetVariables, numberOfTargetVariables);
    iDynTree::toEigen(m_P).setIdentity();

    m_P_prime = iDynTree::MatrixDynSize(numberOfTargetVariables + configSpaceSize,
                                        numberOfTargetVariables + configSpaceSize);
    iDynTree::toEigen(m_P_prime).topLeftCorner(configSpaceSize, configSpaceSize) =
        iDynTree::toEigen(regularizationMatrixBuffer);
    iDynTree::toEigen(m_P_prime).topRightCorner(configSpaceSize, numberOfTargetVariables) =
        Eigen::MatrixXd::Zero(configSpaceSize, numberOfTargetVariables);
    iDynTree::toEigen(m_P_prime).bottomLeftCorner(numberOfTargetVariables, configSpaceSize) =
        Eigen::MatrixXd::Zero(numberOfTargetVariables, configSpaceSize);
    iDynTree::toEigen(m_P_prime).bottomRightCorner(
        numberOfTargetVariables, numberOfTargetVariables) = iDynTree::toEigen(m_P);

    // generate A_prime matrix for constraints
    m_A = iDynTree::MatrixDynSize(numberOfConstraints, configSpaceSize);
    // to implement, get constraint matrix from the user!
    if (numberOfConstraints == dofs) {
        iDynTree::toEigen(m_A).bottomRows(dofs).setIdentity();
    }
    else {
        yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
                 << " to implement: user defined constraint matrix";
    }
    m_A_prime = iDynTree::MatrixDynSize(numberOfConstraints + numberOfTargetVariables,
                                        configSpaceSize + numberOfTargetVariables);

    iDynTree::toEigen(m_A_prime).topLeftCorner(numberOfTargetVariables, configSpaceSize) =
        Eigen::MatrixXd::Zero(numberOfTargetVariables, configSpaceSize);
    iDynTree::toEigen(m_A_prime)
        .topRightCorner(numberOfTargetVariables, numberOfTargetVariables)
        .setIdentity();
    iDynTree::toEigen(m_A_prime).bottomLeftCorner(numberOfConstraints, configSpaceSize) =
        iDynTree::toEigen(m_A);
    iDynTree::toEigen(m_A_prime).bottomRightCorner(numberOfConstraints, numberOfTargetVariables) =
        Eigen::MatrixXd::Zero(numberOfConstraints, numberOfTargetVariables);
    // generate upper limit vector (u_prime) and lower limit vector (l_prime)
    m_l = iDynTree::VectorDynSize(numberOfConstraints);
    m_u = iDynTree::VectorDynSize(numberOfConstraints);

    // to implement, get velocity vector limit from the configuration files
    double jointVelocityLimit = 10.0;

    iDynTree::toEigen(m_u) = Eigen::VectorXd::Ones(numberOfConstraints, 1) * jointVelocityLimit;
    iDynTree::toEigen(m_l) =
        Eigen::VectorXd::Ones(numberOfConstraints, 1) * -1.0 * jointVelocityLimit;
    m_l_prime = iDynTree::VectorDynSize(numberOfConstraints + numberOfTargetVariables);
    m_u_prime = iDynTree::VectorDynSize(numberOfConstraints + numberOfTargetVariables);
    iDynTree::toEigen(m_u_prime).topRows(numberOfTargetVariables) =
        Eigen::VectorXd::Zero(numberOfTargetVariables, 1);
    iDynTree::toEigen(m_u_prime).bottomRows(numberOfConstraints) = iDynTree::toEigen(m_u);

    iDynTree::toEigen(m_l_prime).topRows(numberOfTargetVariables) =
        Eigen::VectorXd::Zero(numberOfTargetVariables, 1);
    iDynTree::toEigen(m_l_prime).bottomRows(numberOfConstraints) = iDynTree::toEigen(m_l);
    m_optimizerSolver = std::make_unique<OsqpEigen::Solver>();

    // solver.settings()->setVerbosity(false);
    m_optimizerSolver->settings()->setWarmStart(true);
    m_optimizerSolver->settings()->setVerbosity(false);

    m_optimizerSolver->data()->setNumberOfVariables(numberOfTargetVariables + configSpaceSize);
    m_optimizerSolver->data()->setNumberOfConstraints(numberOfConstraints
                                                      + numberOfTargetVariables);

    //        Eigen::SparseMatrix<double> A_prime_sparse = A_prime.sparseView();
    //        Eigen::SparseMatrix<double> P_prime_sparse = P_prime.sparseView();
    //    Eigen::VectorXd q_prime(numberOfTargetVariables + configSpaceSize);
    m_q_prime = iDynTree::VectorDynSize(numberOfTargetVariables + configSpaceSize);
    iDynTree::toEigen(m_q_prime).setZero();

    // set A
    Eigen::SparseMatrix<double> constraintMatrix = iDynTree::toEigen(m_A_prime).sparseView();
    if (!m_optimizerSolver->data()->setLinearConstraintsMatrix(constraintMatrix)) {
        yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
                 << "qp solver for [setLinearConstraintsMatrix] returns false.";
        return;
    }

    // set U
    auto upperBuond = iDynTree::toEigen(m_u_prime);
    if (!m_optimizerSolver->data()->setUpperBound(upperBuond)) {
        yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
                 << "qp solver for [setUpperBound] returns false.";

        return;
    }

    // set L
    auto lowerBuond = iDynTree::toEigen(m_l_prime);
    if (!m_optimizerSolver->data()->setLowerBound(lowerBuond)) {
        yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
                 << "qp solver for [setLowerBound] returns false.";

        return;
    }

    // set P
    Eigen::SparseMatrix<double> HessianMatrix = iDynTree::toEigen(m_P_prime).sparseView();
    if (!m_optimizerSolver->data()->setHessianMatrix(HessianMatrix)) {
        yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
                 << "qp solver for [setHessianMatrix] returns false.";

        return;
    }

    // set q
    auto GradientVector = iDynTree::toEigen(m_q_prime);
    if (!m_optimizerSolver->data()->setGradient(GradientVector)) {
        yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
                 << "qp solver for [setGradient] returns false.";
        return;
    }

    if (!m_optimizerSolver->initSolver()) {
        yError() << "[InverseVelocityKinematics::impl::prepareOptimizer] "
                 << "qp solver for [initSolver] returns false.";

        return;
    }

    // controller QPSolution vector
    //   Eigen::VectorXd QPSolution;
    //    }
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

bool InverseVelocityKinematics::setResolutionMode(
    InverseVelocityKinematicsResolutionMode resolutionMode)
{
    pImpl->resolutionMode = resolutionMode;
    return true;
}

bool InverseVelocityKinematics::setResolutionMode(std::string resolutionModeName)
{
    if (resolutionModeName == "QP") {
        pImpl->resolutionMode = InverseVelocityKinematicsResolutionMode::QP;
    }
    else if (resolutionModeName == "moorePenrose") {
        pImpl->resolutionMode = InverseVelocityKinematicsResolutionMode::moorePenrose;
    }
    else if (resolutionModeName == "completeOrthogonalDecomposition") {
        pImpl->resolutionMode =
            InverseVelocityKinematicsResolutionMode::completeOrthogonalDecomposition;
    }
    else if (resolutionModeName == "leastSquare") {
        pImpl->resolutionMode = InverseVelocityKinematicsResolutionMode::leastSquare;
    }
    else if (resolutionModeName == "choleskyDecomposition") {
        pImpl->resolutionMode = InverseVelocityKinematicsResolutionMode::choleskyDecomposition;
    }
    else if (resolutionModeName == "sparseCholeskyDecomposition") {
        pImpl->resolutionMode =
            InverseVelocityKinematicsResolutionMode::sparseCholeskyDecomposition;
    }
    else if (resolutionModeName == "robustCholeskyDecomposition") {
        pImpl->resolutionMode =
            InverseVelocityKinematicsResolutionMode::robustCholeskyDecomposition;
    }
    else if (resolutionModeName == "sparseRobustCholeskyDecomposition") {
        pImpl->resolutionMode =
            InverseVelocityKinematicsResolutionMode::sparseRobustCholeskyDecomposition;
    }
    else {
        std::cerr << "[ERROR] Invalid resolution mode: " << resolutionModeName << std::endl;
        return false;
    }

    return true;
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
    return pImpl->addTarget(InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(
        linkName, linearVelocity, angularVelocity, linearWeight, angularWeight));
}

bool InverseVelocityKinematics::addTarget(std::string linkName,
                                          iDynTree::Twist twist,
                                          double linearWeight,
                                          double angularWeight)
{
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
    if (pImpl->state.jointsConfiguration.size() == jointsConfiguration.size()) {
        pImpl->state.jointsConfiguration = jointsConfiguration;
        pImpl->updateConfiguration();
        return true;
    }
    else {
        return false;
    }
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
    if (setJointsConfiguration(jointsConfiguration) && setBasePose(baseTransform)) {
        pImpl->updateConfiguration();
        return true;
    }
    else {
        return false;
    }
}

bool InverseVelocityKinematics::setConfiguration(iDynTree::Vector3 basePosition,
                                                 iDynTree::Rotation baseRotation,
                                                 iDynTree::VectorDynSize jointsConfiguration)
{
    if (setJointsConfiguration(jointsConfiguration) && setBasePose(basePosition, baseRotation)) {
        pImpl->updateConfiguration();
        return true;
    }
    else {
        return false;
    }
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
    return getJointsVelocitySolution(jointsVelocity) && getBaseVelocitySolution(baseVelocity);
}

bool InverseVelocityKinematics::getJointsVelocitySolution(iDynTree::VectorDynSize& jointsVelocity)
{
    if (jointsVelocity.size() == pImpl->jointVelocityResult.size()) {
        jointsVelocity = pImpl->jointVelocityResult;
        return true;
    }
    else {
        return false;
    }
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

bool InverseVelocityKinematics::setBaseVelocityLimit(iDynTree::Vector6 lowerBound,
                                                     iDynTree::Vector6 upperBound)
{
    // to implement!
    return true;
}
bool InverseVelocityKinematics::setJointsVelocityLimit(std::vector<std::string> jointsNameList,
                                                       iDynTree::VectorDynSize jointsLimitList)
{
    // to implement!
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
