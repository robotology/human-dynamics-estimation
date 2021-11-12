/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "hde/algorithms/InverseVelocityKinematics.hpp"

#include <hde/utils/iDynTreeUtils.hpp>

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
    size_t dofs; // joint configuration size
    size_t configSpaceSize; // configuration space DoFs (dofs+ baseDofs)
    size_t baseDofs; // base DoFs

    //*********** cosntraints *************//
    //****** general cosntraints *******//
    double m_generalJointVelocityLimit;
    iDynTree::VectorDynSize m_jointsLowerLimits, m_jointsUpperLimits;

    //****** custom constraints *******//
    // custom joint velocities limit
    std::vector<iDynTree::JointIndex> m_customJointsVelocityLimitsIndexes;
    iDynTree::VectorDynSize m_custom_jointsVelocityLimitsUpperBound;
    iDynTree::VectorDynSize m_custom_jointsVelocityLimitsLowerBound;
    // custom base velocities limit
    iDynTree::VectorDynSize m_custom_baseVelocityUpperLimit;
    iDynTree::VectorDynSize m_custom_baseVelocityLowerLimit;
    // custom ineqality constraint for joint values
    iDynTree::MatrixDynSize
        m_custom_ConstraintMatrix; // c (no of constraints) x g (no of variables) size
    std::vector<iDynTree::JointIndex> m_custom_ConstraintVariablesIndex; // gx1 size
    iDynTree::VectorDynSize m_custom_ConstraintUpperBound; // upperBuond, cx1 Vector
    iDynTree::VectorDynSize m_custom_ConstraintLowerBound; // lowerBound, cx1 Vector
    double m_k_u, m_k_l;
    Eigen::MatrixXd coefVectorCustomConstraints;

    iDynTree::VectorDynSize m_JointConfigSpaceConstraintsMax;
    iDynTree::VectorDynSize m_JointConfigSpaceConstraintsMin;

    hde::utils::idyntree::state::State state;

    ResolutionMode resolutionMode;

    iDynTree::Vector3 worldGravity;

    class VelocityConstraint;
    typedef std::map<int, VelocityConstraint> VelocityMap;
    VelocityMap velocityTargets;

    size_t numberOfTargetVariables;
    size_t totalNumberOfConstraints; // overall constraint number :velocity and configuration space
                                     // constraints
    size_t configSpaceNumberOfConstraints; //  configuration space constraint number
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

    bool addTarget(const VelocityConstraint& frameConstraint);

    void updateTargetLinearVelocity(const VelocityMap::iterator& target,
                                    const iDynTree::Vector3& newLinearVelocity,
                                    const double newLinearVelocityWeight);
    void updateTargetAngularVelocity(const VelocityMap::iterator& target,
                                     const iDynTree::Vector3& newAngularVelocity,
                                     const double newAngularVelocityWeight);

    VelocityMap::iterator getTargetRefIfItExists(const std::string& targetFrameName);

    bool solveProblem();

    bool solveInverseDifferentialKinematics(const iDynTree::MatrixDynSize& matrix,
                                            const iDynTree::VectorDynSize& inputVector,
                                            iDynTree::VectorDynSize& outputVector,
                                            const iDynTree::VectorDynSize& weightVector,
                                            const iDynTree::MatrixDynSize& regularizationMatrix);

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

    VelocityConstraint(const std::string& frameName, const VelocityConstraintType& type);

    static VelocityConstraint linearVelocityConstraint(const std::string& frameName,
                                                       const iDynTree::Vector3& linearVelocity,
                                                       const double linearVelocityWeight = 1.0);
    static VelocityConstraint angularVelocityConstraint(const std::string& frameName,
                                                        const iDynTree::Vector3& angularVelocity,
                                                        const double angularVelocityWeight = 1.0);
    static VelocityConstraint TwistConstraint(const std::string& frameName,
                                              const iDynTree::Vector3& linearVelocity,
                                              const iDynTree::Vector3& angularVelocity,
                                              const double linearVelocityWeight = 1.0,
                                              const double angularVelocityWeight = 1.0);
    static VelocityConstraint TwistConstraint(const std::string& frameName,
                                              const iDynTree::Twist& twist,
                                              const double linearVelocityWeight = 1.0,
                                              const double angularVelocityWeight = 1.0);

    VelocityConstraintType getType() const;
    std::string getFrameName() const;

    bool hasLinearVelocityConstraint() const;
    bool hasAngularVelocityConstraint() const;

    iDynTree::Vector3 getLinearVelocity() const;
    void setLinearVelocity(const iDynTree::Vector3& newLinearVelocity);

    iDynTree::Vector3 getAngularVelocity() const;
    void setAngularVelocity(const iDynTree::Vector3& newAngularVelocity);

    iDynTree::Twist getTwist() const;
    void setTwist(const iDynTree::Twist& newTwist);

    double getLinearVelocityWeight() const;
    void setLinearVelocityWeight(const double newLinearVelocityWeight);

    double getAngularVelocityWeight() const;
    void setAngularVelocityWeight(const double newAngularVelocityWeight);
};

// ====
// IMPL
// ====

InverseVelocityKinematics::impl::impl()
    : dofs(0)
    , resolutionMode(ResolutionMode::moorePenrose)
    , numberOfTargetVariables(0)
    , regularizationWeight(1E-8)
    , problemInitialized(false)
{
    // These variables are touched only once.
    worldGravity.zero();
}

bool InverseVelocityKinematics::impl::updateConfiguration()
{

    return dynamics.setRobotState(iDynTree::Transform(state.W_R_B, iDynTree::Position(state.W_p_B)),
                                  state.s,
                                  iDynTree::Twist(state.dot_W_p_B, state.omega_B),
                                  state.dot_s,
                                  worldGravity);
}

bool InverseVelocityKinematics::impl::addTarget(const VelocityConstraint& frameConstraint)
{
    int frameIndex = dynamics.getFrameIndex(frameConstraint.getFrameName());
    if (frameIndex < 0)
        return false;

    std::pair<VelocityMap::iterator, bool> result =
        velocityTargets.insert(VelocityMap::value_type(frameIndex, frameConstraint));

    problemInitialized = false;
    return result.second;
}

void InverseVelocityKinematics::impl::updateTargetLinearVelocity(
    const VelocityMap::iterator& target,
    const iDynTree::Vector3& newLinearVelocity,
    const double newLinearVelocityWeight)
{
    target->second.setLinearVelocity(newLinearVelocity);
    target->second.setLinearVelocityWeight(newLinearVelocityWeight);
}

void InverseVelocityKinematics::impl::updateTargetAngularVelocity(
    const VelocityMap::iterator& target,
    const iDynTree::Vector3& newAngularVelocity,
    const double newAngularVelocityWeight)
{
    target->second.setAngularVelocity(newAngularVelocity);
    target->second.setAngularVelocityWeight(newAngularVelocityWeight);
}

InverseVelocityKinematics::impl::VelocityMap::iterator
InverseVelocityKinematics::impl::getTargetRefIfItExists(const std::string& targetFrameName)
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
    solveInverseDifferentialKinematics(
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

    return true;
}

bool InverseVelocityKinematics::impl::solveInverseDifferentialKinematics(
    const iDynTree::MatrixDynSize& matrix,
    const iDynTree::VectorDynSize& inputVector,
    iDynTree::VectorDynSize& outputVector,
    const iDynTree::VectorDynSize& weightVector,
    const iDynTree::MatrixDynSize& regularizationMatrix)
{
    if (inputVector.size() != matrix.rows() || matrix.cols() != regularizationMatrix.rows()) {
        return false;
    }

    outputVector.resize(matrix.cols());

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> weightInverse(weightVector.size());
    weightInverse =
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(iDynTree::toEigen(weightVector)).inverse();

    if (resolutionMode == ResolutionMode::QP) {

        if (!m_optimizerSolver->isInitialized()) {

            if (!m_optimizerSolver->initSolver()) {
                yError() << "[InverseVelocityKinematics::impl::solveInverseDifferentialKinematics] "
                         << "qp solver for [initSolver] returns false.";

                return 1;
            }
        }
        else {
            // update matrix A' for the jacobian changes
            iDynTree::toEigen(m_A_prime).topLeftCorner(numberOfTargetVariables, configSpaceSize) =
                iDynTree::toEigen(matrix).sparseView();

            // update upper/lower buondary vector because of changes in target velocity vector
            iDynTree::toEigen(m_u_prime).topRows(numberOfTargetVariables) =
                iDynTree::toEigen(inputVector);
            iDynTree::toEigen(m_l_prime).topRows(numberOfTargetVariables) =
                iDynTree::toEigen(inputVector);

            // update upper/lower boundary vector for configuration space constraints
            // previous nu results: jointVelocityResult
            for (unsigned i = 0; i < configSpaceNumberOfConstraints; i++) {

                // check issue #132
                // get the vector 'b' to compute tanh function
                Eigen::MatrixXd tmp_mat =
                    iDynTree::toEigen(m_A_prime)
                        .block(numberOfTargetVariables + totalNumberOfConstraints
                                   - configSpaceNumberOfConstraints + i,
                               baseDofs,
                               1,
                               dofs)
                        .row(0);
                Eigen::VectorXd tmp_configConstraint_vector(
                    Eigen::Map<Eigen::VectorXd>(tmp_mat.data(), tmp_mat.cols() * tmp_mat.rows()));

                double tmp_bXq =
                    iDynTree::toEigen(state.s).dot(tmp_configConstraint_vector);

                double tmp_upperBound =
                    m_JointConfigSpaceConstraintsMax(i)
                    * std::tanh(m_k_u * (m_custom_ConstraintUpperBound(i) - tmp_bXq));

                double tmp_lowerBound =
                    m_JointConfigSpaceConstraintsMin(i)
                    * std::tanh(m_k_l * (tmp_bXq - m_custom_ConstraintLowerBound(i)));

                // last |configSpaceNumberOfConstraints| rows are related to config space limits
                m_u_prime.setVal(numberOfTargetVariables + totalNumberOfConstraints
                                     - configSpaceNumberOfConstraints + i,
                                 tmp_upperBound);
                m_l_prime.setVal(numberOfTargetVariables + totalNumberOfConstraints
                                     - configSpaceNumberOfConstraints + i,
                                 tmp_lowerBound);
            }

            // prepare constraint matrix, upper/lower bound to set it in qp optimizer solver
            Eigen::SparseMatrix<double> constraintMatrix =
                iDynTree::toEigen(m_A_prime).sparseView();

            auto upperBuond = iDynTree::toEigen(m_u_prime);
            auto lowerBuond = iDynTree::toEigen(m_l_prime);

            if (!m_optimizerSolver->updateBounds(lowerBuond, upperBuond)) {
                yError() << "[InverseVelocityKinematics::impl::solveInverseDifferentialKinematics] "
                         << "qp solver for [updateBounds] returns false.";
                return 1;
            }

            if (!m_optimizerSolver->updateLinearConstraintsMatrix(constraintMatrix)) {
                yError() << "[InverseVelocityKinematics::impl::solveInverseDifferentialKinematics] "
                         << "qp solver for [updateLinearConstraintsMatrix] returns false.";
                return 1;
            }
        }

        // solve the QP problem
        if (!m_optimizerSolver->solve()) {
            yError() << "[InverseVelocityKinematics::impl::solveInverseDifferentialKinematics] "
                     << "qp solver for [solve] returns false.";

            return 1;
        }

        iDynTree::toEigen(outputVector) = m_optimizerSolver->getSolution().topRows(configSpaceSize);
        //        exit(0);
    }
    else if (resolutionMode == ResolutionMode::moorePenrose) {
        iDynTree::toEigen(outputVector) =
            (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                 * iDynTree::toEigen(matrix)
             + iDynTree::toEigen(regularizationMatrix))
                .inverse()
            * iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
            * iDynTree::toEigen(inputVector);
    }
    else if (resolutionMode
             == ResolutionMode::completeOrthogonalDecomposition) {
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
    else if (resolutionMode == ResolutionMode::leastSquare) {
        iDynTree::toEigen(outputVector) =
            (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                 * iDynTree::toEigen(matrix)
             + iDynTree::toEigen(regularizationMatrix))
                .bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
                .solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                       * iDynTree::toEigen(inputVector));
    }
    else if (resolutionMode == ResolutionMode::choleskyDecomposition) {
        iDynTree::toEigen(outputVector) =
            (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                 * iDynTree::toEigen(matrix)
             + iDynTree::toEigen(regularizationMatrix))
                .llt()
                .solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                       * iDynTree::toEigen(inputVector));
    }
    else if (resolutionMode
             == ResolutionMode::sparseCholeskyDecomposition) {
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
        solver.compute((iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                            * iDynTree::toEigen(matrix)
                        + iDynTree::toEigen(regularizationMatrix))
                           .sparseView());

        iDynTree::toEigen(outputVector) =
            solver.solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                         * iDynTree::toEigen(inputVector));
    }
    else if (resolutionMode
             == ResolutionMode::robustCholeskyDecomposition) {
        iDynTree::toEigen(outputVector) =
            (iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                 * iDynTree::toEigen(matrix)
             + iDynTree::toEigen(regularizationMatrix))
                .ldlt()
                .solve(iDynTree::toEigen(matrix).transpose() * weightInverse.toDenseMatrix()
                       * iDynTree::toEigen(inputVector));
    }
    else if (resolutionMode
             == ResolutionMode::sparseRobustCholeskyDecomposition) {
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
    yInfo() << "********************** prepareOptimizer ***********************";
    baseDofs = 0;
    configSpaceSize = 0;
    totalNumberOfConstraints = 0;
    configSpaceNumberOfConstraints = 0;

    // For more information look at this issue #123,132:
    // https://github.com/robotology/human-dynamics-estimation/issues/123
    // https://github.com/robotology/human-dynamics-estimation/issues/132
    if (resolutionMode == ResolutionMode::QP) {
        baseDofs = 6; // because it is a 6 dof floating base
        configSpaceSize = dofs + baseDofs;
        //********************************************//
        //****** SET THE CONSTRAINTS *****************//
        //********************************************//
        // set customized joint velocity limit
        if (m_generalJointVelocityLimit > 0.0) {
            totalNumberOfConstraints = dofs;

            // general joint velocity constraints
            m_u.resize(totalNumberOfConstraints);
            m_l.resize(totalNumberOfConstraints);
            for (size_t i = 0; i < m_u.size(); i++) {
                m_u.setVal(i, m_generalJointVelocityLimit);
                m_l.setVal(i, -1.0 * m_generalJointVelocityLimit);
            }

            m_A = iDynTree::MatrixDynSize(totalNumberOfConstraints, configSpaceSize);
            iDynTree::toEigen(m_A).rightCols(dofs).setIdentity();
        }

        // customized joint velocity constraint
        if (m_generalJointVelocityLimit > 0.0) {
            if (m_customJointsVelocityLimitsIndexes.size() > 0) {
                for (size_t i = 0; i < m_customJointsVelocityLimitsIndexes.size(); i++) {
                    m_u.setVal(m_customJointsVelocityLimitsIndexes[i],
                               m_custom_jointsVelocityLimitsUpperBound.getVal(i));
                    m_l.setVal(m_customJointsVelocityLimitsIndexes[i],
                               m_custom_jointsVelocityLimitsLowerBound.getVal(i));
                }
            }
        }
        else {
            if (m_customJointsVelocityLimitsIndexes.size() > 0) {
                totalNumberOfConstraints = m_customJointsVelocityLimitsIndexes.size();
                m_A = iDynTree::MatrixDynSize(totalNumberOfConstraints, configSpaceSize);
                m_u.resize(totalNumberOfConstraints);
                m_l.resize(totalNumberOfConstraints);

                for (size_t i = 0; i < m_customJointsVelocityLimitsIndexes.size(); i++) {
                    m_u.setVal(i, m_custom_jointsVelocityLimitsUpperBound.getVal(i));
                    m_l.setVal(i, m_custom_jointsVelocityLimitsLowerBound.getVal(i));

                    m_A.setVal(i, m_customJointsVelocityLimitsIndexes[i] + baseDofs, 1.0);
                }
            }
        }
        // customized base velocity limit
        if (m_custom_baseVelocityUpperLimit.size() == 6) {
            totalNumberOfConstraints += baseDofs;
            iDynTree::VectorDynSize tmp_m_l, tmp_m_u;
            tmp_m_l = m_l;
            tmp_m_u = m_u;
            m_l.resize(totalNumberOfConstraints);
            m_u.resize(totalNumberOfConstraints);
            m_l.zero();
            m_u.zero();

            iDynTree::toEigen(m_u).topRows(baseDofs) =
                iDynTree::toEigen(m_custom_baseVelocityUpperLimit);
            iDynTree::toEigen(m_l).topRows(baseDofs) =
                iDynTree::toEigen(m_custom_baseVelocityLowerLimit);

            iDynTree::toEigen(m_u).bottomRows(totalNumberOfConstraints - baseDofs) =
                iDynTree::toEigen(tmp_m_u);
            iDynTree::toEigen(m_l).bottomRows(totalNumberOfConstraints - baseDofs) =
                iDynTree::toEigen(tmp_m_l);

            iDynTree::MatrixDynSize tmp_m_A = m_A;
            m_A.resize(totalNumberOfConstraints, configSpaceSize);
            m_A.zero();
            iDynTree::toEigen(m_A).topLeftCorner(baseDofs, baseDofs).setIdentity();
            iDynTree::toEigen(m_A).bottomRows(totalNumberOfConstraints - baseDofs) =
                iDynTree::toEigen(tmp_m_A);
        }
        // *********************************************************************//
        // ********** inequality constraint for joint limits values ********** //
        // *********************************************************************//
        unsigned general_JointLimitSize = m_jointsUpperLimits.size();
        unsigned customized_JointLimitSize = m_custom_ConstraintMatrix.rows();

        configSpaceNumberOfConstraints += general_JointLimitSize;
        configSpaceNumberOfConstraints += customized_JointLimitSize;
        totalNumberOfConstraints += configSpaceNumberOfConstraints;

        //  general inequality constraint for joint limits values
        /* NOTICE:
         * Since the customized joints constraints is implemented, we append the customized
         * constraints with the general limits, and the rest of the code remains the same
         */

        //         add the joints limit values to the upper and lower bounds vector
        if (general_JointLimitSize > 0) {
            iDynTree::VectorDynSize tmp_custom_ConstraintUpperBound,
                tmp_custom_ConstraintLowerBound;
            tmp_custom_ConstraintUpperBound = m_custom_ConstraintUpperBound;
            tmp_custom_ConstraintLowerBound = m_custom_ConstraintLowerBound;

            m_custom_ConstraintUpperBound.resize(configSpaceNumberOfConstraints);
            m_custom_ConstraintLowerBound.resize(configSpaceNumberOfConstraints);

            m_custom_ConstraintUpperBound.zero();
            m_custom_ConstraintLowerBound.zero();

            iDynTree::toEigen(m_custom_ConstraintUpperBound).topRows(customized_JointLimitSize) =
                iDynTree::toEigen(tmp_custom_ConstraintUpperBound);
            iDynTree::toEigen(m_custom_ConstraintUpperBound).bottomRows(general_JointLimitSize) =
                iDynTree::toEigen(m_jointsUpperLimits);
            iDynTree::toEigen(m_custom_ConstraintLowerBound).topRows(customized_JointLimitSize) =
                iDynTree::toEigen(tmp_custom_ConstraintLowerBound);
            iDynTree::toEigen(m_custom_ConstraintLowerBound).bottomRows(general_JointLimitSize) =
                iDynTree::toEigen(m_jointsLowerLimits);
        }

        // customized inequality constraint for joint values
        if (configSpaceNumberOfConstraints > 0) {

            iDynTree::VectorDynSize tmp_m_l, tmp_m_u;
            tmp_m_l = m_l;
            tmp_m_u = m_u;
            m_l.resize(totalNumberOfConstraints);
            m_u.resize(totalNumberOfConstraints);
            m_l.zero();
            m_u.zero();
            iDynTree::toEigen(m_u).topRows(totalNumberOfConstraints
                                           - configSpaceNumberOfConstraints) =
                iDynTree::toEigen(tmp_m_u);
            iDynTree::toEigen(m_l).topRows(totalNumberOfConstraints
                                           - configSpaceNumberOfConstraints) =
                iDynTree::toEigen(tmp_m_l);
        }

        m_JointConfigSpaceConstraintsMax.resize(configSpaceNumberOfConstraints);
        m_JointConfigSpaceConstraintsMin.resize(configSpaceNumberOfConstraints);
        m_JointConfigSpaceConstraintsMax.zero();
        m_JointConfigSpaceConstraintsMin.zero();

        iDynTree::MatrixDynSize tmp_m_A = m_A;
        m_A.resize(totalNumberOfConstraints, configSpaceSize);
        m_A.zero();
        iDynTree::toEigen(m_A).topRows(totalNumberOfConstraints - configSpaceNumberOfConstraints) =
            iDynTree::toEigen(tmp_m_A);
        iDynTree::MatrixDynSize jointValuesConstraintsMatrix(configSpaceNumberOfConstraints,
                                                             configSpaceSize);
        jointValuesConstraintsMatrix.zero();

        if (customized_JointLimitSize > 0) {
            for (unsigned i = 0; i < m_custom_ConstraintMatrix.rows(); i++) {
                for (unsigned j = 0; j < m_custom_ConstraintMatrix.cols(); j++) {

                    jointValuesConstraintsMatrix.setVal(
                        i,
                        m_custom_ConstraintVariablesIndex[j] + baseDofs,
                        m_custom_ConstraintMatrix.getVal(
                            i,
                            j)); // +baseDofs: because of the floating base
                }
            }
        }

        // add the joints limits identity matrix to the constraints matrix
        if (general_JointLimitSize > 0) {
            iDynTree::toEigen(jointValuesConstraintsMatrix)
                .bottomRightCorner(general_JointLimitSize, general_JointLimitSize)
                .setIdentity();
        }

        // set the maximum velocity for the config space constraints of the joints

        /* * jointValuesConstraintsMatrix size: (number of constraints) X (base DOF + # all the
         *   joints DOFs)
         * * This matrix represents all the limits on the joint values (not the
         *   velocities!)
         * * To apply this part, all the joints should have a velocity limit
         * * The first 'baseDofs' columns of the matrix is the related the base constraints,
         *   since we do not have any constraints on them, they are zero, iterating on
         *   it starts from baseDofs
         * * m_u starts with the joint velicity limits, therefore iterating on m_u starts from
         *   '0'
         */
        if (configSpaceNumberOfConstraints > 0) {
            for (unsigned i = 0; i < jointValuesConstraintsMatrix.rows(); i++) {
                for (unsigned j = baseDofs; j < jointValuesConstraintsMatrix.cols(); j++) {

                    m_JointConfigSpaceConstraintsMax(i) +=
                        std::abs(jointValuesConstraintsMatrix.getVal(i, j)
                                 * m_u((j - baseDofs) + m_custom_baseVelocityUpperLimit.size()));
                }
                m_JointConfigSpaceConstraintsMin(i) = -1.0 * m_JointConfigSpaceConstraintsMax(i);
            }

            iDynTree::toEigen(m_A).bottomRows(configSpaceNumberOfConstraints) =
                iDynTree::toEigen(jointValuesConstraintsMatrix);
        }
        coefVectorCustomConstraints.resize(1, dofs);
        //*******************************************//
        //****** GENERATE QP MATRICES ***************//
        //*******************************************//
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

        // generate A_prime matrix
        m_A_prime = iDynTree::MatrixDynSize(totalNumberOfConstraints + numberOfTargetVariables,
                                            configSpaceSize + numberOfTargetVariables);
        iDynTree::toEigen(m_A_prime).topLeftCorner(numberOfTargetVariables, configSpaceSize) =
            Eigen::MatrixXd::Zero(numberOfTargetVariables, configSpaceSize);
        iDynTree::toEigen(m_A_prime)
            .topRightCorner(numberOfTargetVariables, numberOfTargetVariables)
            .setIdentity();
        iDynTree::toEigen(m_A_prime).bottomLeftCorner(totalNumberOfConstraints, configSpaceSize) =
            iDynTree::toEigen(m_A);
        iDynTree::toEigen(m_A_prime).bottomRightCorner(totalNumberOfConstraints,
                                                       numberOfTargetVariables) =
            Eigen::MatrixXd::Zero(totalNumberOfConstraints, numberOfTargetVariables);

        // generate upper limit vector (u_prime) and lower limit vector (l_prime)
        m_l_prime = iDynTree::VectorDynSize(totalNumberOfConstraints + numberOfTargetVariables);
        m_u_prime = iDynTree::VectorDynSize(totalNumberOfConstraints + numberOfTargetVariables);
        iDynTree::toEigen(m_u_prime).topRows(numberOfTargetVariables) =
            Eigen::VectorXd::Zero(numberOfTargetVariables, 1);
        iDynTree::toEigen(m_u_prime).bottomRows(totalNumberOfConstraints) = iDynTree::toEigen(m_u);
        iDynTree::toEigen(m_l_prime).topRows(numberOfTargetVariables) =
            Eigen::VectorXd::Zero(numberOfTargetVariables, 1);
        iDynTree::toEigen(m_l_prime).bottomRows(totalNumberOfConstraints) = iDynTree::toEigen(m_l);
        m_optimizerSolver = std::make_unique<OsqpEigen::Solver>();

        m_optimizerSolver->settings()->setWarmStart(true);
        m_optimizerSolver->settings()->setVerbosity(false);
        m_optimizerSolver->data()->setNumberOfVariables(numberOfTargetVariables + configSpaceSize);
        m_optimizerSolver->data()->setNumberOfConstraints(totalNumberOfConstraints
                                                          + numberOfTargetVariables);

        // generate q_prime matrix
        m_q_prime = iDynTree::VectorDynSize(numberOfTargetVariables + configSpaceSize);
        iDynTree::toEigen(m_q_prime).setZero();

        //*******************************************//
        //****** SET QP MATRICES ********************//
        //*******************************************//
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
        yInfo() << "===========================================";
        yInfo() << "QP initialization is done, more info:";
        yInfo() << "constraint Matrix size:" << constraintMatrix.rows() << " , "
                << constraintMatrix.cols();
        yInfo() << "upper bound vector size:" << m_u_prime.size();
        yInfo() << "lower bound vector size:" << m_l_prime.size();
        yInfo() << "Hessian Matrix size:" << HessianMatrix.rows() << " , " << HessianMatrix.cols();
        yInfo() << "Gradient Vector size:" << GradientVector.size();
        yInfo() << "===========================================";
        //        yInfo() << "constraintMatrix: " << m_A_prime.toString();
        //        yInfo() << "upper bound vector size:" << m_u_prime.toString();
        //        yInfo() << "lower bound vector size:" << m_l_prime.toString();
    }
}

InverseVelocityKinematics::impl::VelocityConstraint::VelocityConstraint(
    const std::string& frameName,
    const VelocityConstraintType& type)
    : type(type)
    , frameName(frameName)
    , linearVelocityWeight(1.0)
    , angularVelocityWeight(1.0)
{}

InverseVelocityKinematics::impl::VelocityConstraint
InverseVelocityKinematics::impl::VelocityConstraint::linearVelocityConstraint(
    const std::string& frameName,
    const iDynTree::Vector3& linearVelocity,
    const double linearVelocityWeight)
{
    VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeLinearVelocity);
    velocityConstraint.setLinearVelocity(linearVelocity);
    velocityConstraint.setLinearVelocityWeight(linearVelocityWeight);
    return velocityConstraint;
}

InverseVelocityKinematics::impl::VelocityConstraint
InverseVelocityKinematics::impl::VelocityConstraint::angularVelocityConstraint(
    const std::string& frameName,
    const iDynTree::Vector3& angularVelocity,
    const double angularVelocityWeight)
{
    VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeAngularVelocity);
    velocityConstraint.setLinearVelocity(angularVelocity);
    velocityConstraint.setLinearVelocityWeight(angularVelocityWeight);
    return velocityConstraint;
}

InverseVelocityKinematics::impl::VelocityConstraint
InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(
    const std::string& frameName,
    const iDynTree::Vector3& linearVelocity,
    const iDynTree::Vector3& angularVelocity,
    const double linearVelocityWeight,
    const double angularVelocityWeight)
{
    iDynTree::Twist twist(linearVelocity, angularVelocity);
    return TwistConstraint(frameName, twist, linearVelocityWeight, angularVelocityWeight);
}

InverseVelocityKinematics::impl::VelocityConstraint
InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(
    const std::string& frameName,
    const iDynTree::Twist& twist,
    const double linearVelocityWeight,
    const double angularVelocityWeight)
{
    VelocityConstraint velocityConstraint(frameName, VelocityConstraintTypeTwist);
    velocityConstraint.setTwist(twist);
    velocityConstraint.setLinearVelocityWeight(linearVelocityWeight);
    velocityConstraint.setAngularVelocityWeight(angularVelocityWeight);
    return velocityConstraint;
}

InverseVelocityKinematics::impl::VelocityConstraint::VelocityConstraintType
InverseVelocityKinematics::impl::VelocityConstraint::getType() const
{
    return type;
}

std::string InverseVelocityKinematics::impl::VelocityConstraint::getFrameName() const
{
    return frameName;
}

bool InverseVelocityKinematics::impl::VelocityConstraint::hasLinearVelocityConstraint() const
{
    return (type == VelocityConstraintTypeLinearVelocity) || (type == VelocityConstraintTypeTwist);
}

bool InverseVelocityKinematics::impl::VelocityConstraint::hasAngularVelocityConstraint() const
{
    return (type == VelocityConstraintTypeAngularVelocity) || (type == VelocityConstraintTypeTwist);
}

iDynTree::Vector3 InverseVelocityKinematics::impl::VelocityConstraint::getLinearVelocity() const
{
    return twist.getLinearVec3();
}

void InverseVelocityKinematics::impl::VelocityConstraint::setLinearVelocity(
    const iDynTree::Vector3& newLinearVelocity)
{
    twist.setLinearVec3(newLinearVelocity);
}

iDynTree::Vector3 InverseVelocityKinematics::impl::VelocityConstraint::getAngularVelocity() const
{
    return twist.getAngularVec3();
}

void InverseVelocityKinematics::impl::VelocityConstraint::setAngularVelocity(
    const iDynTree::Vector3& newAngularVelocity)
{
    twist.setAngularVec3(newAngularVelocity);
}

iDynTree::Twist InverseVelocityKinematics::impl::VelocityConstraint::getTwist() const
{
    return twist;
}

void InverseVelocityKinematics::impl::VelocityConstraint::setTwist(const iDynTree::Twist& newTwist)
{
    twist = newTwist;
}

double InverseVelocityKinematics::impl::VelocityConstraint::getLinearVelocityWeight() const
{
    return linearVelocityWeight;
}

void InverseVelocityKinematics::impl::VelocityConstraint::setLinearVelocityWeight(
    const double newLinearVelocityWeight)
{
    linearVelocityWeight = newLinearVelocityWeight;
}

double InverseVelocityKinematics::impl::VelocityConstraint::getAngularVelocityWeight() const
{
    return angularVelocityWeight;
}

void InverseVelocityKinematics::impl::VelocityConstraint::setAngularVelocityWeight(
    const double newAngularVelocityWeight)
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

bool InverseVelocityKinematics::setModel(const iDynTree::Model& model)
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

bool InverseVelocityKinematics::setFloatingBaseOnFrameNamed(
    const std::string& floatingBaseFrameName)
{
    return pImpl->dynamics.setFloatingBase(floatingBaseFrameName);
}

bool InverseVelocityKinematics::setResolutionMode(
    const ResolutionMode& resolutionMode)
{
    pImpl->resolutionMode = resolutionMode;
    return true;
}

bool InverseVelocityKinematics::setResolutionMode(const std::string& resolutionModeName)
{
    if (resolutionModeName == "QP") {
        pImpl->resolutionMode = ResolutionMode::QP;
    }
    else if (resolutionModeName == "moorePenrose") {
        pImpl->resolutionMode = ResolutionMode::moorePenrose;
    }
    else if (resolutionModeName == "completeOrthogonalDecomposition") {
        pImpl->resolutionMode =
            ResolutionMode::completeOrthogonalDecomposition;
    }
    else if (resolutionModeName == "leastSquare") {
        pImpl->resolutionMode = ResolutionMode::leastSquare;
    }
    else if (resolutionModeName == "choleskyDecomposition") {
        pImpl->resolutionMode = ResolutionMode::choleskyDecomposition;
    }
    else if (resolutionModeName == "sparseCholeskyDecomposition") {
        pImpl->resolutionMode =
            ResolutionMode::sparseCholeskyDecomposition;
    }
    else if (resolutionModeName == "robustCholeskyDecomposition") {
        pImpl->resolutionMode =
            ResolutionMode::robustCholeskyDecomposition;
    }
    else if (resolutionModeName == "sparseRobustCholeskyDecomposition") {
        pImpl->resolutionMode =
            ResolutionMode::sparseRobustCholeskyDecomposition;
    }
    else {
        std::cerr << "[ERROR] Invalid resolution mode: " << resolutionModeName << std::endl;
        return false;
    }

    return true;
}

void InverseVelocityKinematics::setRegularization(const double regularizationWeight)
{
    pImpl->regularizationWeight = regularizationWeight;
}

bool InverseVelocityKinematics::addTarget(const std::string& linkName,
                                          const iDynTree::Vector3& linearVelocity,
                                          const iDynTree::Vector3& angularVelocity,
                                          const double linearWeight,
                                          const double angularWeight)
{
    return pImpl->addTarget(InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(
        linkName, linearVelocity, angularVelocity, linearWeight, angularWeight));
}

bool InverseVelocityKinematics::addTarget(const std::string& linkName,
                                          const iDynTree::Twist& twist,
                                          const double linearWeight,
                                          const double angularWeight)
{
    return pImpl->addTarget(InverseVelocityKinematics::impl::VelocityConstraint::TwistConstraint(
        linkName, twist, linearWeight, angularWeight));
}

bool InverseVelocityKinematics::addLinearVelocityTarget(const std::string& linkName,
                                                        const iDynTree::Vector3& linearVelocity,
                                                        const double linearWeight)
{
    return pImpl->addTarget(
        InverseVelocityKinematics::impl::VelocityConstraint::linearVelocityConstraint(
            linkName, linearVelocity, linearWeight));
}

bool InverseVelocityKinematics::addLinearVelocityTarget(const std::string& linkName,
                                                        const iDynTree::Twist& twist,
                                                        const double linearWeight)
{
    return pImpl->addTarget(
        InverseVelocityKinematics::impl::VelocityConstraint::linearVelocityConstraint(
            linkName, twist.getLinearVec3(), linearWeight));
}

bool InverseVelocityKinematics::addAngularVelocityTarget(const std::string& linkName,
                                                         const iDynTree::Vector3& angularVelocity,
                                                         const double angularWeight)
{
    return pImpl->addTarget(
        InverseVelocityKinematics::impl::VelocityConstraint::angularVelocityConstraint(
            linkName, angularVelocity, angularWeight));
}

bool InverseVelocityKinematics::addAngularVelocityTarget(const std::string& linkName,
                                                         const iDynTree::Twist& twist,
                                                         const double angularWeight)
{
    return pImpl->addTarget(
        InverseVelocityKinematics::impl::VelocityConstraint::angularVelocityConstraint(
            linkName, twist.getAngularVec3(), angularWeight));
}

bool InverseVelocityKinematics::setJointConfiguration(const std::string& jointName,
                                                      const double jointConfiguration)
{
    iDynTree::JointIndex jointIndex = pImpl->dynamics.model().getJointIndex(jointName);
    if (jointIndex == iDynTree::JOINT_INVALID_INDEX)
        return false;
    pImpl->state.s(jointIndex) = jointConfiguration;
    pImpl->updateConfiguration();
    return true;
}

bool InverseVelocityKinematics::setJointsConfiguration(
    const iDynTree::VectorDynSize& jointsConfiguration)
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

bool InverseVelocityKinematics::setBasePose(const iDynTree::Transform& baseTransform)
{
    pImpl->state.W_p_B = baseTransform.getPosition();
    pImpl->state.W_R_B = baseTransform.getRotation();
    pImpl->updateConfiguration();
    return true;
}

bool InverseVelocityKinematics::setBasePose(const iDynTree::Vector3& basePosition,
                                            const iDynTree::Rotation& baseRotation)
{
    pImpl->state.W_p_B = basePosition;
    pImpl->state.W_R_B = baseRotation;
    pImpl->updateConfiguration();
    return true;
}

bool InverseVelocityKinematics::setConfiguration(const iDynTree::Transform& baseTransform,
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

bool InverseVelocityKinematics::setConfiguration(const iDynTree::Vector3& basePosition,
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

bool InverseVelocityKinematics::updateTarget(const std::string& linkName,
                                             const iDynTree::Vector3& linearVelocity,
                                             const iDynTree::Vector3& angularVelocity,
                                             const double linearWeight,
                                             const double angularWeight)
{
    InverseVelocityKinematics::impl::VelocityMap::iterator targetConstr =
        pImpl->getTargetRefIfItExists(linkName);

    if (targetConstr == pImpl->velocityTargets.end()) {
        std::stringstream ss;
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateTargetLinearVelocity(targetConstr, linearVelocity, linearWeight);
    pImpl->updateTargetAngularVelocity(targetConstr, angularVelocity, angularWeight);
    return true;
}

bool InverseVelocityKinematics::updateTarget(const std::string& linkName,
                                             const iDynTree::Twist& twist,
                                             const double linearWeight,
                                             const double angularWeight)
{
    return updateTarget(
        linkName, twist.getLinearVec3(), twist.getAngularVec3(), linearWeight, angularWeight);
}

bool InverseVelocityKinematics::updateTargetLinearVelocity(const std::string& linkName,
                                                           const iDynTree::Vector3& linearVelocity,
                                                           const double linearWeight)
{
    InverseVelocityKinematics::impl::VelocityMap::iterator targetConstr =
        pImpl->getTargetRefIfItExists(linkName);

    if (targetConstr == pImpl->velocityTargets.end()) {
        std::stringstream ss;
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateTargetLinearVelocity(targetConstr, linearVelocity, linearWeight);
    return true;
}

bool InverseVelocityKinematics::updateTargetAngularVelocity(
    const std::string& linkName,
    const iDynTree::Vector3& angularVelocity,
    const double angularWeight)
{
    InverseVelocityKinematics::impl::VelocityMap::iterator targetConstr =
        pImpl->getTargetRefIfItExists(linkName);

    if (targetConstr == pImpl->velocityTargets.end()) {
        std::cerr << "No target for frame " << linkName
                  << " was added to the Inverse Velocity Kinematics problem.";
        return false;
    }

    pImpl->updateTargetAngularVelocity(targetConstr, angularVelocity, angularWeight);
    return true;
}

bool InverseVelocityKinematics::getVelocitySolution(iDynTree::Twist& baseVelocity,
                                                    iDynTree::VectorDynSize& jointsVelocity) const
{
    return getJointsVelocitySolution(jointsVelocity) && getBaseVelocitySolution(baseVelocity);
}

bool InverseVelocityKinematics::getVelocitySolution(iDynTree::Vector3& linearVelocity,
                                                    iDynTree::Vector3& angularVelocity,
                                                    iDynTree::VectorDynSize& jointsVelocity) const
{
    return getJointsVelocitySolution(jointsVelocity) && getBaseVelocitySolution(linearVelocity, angularVelocity);
}

bool InverseVelocityKinematics::getJointsVelocitySolution(
    iDynTree::VectorDynSize& jointsVelocity) const
{
    if (jointsVelocity.size() == pImpl->jointVelocityResult.size()) {
        jointsVelocity = pImpl->jointVelocityResult;
        return true;
    }
    else {
        return false;
    }
}

bool InverseVelocityKinematics::getBaseVelocitySolution(iDynTree::Twist& baseVelocity) const
{
    baseVelocity = pImpl->baseVelocityResult;
    return true;
}

bool InverseVelocityKinematics::getBaseVelocitySolution(iDynTree::Vector3& linearVelocity,
                                                        iDynTree::Vector3& angularVelocity) const
{
    linearVelocity = pImpl->baseVelocityResult.getLinearVec3();
    angularVelocity = pImpl->baseVelocityResult.getAngularVec3();
    return true;
}

bool InverseVelocityKinematics::setCustomBaseVelocityLimit(
    const iDynTree::VectorDynSize& lowerBound,
    const iDynTree::VectorDynSize& upperBound)
{
    pImpl->m_custom_baseVelocityLowerLimit = lowerBound;
    pImpl->m_custom_baseVelocityUpperLimit = upperBound;
    return true;
}

bool InverseVelocityKinematics::setCustomJointsVelocityLimit(
    const std::vector<iDynTree::JointIndex>& jointsIndexList,
    const iDynTree::VectorDynSize& jointsLimitList)
{
    pImpl->m_customJointsVelocityLimitsIndexes = jointsIndexList;
    pImpl->m_custom_jointsVelocityLimitsUpperBound = jointsLimitList;
    pImpl->m_custom_jointsVelocityLimitsLowerBound.resize(jointsLimitList.size());
    for (unsigned i = 0; i < jointsLimitList.size(); i++) {
        pImpl->m_custom_jointsVelocityLimitsLowerBound.setVal(i, -1.0 * jointsLimitList.getVal(i));
    }
    return true;
}

bool InverseVelocityKinematics::setCustomConstraintsJointsValues(
    const std::vector<iDynTree::JointIndex>& jointsIndexList,
    const iDynTree::VectorDynSize& upperBoundary,
    const iDynTree::VectorDynSize& lowerBoundary,
    const iDynTree::MatrixDynSize& customConstraintMatrix)
{
    pImpl->m_custom_ConstraintVariablesIndex = jointsIndexList;
    pImpl->m_custom_ConstraintMatrix = customConstraintMatrix;
    pImpl->m_custom_ConstraintUpperBound = upperBoundary;
    pImpl->m_custom_ConstraintLowerBound = lowerBoundary;

    return true;
}

bool InverseVelocityKinematics::setConstraintParametersJointValues(const double& k_u,
                                                                   const double& k_l)
{
    pImpl->m_k_u = k_u;
    pImpl->m_k_l = k_l;
    return true;
}

bool InverseVelocityKinematics::setGeneralJointVelocityConstraints(const double jointVelocityLimit)
{
    pImpl->m_generalJointVelocityLimit = jointVelocityLimit;
    return true;
}

bool InverseVelocityKinematics::setGeneralJointsUpperLowerConstraints(
    const iDynTree::VectorDynSize& jointUpperLimits,
    const iDynTree::VectorDynSize& jointLowerLimits)
{

    pImpl->m_jointsUpperLimits = jointUpperLimits;
    pImpl->m_jointsLowerLimits = jointLowerLimits;
    return true;
}

bool InverseVelocityKinematics::solve()
{
    return pImpl->solveProblem();
}

void InverseVelocityKinematics::clearProblem()
{
    pImpl->state.initializeState(pImpl->dofs);

    pImpl->baseVelocityResult.zero();

    pImpl->jointVelocityResult.resize(pImpl->dofs);
    pImpl->jointVelocityResult.zero();

    pImpl->velocityTargets.clear();

    pImpl->problemInitialized = false;
}
