// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

    // Buffers
    iDynTree::VectorDynSize m_errorBuffer;
    iDynTree::Vector3 m_desiredLinearVelocityBuffer, m_desiredAngularVelocityBuffer;
    iDynTree::Vector3 m_netMeanRotErr, m_netMeanPosErr;
    double m_rotMeanErrNorm{-1.0}, m_posMeanErrNorm{-1.0};

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

    TargetsMap::iterator getTargetRefIfItExists(const std::string& targetFrameName);

    bool solveProblem(const double dt, bool &reset);

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

    double positionFeedbackGain;
    double orientationFeedbackGain;
    double linearVelocityFeedforwardGain;
    double angularVelocityFeedforwardGain;
    double linearVelocityWeight;
    double angularVelocityWeight;
    std::array<bool, 3> activePositionTargetAxis;
    std::array<bool, 3> activeOrientationTargetAxis;


    int errorSize;

    InverseKinematicsTarget(const std::string& frameName,
                            const TargetType& configuration,
                            const double positionFeedbackGain = 0.0,
                            const double orientationFeedbackGain = 0.0,
                            const double linearVelocityFeedforwardGain = 0.0,
                            const double angularVelocityFeedforwardGain = 0.0,
                            const double linearVelocityWeight = 0.0,
                            const double angularVelocityWeight = 0.0,
                            const bool XPositionTargetAxisActive = true,
                            const bool YPositionTargetAxisActive = true,
                            const bool ZPositionTargetAxisActive = true,
                            const bool XRotationTargetAxisActive = true,
                            const bool YRotationTargetAxisActive = true,
                            const bool ZRotationTargetAxisActive = true);

    static InverseKinematicsTarget PositionAndVelocityTarget(const std::string& frameName,
                                                             const iDynTree::Vector3& position,
                                                             const iDynTree::Vector3& linearVelocity,
                                                             const double positionFeedbackGain = 1.0,
                                                             const double linearVelocityFeedforwardGain = 1.0,
                                                             const double linearVeloicityWeight = 1.0,
                                                             const bool XPositionTargetAxisActive = true,
                                                             const bool YPositionTargetAxisActive = true,
                                                             const bool ZPositionTargetAxisActive = true);
    static InverseKinematicsTarget PositionTarget(const std::string& frameName,
                                                  const iDynTree::Vector3& position,
                                                  const double positionFeedbackGain = 1.0,
                                                  const double linearVeloictyWeight = 1.0,
                                                  const bool XPositionTargetAxisActive = true,
                                                  const bool YPositionTargetAxisActive = true,
                                                  const bool ZPositionTargetAxisActive = true);
    static InverseKinematicsTarget OrientationAndVelocityTarget(const std::string& frameName,
                                                                const iDynTree::Rotation& orientation,
                                                                const iDynTree::Vector3& angularVelocity,
                                                                const double orientationFeedbackGain = 1.0,
                                                                const double angularVelocityFeedforwardGain = 1.0,
                                                                const double angularVelocityWeight = 1.0,
                                                                const bool XRotationTargetAxisActive = true,
                                                                const bool YRotationTargetAxisActive = true,
                                                                const bool ZRotationTargetAxisActive = true);
    static InverseKinematicsTarget OrientationTarget(const std::string& frameName,
                                                     const iDynTree::Rotation& orientation,
                                                     const double orientationFeedbackGain = 1.0,
                                                     const double angularVelocityWeight = 1.0,
                                                     const bool XRotationTargetAxisActive = true,
                                                     const bool YRotationTargetAxisActive = true,
                                                     const bool ZRotationTargetAxisActive = true);
    static InverseKinematicsTarget PoseAndVelocityTarget(const std::string& frameName,
                                                         const iDynTree::Transform& transform,
                                                         const iDynTree::Twist& twist,
                                                         const double positionFeedbackGain = 1.0,
                                                         const double orientationFeedbackGain = 1.0,
                                                         const double linearVelocityFeedforwardGain = 1.0,
                                                         const double angularVelocityFeedforwardGain = 1.0,
                                                         const double linearVelocityWeight = 1.0,
                                                         const double angularVelocityWeight = 1.0,
                                                         const bool XPositionTargetAxisActive = true,
                                                         const bool YPositionTargetAxisActive = true,
                                                         const bool ZPositionTargetAxisActive = true,
                                                         const bool XRotationTargetAxisActive = true,
                                                         const bool YRotationTargetAxisActive = true,
                                                         const bool ZRotationTargetAxisActive = true);
    static InverseKinematicsTarget PoseTarget(const std::string& frameName,
                                              const iDynTree::Transform& transform,
                                              const double positionFeedbackGain = 1.0,
                                              const double orientationFeedbackGain = 1.0,
                                              const double linearVelocityWeight = 1.0,
                                              const double angularVelocityWeight = 1.0,
                                              const bool XPositionTargetAxisActive = true,
                                              const bool YPositionTargetAxisActive = true,
                                              const bool ZPositionTargetAxisActive = true,
                                              const bool XRotationTargetAxisActive = true,
                                              const bool YRotationTargetAxisActive = true,
                                              const bool ZRotationTargetAxisActive = true);
    static InverseKinematicsTarget PoseAndVelocityTarget(const std::string& frameName,
                                                         const iDynTree::Vector3& position,
                                                         const iDynTree::Rotation& orientation,
                                                         const iDynTree::Vector3& linearVelocity,
                                                         const iDynTree::Vector3& angularVelocity,
                                                         const double positionFeedbackGain = 1.0,
                                                         const double orientationFeedbackGain = 1.0,
                                                         const double linearVelocityFeedforwardGain = 1.0,
                                                         const double angularVelocityFeedforwardGain = 1.0,
                                                         const double linearVelocityWeight = 1.0,
                                                         const double angularVelocityWeight = 1.0,
                                                         const bool XPositionTargetAxisActive = true,
                                                         const bool YPositionTargetAxisActive = true,
                                                         const bool ZPositionTargetAxisActive = true,
                                                         const bool XRotationTargetAxisActive = true,
                                                         const bool YRotationTargetAxisActive = true,
                                                         const bool ZRotationTargetAxisActive = true);
    static InverseKinematicsTarget PoseTarget(const std::string& frameName,
                                              const iDynTree::Vector3& position,
                                              const iDynTree::Rotation& orientation,
                                              const double positionFeedbackGain = 1.0,
                                              const double orientationFeedbackGain = 1.0,
                                              const double linearVelocityWeight = 1.0,
                                              const double angularVelocityWeight = 1.0,
                                              const bool XPositionTargetAxisActive = true,
                                              const bool YPositionTargetAxisActive = true,
                                              const bool ZPositionTargetAxisActive = true,
                                              const bool XRotationTargetAxisActive = true,
                                              const bool YRotationTargetAxisActive = true,
                                              const bool ZRotationTargetAxisActive = true);


    void setTargetType(const TargetType& targetType);
    TargetType getTargetType() const;
    std::string getFrameName() const;

    bool hasPositionTarget() const;
    bool hasOrientationTarget() const;

    iDynTree::VectorDynSize errorBuffer;
    int getErrorSize() const;
    bool computeError(const iDynTree::Transform& transform, iDynTree::VectorDynSize& error);
    bool computeError(const iDynTree::Vector3& position, iDynTree::VectorDynSize& error);
    bool computeError(const iDynTree::Rotation& orientation, iDynTree::VectorDynSize& error);

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

    bool setActiveOrientationTargetAxis(const bool isXActive, const bool isYActive, const bool isZActive);
    std::array<bool, 3> getActivePositionTargetAxes() const;
    std::array<bool, 3> getActiveOrientationTargetAxes() const;

    double getPositionFeedbackGain() const;
    void setPositionFeedbackGain(const double& newPositionFeedbackGain);
    double getOrientationFeedbackGain() const;
    void setOrientationFeedbackGain(const double& newOrientationFeedbackGain);
    double getLinearVelocityFeedforwardGain() const;
    void setLinearVelocityFeedforwardGain(const double& newLinearVelocityFeedforwardGain);
    double getAngularVelocityFeedforwardGain() const;
    void setAngularVelocityFeedforwardGain(const double& newAngularVelocityFeedforwardGain);
    double getLinearVelocityWeight() const;
    void setLinearVelocityWeight(const double& newLinearVelocityWeight);
    double getAngularVelocityWeight() const;
    void setAngularVelocityWeight(const double& newAngularVelocityWeight);
};


DynamicalInverseKinematics::impl::InverseKinematicsTarget::InverseKinematicsTarget(
    const std::string& frameName,
    const TargetType& type,
    const double positionFeedbackGain_,
    const double orientationFeedbackGain_,
    const double linearVelocityFeedforwardGain_,
    const double angularVelocityFeedforwardGain_,
    const double linearVelocityWeight_,
    const double angularVelocityWeight_,
    const bool XPositionTargetAxisActive_,
    const bool YPositionTargetAxisActive_,
    const bool ZPositionTargetAxisActive_,
    const bool XRotationTargetAxisActive_,
    const bool YRotationTargetAxisActive_,
    const bool ZRotationTargetAxisActive_)
    : frameName(frameName)
    , positionFeedbackGain(positionFeedbackGain_)
    , orientationFeedbackGain(orientationFeedbackGain_)
    , linearVelocityFeedforwardGain(linearVelocityFeedforwardGain_)
    , angularVelocityFeedforwardGain(angularVelocityFeedforwardGain_)
    , linearVelocityWeight(linearVelocityWeight_)
    , angularVelocityWeight(angularVelocityWeight_)
    , activePositionTargetAxis({XPositionTargetAxisActive_, YPositionTargetAxisActive_, ZPositionTargetAxisActive_})
    , activeOrientationTargetAxis({XRotationTargetAxisActive_, YRotationTargetAxisActive_, ZRotationTargetAxisActive_})
{
    setTargetType(type);
    setTransform(iDynTree::Transform::Identity());
    setTwist(iDynTree::Twist::Zero());
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PositionAndVelocityTarget(
    const std::string& frameName,
    const iDynTree::Vector3& position,
    const iDynTree::Vector3& linearVelocity,
    const double positionFeedbackGain_,
    const double linearVelocityFeedforwardGain_,
    const double linearVelocityWeight_,
    const bool XPositionTargetAxisActive_,
    const bool YPositionTargetAxisActive_,
    const bool ZPositionTargetAxisActive_)
{
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePosition,
                                                    positionFeedbackGain_, 0.0,
                                                    linearVelocityFeedforwardGain_, 0.0,
                                                    linearVelocityWeight_, 0.0,
                                                    XPositionTargetAxisActive_, YPositionTargetAxisActive_, ZPositionTargetAxisActive_,
                                                    false, false, false);
    inverseKinematicsTarget.setPosition(position);
    inverseKinematicsTarget.setLinearVelocity(linearVelocity);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PositionTarget(
    const std::string& frameName,
    const iDynTree::Vector3& position,
    const double positionFeedbackGain_,
    const double linearVelocityWeight_,
    const bool XPositionTargetAxisActive_,
    const bool YPositionTargetAxisActive_,
    const bool ZPositionTargetAxisActive_)
{
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePosition,
                                                    positionFeedbackGain_, 0.0,
                                                    0.0, 0.0,
                                                    linearVelocityWeight_, 0.0,
                                                    XPositionTargetAxisActive_, XPositionTargetAxisActive_, XPositionTargetAxisActive_,
                                                    false, false, false);
    inverseKinematicsTarget.setPosition(position);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::OrientationAndVelocityTarget(
    const std::string& frameName,
    const iDynTree::Rotation& orientation,
    const iDynTree::Vector3& angularVelocity,
    const double orientationFeedbackGain_,
    const double angularVelocityFeedforwardGain_,
    const double angularVelocityWeight_,
    const bool XRotationTargetAxisActive_,
    const bool YRotationTargetAxisActive_,
    const bool ZRotationTargetAxisActive_)
{
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypeOrientation,
                                                    0.0, orientationFeedbackGain_,
                                                    0.0, angularVelocityFeedforwardGain_,
                                                    0.0, angularVelocityWeight_,
                                                    false, false, false,
                                                    XRotationTargetAxisActive_, YRotationTargetAxisActive_, ZRotationTargetAxisActive_);
    inverseKinematicsTarget.setOrientation(orientation);
    inverseKinematicsTarget.setAngularVelocity(angularVelocity);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::OrientationTarget(
    const std::string& frameName,
    const iDynTree::Rotation& orientation,
    const double orientationFeedbackGain_,
    const double angularVelocityWeight_,
    const bool XRotationTargetAxisActive_,
    const bool YRotationTargetAxisActive_,
    const bool ZRotationTargetAxisActive_)
{
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypeOrientation,
                                                    0.0, orientationFeedbackGain_,
                                                    0.0, 0.0,
                                                    0.0, angularVelocityWeight_,
                                                    false, false, false,
                                                    XRotationTargetAxisActive_, YRotationTargetAxisActive_, ZRotationTargetAxisActive_);
    inverseKinematicsTarget.setOrientation(orientation);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseAndVelocityTarget(
    const std::string& frameName,
    const iDynTree::Transform& transform,
    const iDynTree::Twist& twist,
    const double positionFeedbackGain_,
    const double orientationFeedbackGain_,
    const double linearVelocityFeedforwardGain_,
    const double angularVelocityFeedforwardGain_,
    const double linearVelocityWeight_,
    const double angularVelocityWeight_,
    const bool XPositionTargetAxisActive_,
    const bool YPositionTargetAxisActive_,
    const bool ZPositionTargetAxisActive_,
    const bool XRotationTargetAxisActive_,
    const bool YRotationTargetAxisActive_,
    const bool ZRotationTargetAxisActive_)
{
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePose,
                                                    positionFeedbackGain_, orientationFeedbackGain_,
                                                    linearVelocityFeedforwardGain_, angularVelocityFeedforwardGain_,
                                                    linearVelocityWeight_, angularVelocityWeight_,
                                                    XPositionTargetAxisActive_, XPositionTargetAxisActive_, XPositionTargetAxisActive_,
                                                    XRotationTargetAxisActive_, YRotationTargetAxisActive_, ZRotationTargetAxisActive_);
    inverseKinematicsTarget.setTransform(transform);
    inverseKinematicsTarget.setTwist(twist);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseTarget(
    const std::string& frameName,
    const iDynTree::Transform& transform,
    const double positionFeedbackGain_,
    const double orientationFeedbackGain_,
    const double linearVelocityWeight_,
    const double angularVelocityWeight_,
    const bool XPositionTargetAxisActive_,
    const bool YPositionTargetAxisActive_,
    const bool ZPositionTargetAxisActive_,
    const bool XRotationTargetAxisActive_,
    const bool YRotationTargetAxisActive_,
    const bool ZRotationTargetAxisActive_)
{
    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePose,
                                                    positionFeedbackGain_, orientationFeedbackGain_,
                                                    0.0, 0.0,
                                                    linearVelocityWeight_, angularVelocityWeight_,
                                                    XPositionTargetAxisActive_, XPositionTargetAxisActive_, XPositionTargetAxisActive_,
                                                    XRotationTargetAxisActive_, YRotationTargetAxisActive_, ZRotationTargetAxisActive_);
    inverseKinematicsTarget.setTransform(transform);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseAndVelocityTarget(
    const std::string& frameName,
    const iDynTree::Vector3& position,
    const iDynTree::Rotation& orientation,
    const iDynTree::Vector3& linearVelocity,
    const iDynTree::Vector3& angularVelocity,
    const double positionFeedbackGain_,
    const double orientationFeedbackGain_,
    const double linearVelocityFeedforwardGain_,
    const double angularVelocityFeedforwardGain_,
    const double linearVelocityWeight_,
    const double angularVelocityWeight_,
    const bool XPositionTargetAxisActive_,
    const bool YPositionTargetAxisActive_,
    const bool ZPositionTargetAxisActive_,
    const bool XRotationTargetAxisActive_,
    const bool YRotationTargetAxisActive_,
    const bool ZRotationTargetAxisActive_)
{
    iDynTree::Twist twist(linearVelocity, angularVelocity);
    iDynTree::Transform transform(orientation, iDynTree::Position(position));

    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePose,
                                                    positionFeedbackGain_, orientationFeedbackGain_,
                                                    linearVelocityFeedforwardGain_, angularVelocityFeedforwardGain_,
                                                    linearVelocityWeight_, angularVelocityWeight_,
                                                    XPositionTargetAxisActive_, XPositionTargetAxisActive_, XPositionTargetAxisActive_,
                                                    XRotationTargetAxisActive_, YRotationTargetAxisActive_, ZRotationTargetAxisActive_);
    inverseKinematicsTarget.setTransform(transform);
    inverseKinematicsTarget.setTwist(twist);
    return inverseKinematicsTarget;
}

DynamicalInverseKinematics::impl::InverseKinematicsTarget
DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseTarget(
    const std::string& frameName,
    const iDynTree::Vector3& position,
    const iDynTree::Rotation& orientation,
    const double positionFeedbackGain_,
    const double orientationFeedbackGain_,
    const double linearVelocityWeight_,
    const double angularVelocityWeight_,
    const bool XPositionTargetAxisActive_,
    const bool YPositionTargetAxisActive_,
    const bool ZPositionTargetAxisActive_,
    const bool XRotationTargetAxisActive_,
    const bool YRotationTargetAxisActive_,
    const bool ZRotationTargetAxisActive_)
{
    iDynTree::Transform transform(orientation, iDynTree::Position(position));

    InverseKinematicsTarget inverseKinematicsTarget(frameName, TargetTypePose,
                                                    positionFeedbackGain_, orientationFeedbackGain_,
                                                    0.0, 0.0,
                                                    linearVelocityWeight_, angularVelocityWeight_,
                                                    XPositionTargetAxisActive_, XPositionTargetAxisActive_, XPositionTargetAxisActive_,
                                                    XRotationTargetAxisActive_, YRotationTargetAxisActive_, ZRotationTargetAxisActive_);
    inverseKinematicsTarget.setTransform(transform);
    return inverseKinematicsTarget;
}


DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType
DynamicalInverseKinematics::impl::InverseKinematicsTarget::getTargetType() const
{
    return type;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setTargetType(const DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType& targetType)
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

bool DynamicalInverseKinematics::impl::InverseKinematicsTarget::computeError(const iDynTree::Transform& transform, iDynTree::VectorDynSize& error)
{
    if ( (type != TargetTypePose) || (error.size() != errorSize))
    {
        return false;
    }

    error.setVal(0, activePositionTargetAxis[0] * (transform.getPosition().getVal(0) - targetTransform.getPosition().getVal(0)));
    error.setVal(1, activePositionTargetAxis[1] * (transform.getPosition().getVal(1) - targetTransform.getPosition().getVal(1)));
    error.setVal(2, activePositionTargetAxis[2] * (transform.getPosition().getVal(2) - targetTransform.getPosition().getVal(2)));

    if (std::count(activeOrientationTargetAxis.begin(), activeOrientationTargetAxis.end(), true) > 1)
    {
        // If we want to align all the axis, we can use skew-vee rotation error
        iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(hde::utils::idyntree::rotation::skewVee(transform.getRotation() * targetTransform.getRotation().inverse()));
    }
    else
    {
        if (activeOrientationTargetAxis[0])
        {
            iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(transform.getRotation()).row(0).cross(iDynTree::toEigen(targetTransform.getRotation()).row(0));
        }
        else if (activeOrientationTargetAxis[1])
        {
            iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(transform.getRotation()).row(1).cross(iDynTree::toEigen(targetTransform.getRotation()).row(1));
        }
        else if (activeOrientationTargetAxis[2])
        {
            iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(transform.getRotation()).row(2).cross(iDynTree::toEigen(targetTransform.getRotation()).row(2));
        }
        else
        {
            return false;
        }
        iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(transform.getRotation()) * iDynTree::toEigen(error).tail(3);
    }

    return true;
}

bool DynamicalInverseKinematics::impl::InverseKinematicsTarget::computeError(const iDynTree::Vector3& position, iDynTree::VectorDynSize& error)
{
    if ( (type != TargetTypePosition) || (error.size() != errorSize))
    {
        return false;
    }

    error.setVal(0, activePositionTargetAxis[0] * (position.getVal(0) - targetTransform.getPosition().getVal(0)));
    error.setVal(1, activePositionTargetAxis[1] * (position.getVal(1) - targetTransform.getPosition().getVal(1)));
    error.setVal(2, activePositionTargetAxis[2] * (position.getVal(2) - targetTransform.getPosition().getVal(2)));

    return true;
}


bool DynamicalInverseKinematics::impl::InverseKinematicsTarget::computeError(const iDynTree::Rotation& orientation, iDynTree::VectorDynSize& error)
{
    if ( (type != TargetTypeOrientation) || (error.size() != errorSize))
    {
        return false;
    }

    if (std::count(activeOrientationTargetAxis.begin(), activeOrientationTargetAxis.end(), true) > 1)
    {
        // If we want to align all the axis, we can use skew-vee rotation error
        iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(hde::utils::idyntree::rotation::skewVee(orientation * targetTransform.getRotation().inverse()));
    }
    else
    {
        if (activeOrientationTargetAxis[0])
        {
            iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(orientation).row(0).cross(iDynTree::toEigen(targetTransform.getRotation()).row(0));
        }
        else if (activeOrientationTargetAxis[1])
        {
            iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(orientation).row(1).cross(iDynTree::toEigen(targetTransform.getRotation()).row(1));
        }
        else if (activeOrientationTargetAxis[2])
        {
            iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(orientation).row(2).cross(iDynTree::toEigen(targetTransform.getRotation()).row(2));
        }
        else
        {
            return false;
        }
        iDynTree::toEigen(error).tail(3) = iDynTree::toEigen(orientation) * iDynTree::toEigen(error).tail(3);
    }

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

bool DynamicalInverseKinematics::impl::InverseKinematicsTarget::setActiveOrientationTargetAxis(const bool isXActive, const bool isYActive, const bool isZActive)
{
    // TODO: check if we should verify at least one axis is true,
    // and trow a warning if two axis have been selected (two axis is equivalent to three axis)
    activeOrientationTargetAxis = {isXActive, isYActive, isZActive};
    return true;
}

std::array<bool, 3> DynamicalInverseKinematics::impl::InverseKinematicsTarget::getActiveOrientationTargetAxes() const
{
    return activeOrientationTargetAxis;
}

std::array<bool, 3> DynamicalInverseKinematics::impl::InverseKinematicsTarget::getActivePositionTargetAxes() const
{
    return activePositionTargetAxis;
}

double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getPositionFeedbackGain() const
{
    return positionFeedbackGain;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setPositionFeedbackGain(
    const double& newPositionFeedbackGain)
{
    positionFeedbackGain = newPositionFeedbackGain;
}

double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getOrientationFeedbackGain() const
{
    return orientationFeedbackGain;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setOrientationFeedbackGain(
    const double& newOrientationFeedbackGain)
{
    orientationFeedbackGain = newOrientationFeedbackGain;
}

double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getLinearVelocityFeedforwardGain() const
{
    return linearVelocityFeedforwardGain;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setLinearVelocityFeedforwardGain(
    const double& newLinearVelocityFeedforwardGain)
{
    linearVelocityFeedforwardGain = newLinearVelocityFeedforwardGain;
}

double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getAngularVelocityFeedforwardGain() const
{
    return angularVelocityFeedforwardGain;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setAngularVelocityFeedforwardGain(
    const double& newAngularVelocityFeedforwardGain)
{
    angularVelocityFeedforwardGain = newAngularVelocityFeedforwardGain;
}

double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getLinearVelocityWeight() const
{
    return linearVelocityWeight;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setLinearVelocityWeight(
    const double& newLinearVelocityWeight)
{
    linearVelocityWeight = newLinearVelocityWeight;
}

double DynamicalInverseKinematics::impl::InverseKinematicsTarget::getAngularVelocityWeight() const
{
    return angularVelocityWeight;
}

void DynamicalInverseKinematics::impl::InverseKinematicsTarget::setAngularVelocityWeight(
    const double& newAngularVelocityWeight)
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
                if (!m_inverseVelocityKinematics.addLinearVelocityTarget(target.getFrameName(), target.getLinearVelocity(), target.getLinearVelocityWeight()))
                    return false;
                break;
            case InverseKinematicsTarget::TargetType::TargetTypeOrientation:
                if (!m_inverseVelocityKinematics.addAngularVelocityTarget(target.getFrameName(), target.getAngularVelocity(), target.getAngularVelocityWeight()))
                    return false;
                break;
            case InverseKinematicsTarget::TargetType::TargetTypePose:
                if (!m_inverseVelocityKinematics.addTarget(target.getFrameName(), target.getLinearVelocity(), target.getAngularVelocity(), target.getLinearVelocityWeight(), target.getAngularVelocityWeight()))
                    return false;
                break;
        }
    }
    if (!m_inverseVelocityKinematics.setGeneralJointVelocityConstraints(1E12)) // dummy value, that will be replaced by the custom joint velocity limits
        return false;

    if (!m_inverseVelocityKinematics.setConstraintParametersJointValues(m_linearLimits.k_u,
                                                                        m_linearLimits.k_l))
        return false;

    if (!m_inverseVelocityKinematics.setGeneralJointsUpperLowerConstraints(
            m_limits.jointPositionUpperLimit, m_limits.jointPositionLowerLimit))
        return false;

    if (m_limits.jointVelocityUpperLimit.size() > 0) {
        std::vector<iDynTree::JointIndex> jointsList;
        jointsList.clear();
        for (size_t i = 0; i < m_dofs; i++)
            jointsList.push_back(i);
        if (!m_inverseVelocityKinematics.setCustomJointsVelocityLimit(jointsList, m_limits.jointVelocityUpperLimit))
            return false;
    }
    if (m_linearLimits.constraintVariablesIndex.size() > 0) {
        if (!m_inverseVelocityKinematics.setCustomConstraintsJointsValues(
                m_linearLimits.constraintVariablesIndex,
                m_linearLimits.constraintUpperBound,
                m_linearLimits.constraintLowerBound,
                m_linearLimits.constraintMatrix))
            return false;
    }

    if (!m_inverseVelocityKinematics.setCustomBaseVelocityLimit(
        m_limits.baseVelocityLowerLimit, m_limits.baseVelocityUpperLimit))
        return false;

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

bool DynamicalInverseKinematics::impl::solveProblem(const double dt, bool &reset)
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
                              dt,
                              reset);
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
    m_netMeanPosErr.zero();
    m_netMeanRotErr.zero();
    std::array<std::size_t, 3> nrRotTargets{0, 0, 0};
    std::array<std::size_t, 3> nrPosTargets{0, 0, 0};
    std::array<bool, 3> posActivation, rotActivation;
    for (auto const& it : m_targets)
    {
        auto target = it.second;

        switch (target.getTargetType()) {
            case DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType::TargetTypePosition:
                m_errorBuffer.resize(3);
                if (!target.computeError(m_dynamics.getWorldTransform(target.getFrameName()).getPosition(), m_errorBuffer))
                    return false;
                iDynTree::toEigen(m_netMeanPosErr) += iDynTree::toEigen(m_errorBuffer);
                posActivation = target.getActivePositionTargetAxes();
                for (std::size_t idx = 0; idx < 3; idx++)
                {
                    if (posActivation[idx])
                    {
                        nrPosTargets[idx] +=1;
                    }
                }
                iDynTree::toEigen(m_desiredLinearVelocityBuffer) = target.getLinearVelocityFeedforwardGain() * iDynTree::toEigen(target.getLinearVelocity())
                                                                 - target.getPositionFeedbackGain()  * iDynTree::toEigen(m_errorBuffer);
                if (!m_inverseVelocityKinematics.updateTargetLinearVelocity(target.getFrameName(), m_desiredLinearVelocityBuffer, target.getLinearVelocityWeight()))
                    return false;
                break;
            case DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType::TargetTypeOrientation:
                m_errorBuffer.resize(3);
                if (!target.computeError(m_dynamics.getWorldTransform(target.getFrameName()).getRotation(), m_errorBuffer))
                    return false;
                iDynTree::toEigen(m_netMeanRotErr) += iDynTree::toEigen(m_errorBuffer);
                rotActivation = target.getActiveOrientationTargetAxes();
                for (std::size_t idx = 0; idx < 3; idx++)
                {
                    if (rotActivation[idx])
                    {
                        nrRotTargets[idx] +=1;
                    }
                }
                iDynTree::toEigen(m_desiredAngularVelocityBuffer) = target.getAngularVelocityFeedforwardGain() * iDynTree::toEigen(target.getAngularVelocity())
                                                                 - target.getOrientationFeedbackGain()  * iDynTree::toEigen(m_errorBuffer);
                if (!m_inverseVelocityKinematics.updateTargetAngularVelocity(target.getFrameName(), m_desiredAngularVelocityBuffer, target.getAngularVelocityWeight()))
                    return false;
                break;
            case DynamicalInverseKinematics::impl::InverseKinematicsTarget::TargetType::TargetTypePose:
                m_errorBuffer.resize(6);
                if (!target.computeError(m_dynamics.getWorldTransform(target.getFrameName()), m_errorBuffer))
                    return false;
                iDynTree::toEigen(m_netMeanPosErr) += iDynTree::toEigen(m_errorBuffer).head<3>();
                iDynTree::toEigen(m_netMeanRotErr) += iDynTree::toEigen(m_errorBuffer).tail<3>();
                posActivation = target.getActivePositionTargetAxes();
                rotActivation = target.getActiveOrientationTargetAxes();
                for (std::size_t idx = 0; idx < 3; idx++)
                {
                    if (posActivation[idx])
                    {
                        nrPosTargets[idx] +=1;
                    }

                    if (rotActivation[idx])
                    {
                        nrRotTargets[idx] +=1;
                    }
                }
                iDynTree::toEigen(m_desiredLinearVelocityBuffer) = target.getLinearVelocityFeedforwardGain() * iDynTree::toEigen(target.getLinearVelocity())
                                                                 - target.getPositionFeedbackGain()  * iDynTree::toEigen(m_errorBuffer).head(3);
                iDynTree::toEigen(m_desiredAngularVelocityBuffer) = target.getAngularVelocityFeedforwardGain() * iDynTree::toEigen(target.getAngularVelocity())
                                                                 - target.getOrientationFeedbackGain()  * iDynTree::toEigen(m_errorBuffer).tail(3);
                if (!m_inverseVelocityKinematics.updateTarget(target.getFrameName(), iDynTree::Twist(m_desiredLinearVelocityBuffer, m_desiredAngularVelocityBuffer), target.getLinearVelocityWeight(), target.getAngularVelocityWeight()))
                    return false;
                break;
        }
    }

    // compute mean error and its norm
    for (std::size_t idx = 0; idx < 3; idx++)
    {
        if (nrRotTargets[idx] > 0)
            m_netMeanRotErr.setVal(idx, m_netMeanRotErr(idx)/nrRotTargets[idx]);
        if (nrPosTargets[idx] > 0)
            m_netMeanPosErr.setVal(idx, m_netMeanPosErr(idx)/nrPosTargets[idx]);
    }

    m_rotMeanErrNorm = iDynTree::toEigen(m_netMeanRotErr).norm();
    m_posMeanErrNorm = iDynTree::toEigen(m_netMeanPosErr).norm();

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


bool DynamicalInverseKinematics::setLinearJointConfigurationLimits(
    const std::vector<iDynTree::JointIndex>& jointsIndexList,
    const iDynTree::VectorDynSize& upperBoundary,
    const iDynTree::VectorDynSize& lowerBoundary,
    const iDynTree::MatrixDynSize& customConstraintMatrix)
{
    if (!pImpl->m_isModelLoaded)
        return false;


    pImpl->m_linearLimits.constraintVariablesIndex = jointsIndexList;
    pImpl->m_linearLimits.constraintMatrix = customConstraintMatrix;
    pImpl->m_linearLimits.constraintUpperBound = upperBoundary;
    pImpl->m_linearLimits.constraintLowerBound = lowerBoundary;

    return true;
}

bool DynamicalInverseKinematics::setConstraintParametersJointValues(const double& k_u,
                                                                    const double& k_l)
{
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
                                                          const std::array<bool, 3> positionTargetActive,
                                                          const std::array<bool, 3> orientationTargetActive,
                                                          const double positionFeedbackGain,
                                                          const double orientationFeedbackGain,
                                                          const double linearVelocityFeedforwardGain,
                                                          const double angularVelocityFeedforwardGain,
                                                          const double linearVelocityWeight,
                                                          const double angularVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseAndVelocityTarget(
        linkName, transform, twist, positionFeedbackGain, orientationFeedbackGain, linearVelocityFeedforwardGain, angularVelocityFeedforwardGain, linearVelocityWeight, angularVelocityWeight, positionTargetActive[0], positionTargetActive[1], positionTargetActive[2], orientationTargetActive[0], orientationTargetActive[1], orientationTargetActive[2]));
}

bool DynamicalInverseKinematics::addPoseTarget(const std::string& linkName,
                const iDynTree::Transform& transform,
                const std::array<bool, 3> positionTargetActive,
                const std::array<bool, 3> orientationTargetActive,
                const double positionFeedbackGain,
                const double orientationFeedbackGain,
                const double linearVelocityWeight,
                const double angularVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseTarget(
        linkName, transform, positionFeedbackGain, orientationFeedbackGain, linearVelocityWeight, angularVelocityWeight, positionTargetActive[0], positionTargetActive[1], positionTargetActive[2], orientationTargetActive[0], orientationTargetActive[1], orientationTargetActive[2]));
}

bool DynamicalInverseKinematics::addPoseAndVelocityTarget(const std::string& linkName,
                            const iDynTree::Vector3& position,
                            const iDynTree::Rotation& orientation,
                            const iDynTree::Vector3& linearVelocity,
                            const iDynTree::Vector3& angularVelocity,
                            const std::array<bool, 3> positionTargetActive,
                            const std::array<bool, 3> orientationTargetActive,
                            const double positionFeedbackGain,
                            const double orientationFeedbackGain,
                            const double linearVelocityFeedforwardGain,
                            const double angularVelocityFeedforwardGain,
                            const double linearVelocityWeight,
                            const double angularVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseAndVelocityTarget(
        linkName, position, orientation, linearVelocity, angularVelocity, positionFeedbackGain, orientationFeedbackGain, linearVelocityFeedforwardGain, angularVelocityFeedforwardGain, linearVelocityWeight, angularVelocityWeight, positionTargetActive[0], positionTargetActive[1], positionTargetActive[2], orientationTargetActive[0], orientationTargetActive[1], orientationTargetActive[2]));
}

bool DynamicalInverseKinematics::addPoseTarget(const std::string& linkName,
                const iDynTree::Vector3& position,
                const iDynTree::Rotation& orientation,
                const std::array<bool, 3> positionTargetActive,
                const std::array<bool, 3> orientationTargetActive,
                const double positionFeedbackGain,
                const double orientationFeedbackGain,
                const double linearVelocityWeight,
                const double angularVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PoseTarget(
        linkName, position, orientation, positionFeedbackGain, orientationFeedbackGain, linearVelocityWeight, angularVelocityWeight, positionTargetActive[0], positionTargetActive[1], positionTargetActive[2], orientationTargetActive[0], orientationTargetActive[1], orientationTargetActive[2]));
}

bool DynamicalInverseKinematics::addPositionAndVelocityTarget(const std::string& linkName,
                                const iDynTree::Vector3& position,
                                const iDynTree::Vector3& linearVelocity,
                                const std::array<bool, 3> positionTargetActive,
                                const double positionFeedbackGain,
                                const double linearVelocityFeedforwardGain,
                                const double linearVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PositionAndVelocityTarget(
        linkName, position, linearVelocity, positionFeedbackGain, linearVelocityFeedforwardGain, linearVelocityWeight, positionTargetActive[0], positionTargetActive[1], positionTargetActive[2]));
}

bool DynamicalInverseKinematics::addPositionTarget(const std::string& linkName,
                    const iDynTree::Vector3& position,
                    const std::array<bool, 3> positionTargetActive,
                    const double positionFeedbackGain,
                    const double linearVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::PositionTarget(
        linkName, position, positionFeedbackGain, linearVelocityWeight, positionTargetActive[0], positionTargetActive[1], positionTargetActive[2]));
}

bool DynamicalInverseKinematics::addOrientationAndVelocityTarget(const std::string& linkName,
                                    const iDynTree::Rotation& orientation,
                                    const iDynTree::Vector3& angularVelocity,
                                    const std::array<bool, 3> orientationTargetActive,
                                    const double orientationFeedbackGain,
                                    const double angularVelocityFeedforwardGain,
                                    const double angularVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::OrientationAndVelocityTarget(
        linkName, orientation, angularVelocity, orientationFeedbackGain, angularVelocityFeedforwardGain, angularVelocityWeight, orientationTargetActive[0], orientationTargetActive[1], orientationTargetActive[2]));
}

bool DynamicalInverseKinematics::addOrientationTarget(const std::string& linkName,
                        const iDynTree::Rotation& orientation,
                        const std::array<bool, 3> orientationTargetActive,
                        const double orientationFeedbackGain,
                        const double angularVelocityWeight)
{
    return pImpl->addTarget(DynamicalInverseKinematics::impl::InverseKinematicsTarget::OrientationTarget(
        linkName, orientation, orientationFeedbackGain, angularVelocityWeight, orientationTargetActive[0], orientationTargetActive[1], orientationTargetActive[2]));
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

    if (!(setJointsConfiguration(jointsConfiguration) && setBasePose(baseTransform))) {
        return false;
    }

    return true;
}

bool DynamicalInverseKinematics::setConfiguration(const iDynTree::Vector3& basePosition,
                                                 const iDynTree::Rotation& baseRotation,
                                                 const iDynTree::VectorDynSize& jointsConfiguration)
{
    if (!pImpl->m_isModelLoaded)
        return false;

    if (!(setJointsConfiguration(jointsConfiguration) && setBasePose(basePosition, baseRotation))) {
        return false;
    }

    return true;
}

bool DynamicalInverseKinematics::updatePositionTargetAxis(const std::string& linkname,
                                                          const std::array<bool, 3> positionTargetActive)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkname);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.activePositionTargetAxis[0] = positionTargetActive[0];
    target->second.activePositionTargetAxis[1] = positionTargetActive[1];
    target->second.activePositionTargetAxis[2] = positionTargetActive[2];

    return true;
}

bool DynamicalInverseKinematics::updateOrientationTargetAxis(const std::string& linkname,
                                                             const std::array<bool, 3> orientationTargetActive)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkname);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.activeOrientationTargetAxis[0] = orientationTargetActive[0];
    target->second.activeOrientationTargetAxis[1] = orientationTargetActive[1];
    target->second.activeOrientationTargetAxis[2] = orientationTargetActive[2];

    return true;
}

bool DynamicalInverseKinematics::updateTarget(const std::string& linkName,
                                              const iDynTree::Transform& transform,
                                              const iDynTree::Twist& twist,
                                              const double positionFeedbackGain,
                                              const double orientationFeedbackGain,
                                              const double linearVelocityFeedforwardGain,
                                              const double angularVelocityFeedforwardGain,
                                              const double linearVelocityWeight,
                                              const double angularVelocityWeight)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setTransform(transform);
    target->second.setTwist(twist);
    target->second.setPositionFeedbackGain(positionFeedbackGain);
    target->second.setOrientationFeedbackGain(orientationFeedbackGain);
    target->second.setLinearVelocityFeedforwardGain(linearVelocityFeedforwardGain);
    target->second.setAngularVelocityFeedforwardGain(angularVelocityFeedforwardGain);
    target->second.setLinearVelocityWeight(linearVelocityWeight);
    target->second.setAngularVelocityWeight(angularVelocityWeight);
    return true;
}

bool DynamicalInverseKinematics::updateTarget(const std::string& linkName,
                                              const iDynTree::Vector3& position,
                                              const iDynTree::Rotation& orientation,
                                              const iDynTree::Vector3& linearVelocity,
                                              const iDynTree::Vector3& angularVelocity,
                                              const double positionFeedbackGain,
                                              const double orientationFeedbackGain,
                                              const double linearVelocityFeedforwardGain,
                                              const double angularVelocityFeedforwardGain,
                                              const double linearVelocityWeight,
                                              const double angularVelocityWeight)
{
    return updateTarget(linkName, iDynTree::Transform(orientation, iDynTree::Position(position)), iDynTree::Twist(linearVelocity, angularVelocity), positionFeedbackGain, orientationFeedbackGain, linearVelocityFeedforwardGain, angularVelocityFeedforwardGain, linearVelocityWeight, angularVelocityWeight);
}

bool DynamicalInverseKinematics::updateTargetPoseAndVelocity(const std::string& linkName,
                                                             const iDynTree::Transform& transform,
                                                             const iDynTree::Twist& twist)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setTransform(transform);
    target->second.setTwist(twist);
    return true;
}

bool DynamicalInverseKinematics::updateTargetPoseAndVelocity(const std::string& linkName,
                                                             const iDynTree::Vector3& position,
                                                             const iDynTree::Rotation& orientation,
                                                             const iDynTree::Vector3& linearVelocity,
                                                             const iDynTree::Vector3& angularVelocity)
{
    return updateTargetPoseAndVelocity(linkName, iDynTree::Transform(orientation, iDynTree::Position(position)), iDynTree::Twist(linearVelocity, angularVelocity));
}

bool DynamicalInverseKinematics::updateTargetPose(const std::string& linkName,
                                                  const iDynTree::Transform& transform)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setTransform(transform);
    return true;
}

bool DynamicalInverseKinematics::updateTargetPose(const std::string& linkName,
                                                  const iDynTree::Vector3& position,
                                                  const iDynTree::Rotation& orientation)
{
    return updateTargetPose(linkName, iDynTree::Transform(orientation, iDynTree::Position(position)));
}


bool DynamicalInverseKinematics::updateTargetPosition(const std::string& linkName,
                                                      const iDynTree::Vector3& position)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setPosition(position);
    return true;
}

bool DynamicalInverseKinematics::updateTargetPositionAndVelocity(const std::string& linkName,
                                                                 const iDynTree::Vector3& position,
                                                                 const iDynTree::Vector3& linearVelocity)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setPosition(position);
    target->second.setLinearVelocity(linearVelocity);
    return true;
}


bool DynamicalInverseKinematics::updateTargetOrientation(const std::string& linkName,
                                                         const iDynTree::Rotation& orientation)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setOrientation(orientation);
    return true;
}

bool DynamicalInverseKinematics::updateTargetOrientationAndVelocity(const std::string& linkName,
                                                                    const iDynTree::Rotation& orientation,
                                                                    const iDynTree::Vector3& angularVelocity)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setOrientation(orientation);
    target->second.setAngularVelocity(angularVelocity);
    return true;
}

bool DynamicalInverseKinematics::updateTargetLinearVelocity(const std::string& linkName,
                                                            const iDynTree::Vector3& linearVelocity)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setLinearVelocity(linearVelocity);
    return true;
}

bool DynamicalInverseKinematics::updateTargetAngularVelocity(const std::string& linkName,
                                                             const iDynTree::Vector3& angularVelocity)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setAngularVelocity(angularVelocity);
    return true;
}

bool DynamicalInverseKinematics::updateTargetGains(const std::string& linkName,
                                                   const double positionFeedbackGain,
                                                   const double orientationFeedbackGain,
                                                   const double linearVelocityFeedforwardGain,
                                                   const double angularVelocityFeedforwardGain)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setPositionFeedbackGain(positionFeedbackGain);
    target->second.setOrientationFeedbackGain(orientationFeedbackGain);
    target->second.setLinearVelocityFeedforwardGain(linearVelocityFeedforwardGain);
    target->second.setAngularVelocityFeedforwardGain(angularVelocityFeedforwardGain);
    return true;
}

bool DynamicalInverseKinematics::updateTargetWeights(const std::string& linkName,
                                                   const double linearVelocityWeight,
                                                   const double angularVelocityWeight)
{
    DynamicalInverseKinematics::impl::TargetsMap::iterator target =
        pImpl->getTargetRefIfItExists(linkName);
    if (target == pImpl->m_targets.end()) {
        return false;
    }

    target->second.setLinearVelocityWeight(linearVelocityWeight);
    target->second.setAngularVelocityWeight(angularVelocityWeight);
    return true;
}



bool DynamicalInverseKinematics::getJointsConfigurationSolution(iDynTree::VectorDynSize& jointsConfiguration) const
{
    if (!(jointsConfiguration.size() == pImpl->m_state.s.size()))
    {
        return false;
    }

    jointsConfiguration = pImpl->m_state.s;
    return true;
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
    return getBaseVelocitySolution(baseVelocity) && getJointsVelocitySolution(jointsVelocity);
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

bool DynamicalInverseKinematics::solve(const double dt, bool &reset)
{
    return pImpl->solveProblem(dt, reset);
}

void DynamicalInverseKinematics::clearProblem()
{

    pImpl->m_state.initializeState(pImpl->m_dofs);

    // initialize joint limits
    pImpl->m_limits.initializeLimits(pImpl->m_dofs);
    pImpl->getJointPositionLimitsFromModel();

    pImpl->m_isInverseKinematicsInitializd = false;
}

double DynamicalInverseKinematics::getTargetsMeanPositionErrorNorm() const
{
    return pImpl->m_posMeanErrNorm;
}

double DynamicalInverseKinematics::getTargetsMeanOrientationErrorNorm() const
{
    return pImpl->m_rotMeanErrNorm;
}
