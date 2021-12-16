/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_INTERFACES_IWEARABLETARGETS
#define HDE_INTERFACES_IWEARABLETARGETS

#include <array>
#include <string>
#include <vector>
#include <mutex>

#include <iDynTree/Core/SpatialVector.h>
#include <iDynTree/Core/Transform.h>
#include <Wearable/IWear/IWear.h>

namespace hde {

    enum KinematicTargetType
    {
        none,
        pose,
        poseAndVelocity,
        position,
        positionAndVelocity,
        orientation,
        orientationAndVelocity
    };

    using TargetName = std::string;
    using ModelLinkName = std::string;

    class WearableSensorTarget
    {
        public:
        wearable::WearableName wearableName;
        ModelLinkName modelLinkName;
        hde::KinematicTargetType targetType;

        iDynTree::Vector3 position;
        iDynTree::Rotation rotation;
        iDynTree::Vector3 linearVelocity;
        iDynTree::Vector3 angularVelocity;

        iDynTree::Transform calibrationWorldToMeasurementWorld;
        iDynTree::Transform calibrationMeasurementToLink;
        iDynTree::Vector3 positionScaleFactor;

        // buffer variables
        iDynTree::Vector3 positionInWorld;
        iDynTree::Vector3 positionScaled;
        iDynTree::Vector3 linearVelocityInWorld;
        iDynTree::Vector3 linearVelocityScaled;

        mutable std::mutex mutex;

        WearableSensorTarget(wearable::WearableName wearableName_,
                            ModelLinkName modelLinkName_,
                            hde::KinematicTargetType targetType_)
                            : wearableName(wearableName_)
                            , modelLinkName(modelLinkName_)
                            , targetType(targetType_)
        {
            position.zero();
            rotation.Identity();
            linearVelocity.zero();
            angularVelocity.zero();
            positionScaleFactor(0) = 1;
            positionScaleFactor(1) = 1;
            positionScaleFactor(2) = 1;
            clearCalibrationMatrices();
        };

        void clearCalibrationMatrices()
        {
            std::lock_guard<std::mutex> lock(mutex);
            calibrationWorldToMeasurementWorld = iDynTree::Transform::Identity();
            calibrationMeasurementToLink = iDynTree::Transform::Identity();
        };

        iDynTree::Vector3 getCalibratedPosition()
        {
            std::lock_guard<std::mutex> lock(mutex);
            positionInWorld = iDynTree::Position(position).changeCoordinateFrame(calibrationWorldToMeasurementWorld.getRotation()) + calibrationWorldToMeasurementWorld.getPosition();
            // scale position
            positionScaled.setVal(0, positionInWorld.getVal(0) * positionScaleFactor(0));
            positionScaled.setVal(1, positionInWorld.getVal(1) * positionScaleFactor(1));
            positionScaled.setVal(2, positionInWorld.getVal(2) * positionScaleFactor(2));

            return positionScaled;
        };

        iDynTree::Rotation getCalibratedRotation()
        {
            std::lock_guard<std::mutex> lock(mutex);
            return calibrationWorldToMeasurementWorld.getRotation() * rotation * calibrationMeasurementToLink.getRotation();
        };

        iDynTree::Vector3 getCalibratedLinearVelocity()
        {
            std::lock_guard<std::mutex> lock(mutex);
            linearVelocityInWorld = iDynTree::LinearMotionVector3(linearVelocity).changeCoordFrame(calibrationWorldToMeasurementWorld.getRotation());
            // scale linear velocity
            linearVelocityScaled.setVal(0, linearVelocityInWorld.getVal(0) * positionScaleFactor(0));
            linearVelocityScaled.setVal(1, linearVelocityInWorld.getVal(1) * positionScaleFactor(1));
            linearVelocityScaled.setVal(2, linearVelocityInWorld.getVal(2) * positionScaleFactor(2));
            return linearVelocityScaled;
        };

        iDynTree::Vector3 getCalibratedAngularVelocity()
        {
            std::lock_guard<std::mutex> lock(mutex);
            return iDynTree::AngularMotionVector3(angularVelocity).changeCoordFrame(calibrationWorldToMeasurementWorld.getRotation());
        };
    };

    namespace interfaces {
        class IWearableTargets;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IWearableTargets
{
public:
    virtual ~IWearableTargets() = default;

    virtual std::vector<TargetName> getAllTargetsName() const = 0;

    virtual std::shared_ptr<hde::WearableSensorTarget> getTarget(const TargetName name) const = 0;
};

#endif // HDE_INTERFACES_IWEARABLETARGETS
