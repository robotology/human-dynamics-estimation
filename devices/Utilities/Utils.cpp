/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

// std
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>

#include <Utils.hpp>

iDynTree::Matrix3x3 iDynTreeHelper::Rotation::skewSymmetric(const iDynTree::Matrix3x3& input)
{
    iDynTree::Matrix3x3 output;
    iDynTree::toEigen(output) = 0.5 * (iDynTree::toEigen(input) - iDynTree::toEigen(input).transpose());
    return output;
}

iDynTree::Vector3 iDynTreeHelper::Rotation::skewVee(const iDynTree::Matrix3x3& input)
{
    iDynTree::Vector3 output;
    iDynTree::Matrix3x3 skewSymmetric;

    skewSymmetric = iDynTreeHelper::Rotation::skewSymmetric(input);
    iDynTree::toEigen(output) = iDynTree::unskew(iDynTree::toEigen(skewSymmetric));

    return output;
}

iDynTreeHelper::Rotation::rotationDistance::rotationDistance()
{}

iDynTreeHelper::Rotation::rotationDistance::rotationDistance(const rotationDistance& rotationDistance):
    rotation1(rotationDistance.rotation1),
    rotation2(rotationDistance.rotation2)
{}

iDynTreeHelper::Rotation::rotationDistance::rotationDistance(const iDynTree::Rotation rotation1, const iDynTree::Rotation rotation2):
    rotation1(rotation1),
    rotation2(rotation2)
{}

iDynTreeHelper::Rotation::rotationDistance::rotationDistance(const iDynTree::Transform transform1, const iDynTree::Transform transform2):
    rotation1(transform1.getRotation()),
    rotation2(transform2.getRotation())
{}

iDynTree::Rotation iDynTreeHelper::Rotation::rotationDistance::asRotation()
{
    iDynTree::Rotation distanceRotationMatrix = rotation1 * rotation2.inverse();
    return distanceRotationMatrix;
}

iDynTree::Vector3 iDynTreeHelper::Rotation::rotationDistance::asRPY()
{
    return this->asRotation().asRPY();
}

iDynTree::Vector4 iDynTreeHelper::Rotation::rotationDistance::asQuaternion()
{
    return this->asRotation().asQuaternion();
}

double iDynTreeHelper::Rotation::rotationDistance::asEuclideanDistanceOfEulerAngles()
{
    iDynTree::Vector3 rpy1;
    iDynTree::Vector3 rpy2;

    rpy1 = rotation1.asRPY();
    rpy2 = rotation2.asRPY();

    double euclideanDistance = 0;
    double angleDiff;
    for (unsigned int i=0; i<3; i++)
    {
        angleDiff = abs(rpy1.getVal(i) - rpy2.getVal(i));
        euclideanDistance += std::max(angleDiff, M_PI - angleDiff);
    }
    euclideanDistance = sqrt(euclideanDistance);

    return euclideanDistance;
}

iDynTree::Vector3 iDynTreeHelper::Rotation::rotationDistance::asSkewVee()
{
    iDynTree::Vector3 output;
    output = iDynTreeHelper::Rotation::skewVee(this->asRotation());

    return output;
}

double iDynTreeHelper::Rotation::rotationDistance::asTrace()
{
    double output;
    iDynTree::Matrix3x3 matrix;

    iDynTree::toEigen(matrix) = iDynTree::toEigen(iDynTree::Rotation::Identity()) - iDynTree::toEigen(this->asRotation());
    output = matrix.getVal(0, 0) + matrix.getVal(1, 1) + matrix.getVal(2, 2);

    return output;
}



