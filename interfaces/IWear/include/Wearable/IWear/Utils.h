// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLE_UTILS_H
#define WEARABLE_UTILS_H

#include <cmath>

#include "Wearable/IWear/Sensors/ISensor.h"

namespace wearable {
    namespace utils {
        inline Quaternion normalizeQuaternion(const Quaternion& quat);
        inline Vector3 quaternionToRPY(const Quaternion& quat);
        inline Matrix3 quaternionToRotationMatrix(const Quaternion& quat);
        inline Quaternion rotationMatrixToQuaternion(const Matrix3& rotMat);
        inline Vector3 rotationMatrixToRPY(const Matrix3& rotMat);
        inline Quaternion RPYToQuaternion(const Vector3& rpy);
        inline Matrix3 RPYToRotationMatrix(const Vector3& rpy);
    } // namespace utils
} // namespace wearable

inline wearable::Quaternion wearable::utils::normalizeQuaternion(const Quaternion& quat)
{
    double norm =
        std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);

    Quaternion normalQuat(quat);

    if (norm != 1.0) {
        Quaternion normalQuat;
        for (auto& qi : normalQuat) {
            qi /= norm;
        }
    }
    return normalQuat;
}

inline wearable::Vector3 wearable::utils::quaternionToRPY(const Quaternion& quat)
{
    Quaternion q = normalizeQuaternion(quat);

    double qw = q[0];
    double qx = q[1];
    double qy = q[2];
    double qz = q[3];

    Vector3 rpy;

    // roll (x-axis rotation)
    double sinr = 2.0 * (qw * qx + qy * qz);
    double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
    rpy[0] = std::atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qx * qz);
    if (std::abs(sinp) >= 1.0) {
        rpy[1] = std::copysign(M_PI / 2, sinp);
    }
    else {
        rpy[1] = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny = 2.0 * (qw * qz + qx * qy);
    double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    rpy[2] = std::atan2(siny, cosy);

    return rpy;
};

inline wearable::Matrix3 wearable::utils::quaternionToRotationMatrix(const Quaternion& quat)
{
    Quaternion q = normalizeQuaternion(quat);

    double qw = q[0];
    double qx = q[1];
    double qy = q[2];
    double qz = q[3];

    Matrix3 rotMat;

    // first row
    rotMat[0][0] = 1.0 - 2.0 * qy * qy - 2.0 * qz * qz;
    rotMat[0][1] = 2.0 * qx * qy - 2.0 * qz * qw;
    rotMat[0][2] = 2.0 * qx * qz + 2.0 * qy * qw;

    // second row
    rotMat[1][0] = 2.0 * qx * qy + 2.0 * qz * qw;
    rotMat[1][1] = 1.0 - 2.0 * qx * qx - 2.0 * qz * qz;
    rotMat[1][2] = 2.0 * qy * qz - 2.0 * qx * qw;

    // third row
    rotMat[2][0] = 2.0 * qx + qz - 2.0 * qy * qw;
    rotMat[2][1] = 2.0 * qy * qz + 2 * qx * qw;
    rotMat[2][2] = 1.0 - 2.0 * qx * qx - 2.0 * qy * qy;

    return rotMat;
};

inline wearable::Quaternion wearable::utils::rotationMatrixToQuaternion(const Matrix3& rotMat)
{

    // Taken from "Contributions to the automatic control of aerial vehicles"
    // PhD thesis of "Minh Duc HUA"
    // INRIA Sophia Antipolis
    // Equation 3.9 (page 101)

    // Diagonal elements used only to find the maximum
    // the furthest value from zero
    // we use this value as denominator to find the other elements
    double qw = (rotMat[0][0] + rotMat[1][1] + rotMat[2][2] + 1.0);
    double qx = (rotMat[0][0] - rotMat[1][1] - rotMat[2][2] + 1.0);
    double qy = (-rotMat[0][0] + rotMat[1][1] - rotMat[2][2] + 1.0);
    double qz = (-rotMat[0][0] - rotMat[1][1] + rotMat[2][2] + 1.0);

    if (qw < 0.0) {
        qw = 0.0;
    }
    if (qx < 0.0) {
        qx = 0.0;
    }
    if (qy < 0.0) {
        qy = 0.0;
    }
    if (qz < 0.0) {
        qz = 0.0;
    }

    if (qw >= qx && qw >= qy && qw >= qz) {
        qw = std::sqrt(qw);
        qx = (rotMat[2][1] - rotMat[1][2]) / (2.0 * qw);
        qy = (rotMat[0][2] - rotMat[2][0]) / (2.0 * qw);
        qz = (rotMat[1][0] - rotMat[0][1]) / (2.0 * qw);
        qw /= 2.0;
    }
    else if (qx >= qw && qx >= qy && qx >= qz) {
        qx = std::sqrt(qx);
        qw = (rotMat[2][1] - rotMat[1][2]) / (2.0 * qx);
        qy = (rotMat[1][0] + rotMat[0][1]) / (2.0 * qx);
        qz = (rotMat[2][0] + rotMat[0][2]) / (2.0 * qx);
        qx /= 2.0;
    }
    else if (qy >= qw && qy >= qx && qy >= qz) {
        qy = std::sqrt(qy);
        qw = (rotMat[0][2] - rotMat[2][0]) / (2.0 * qy);
        qx = (rotMat[1][0] + rotMat[0][1]) / (2.0 * qy);
        qz = (rotMat[1][2] + rotMat[2][1]) / (2.0 * qy);
        qy /= 2.0;
    }
    else if (qz >= qw && qz >= qx && qz >= qy) {
        qz = std::sqrt(qz);
        qw = (rotMat[1][0] - rotMat[0][1]) / (2.0 * qz);
        qx = (rotMat[2][0] + rotMat[0][2]) / (2.0 * qz);
        qy = (rotMat[1][2] + rotMat[2][1]) / (2.0 * qz);
        qz /= 2.0;
    }

    // Here we impose that the leftmost nonzero element of the quaternion is positive
    double eps = 1e-7;
    double sign = 1.0;
    if (qw > eps || qw < -eps) {
        sign = qw > 0 ? 1.0 : -1.0;
    }
    else if (qx > eps || qx < -eps) {
        sign = qx > 0 ? 1.0 : -1.0;
    }
    else if (qy > eps || qy < -eps) {
        sign = qy > 0 ? 1.0 : -1.0;
    }
    else if (qz > eps || qz < -eps) {
        sign = qz > 0 ? 1.0 : -1.0;
    }

    Quaternion quat;
    quat[0] = qw / sign;
    quat[1] = qx / sign;
    quat[2] = qy / sign;
    quat[3] = qz / sign;

    // return normalized quaternion
    return normalizeQuaternion(quat);
};

inline wearable::Vector3 wearable::utils::rotationMatrixToRPY(const wearable::Matrix3& rotMat)
{

    Vector3 rpy;

    if (rotMat[2][0] < 1.0) {
        if (rotMat[2][0] > -1.0) {
            rpy[0] = std::atan2(rotMat[2][1], rotMat[2][2]);
            rpy[1] = std::asin(-rotMat[2][0]);
            rpy[2] = std::atan2(rotMat[1][0], rotMat[0][0]);
        }
        else {
            // Not a unique solution
            rpy[0] = 0.0;
            rpy[1] = M_PI / 2.0;
            rpy[2] = -std::atan2(-rotMat[1][2], rotMat[1][1]);
        }
    }
    else {
        // Not a unique solution
        rpy[0] = 0.0;
        rpy[1] = -M_PI / 2.0;
        rpy[2] = std::atan2(-rotMat[1][2], rotMat[1][1]);
    }
    return rpy;
};

inline wearable::Quaternion wearable::utils::RPYToQuaternion(const Vector3& rpy)
{
    double cr = std::cos(rpy[0] * 0.5);
    double sr = std::sin(rpy[0] * 0.5);
    double cp = std::cos(rpy[1] * 0.5);
    double sp = std::sin(rpy[1] * 0.5);
    double cy = std::cos(rpy[2] * 0.5);
    double sy = std::sin(rpy[2] * 0.5);

    Quaternion quat;

    quat[0] = cr * cp * cy + sr * sp * sy;
    quat[1] = sr * cp * cy - cr * sp * sy;
    quat[2] = cr * sp * cy + sr * cp * sy;
    quat[3] = cr * cp * sy - sr * sp * cy;

    return quat;
};

inline wearable::Matrix3 wearable::utils::RPYToRotationMatrix(const Vector3& rpy)
{
    double cr = std::cos(rpy[0]);
    double sr = std::sin(rpy[0]);
    double cp = std::cos(rpy[1]);
    double sp = std::sin(rpy[1]);
    double cy = std::cos(rpy[2]);
    double sy = std::sin(rpy[2]);

    Matrix3 rotMat;

    // first row
    rotMat[0][0] = cp * cy;
    rotMat[0][1] = sr * sp * cy - cr * sy;
    rotMat[0][2] = sr * sy + cr * sp * cy;

    // second row
    rotMat[1][0] = cp * sy;
    rotMat[1][1] = cr * cy + sr * sp * sy;
    rotMat[1][2] = cr * sp * sy - sr * cy;

    // third row
    rotMat[2][0] = -sp;
    rotMat[2][1] = sr * cp;
    rotMat[2][2] = cr * cp;

    return rotMat;
};

#endif // WEARABLE_UTILS_H
