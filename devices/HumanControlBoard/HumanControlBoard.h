/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_HUMANCONTROLBOARD
#define HDE_DEVICES_HUMANCONTROLBOARD

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IControlLimits.h>

#include <memory>

namespace hde {
    namespace devices {
        class HumanControlBoard;
    } // namespace devices
} // namespace hde

class hde::devices::HumanControlBoard final
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
    , public yarp::os::PeriodicThread
    , public yarp::dev::IAxisInfo
    , public yarp::dev::IEncodersTimed
    , public yarp::dev::IPositionControl
    , public yarp::dev::IVelocityControl
    , public yarp::dev::ITorqueControl
    , public yarp::dev::IControlLimits
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanControlBoard();
    ~HumanControlBoard() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;

    // IAxisInfo interace
    bool getAxisName(int axis, std::string& name) override;
    bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override { return false; }

    // IEncoders interface
    bool getAxes(int* ax) override;
    bool getEncoder(int j, double* v) override;
    bool getEncoders(double* encs) override;
    bool getEncoderSpeed(int j, double* sp) override;
    bool getEncoderSpeeds(double* spds) override;
    bool getEncoderAcceleration(int j, double* spds) override;
    bool getEncoderAccelerations(double* accs) override;

    bool resetEncoder(int j) override { return false; }
    bool resetEncoders() override { return false; }
    bool setEncoder(int j, double val) override { return false; }
    bool setEncoders(const double* vals) override { return false; }

    // IEncodersTimed interface
    bool getEncoderTimed(int j, double* encs, double* time) override;
    bool getEncodersTimed(double* encs, double* time) override;

    // IPositionControl Interface
    bool stop() override { return false; }
    bool stop(int j) override { return false; }
    bool stop(const int n_joint, const int *joints) override { return false; }
    bool positionMove(int j, double ref) override { return false; }
    bool positionMove(const double* refs) override { return false; }
    bool positionMove(const int n_joint, const int *joints, const double *refs) override { return false; }
    bool setRefSpeed(int j, double sp) override { return false; }
    bool setRefSpeeds(const double* spds) override { return false; }
    bool setRefSpeeds(const int n_joint, const int *joints, const double *spds) override { return false; }
    bool getRefSpeed(int j, double* ref) override { return false; }
    bool getRefSpeeds(double* spds) override { return false; }
    bool getRefSpeeds(const int n_joint, const int *joints, double *spds) override { return false; }
    bool relativeMove(int j, double delta) override { return false; }
    bool relativeMove(const double* deltas) override { return false; }
    bool relativeMove(const int n_joint, const int *joints, const double *deltas) override { return false; }
    bool checkMotionDone(int j, bool* flag) override { return false; }
    bool checkMotionDone(bool* flag) override { return false; }
    bool checkMotionDone(const int n_joint, const int *joints, bool *flags) override { return false; }
    bool setRefAccelerations(const int n_joint, const int *joints, const double *accs) override { return false; }
    bool getRefAccelerations(const int n_joint, const int *joints, double *accs) override { return false; }

    // IVelocityControl interface
    bool velocityMove(int j, double sp) override { return false; }
    bool velocityMove(const double* sp) override { return false; }
    bool velocityMove(const int n_joint, const int *joints, const double *spds) override { return false; }
    bool setRefAcceleration(int j, double acc) override { return false; }
    bool setRefAccelerations(const double* accs) override { return false; }
    bool getRefAcceleration(int j, double* acc) override { return false; }
    bool getRefAccelerations(double* accs) override { return false; }

    // ITorqueControl interface
    bool getTorque(int j, double* t) override;
    bool getTorques(double* t) override;

    bool setTorqueMode() { return false; }
    bool setRefTorque(int j, double t) override { return false; }
    bool setRefTorques(const double* t) override { return false; }
    bool getRefTorque(int j, double* t) override { return false; }
    bool getRefTorques(double* t) override { return false; }
    bool setRefTorques(const int n_joint, const int* joints, const double* t) override { return false; }
    bool getBemfParam(int j, double* bemf) { return false; }
    bool setBemfParam(int j, double bemf) { return false; }
    bool getTorqueRange(int j, double* min, double* max) override { return false; }
    bool getTorqueRanges(double* min, double* max) override { return false; }
    bool getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params) override { return false; }
    bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override { return false; }

    // IControlLimits
    bool setLimits(int axis, double min, double max) override  { return false; }
    bool getLimits(int axis, double *min, double *max) override;
    bool setVelLimits(int axis, double min, double max) override  { return false; }
    bool getVelLimits(int axis, double *min, double *max) override;
};

#endif // HDE_DEVICES_HUMANCONTROLBOARD
