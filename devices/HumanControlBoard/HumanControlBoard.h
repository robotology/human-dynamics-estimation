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
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>

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
//    , public yarp::dev::IPositionControl
//, public yarp::dev::IVelocityControl
//, public yarp::dev::ITorqueControl
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

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;

    // IAxisInfo interace
    bool getAxisName(int axis, std::string& name) override;
    bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override;

    // IEncoders interface
    bool getAxes(int* ax) override;
    bool getEncoder(int j, double* v) override;
    bool getEncoders(double* encs) override;
    bool resetEncoder(int j) override;
    bool resetEncoders() override;
    bool setEncoder(int j, double val) override;
    bool setEncoders(const double* vals) override;
    bool getEncoderSpeed(int j, double* sp) override;
    bool getEncoderSpeeds(double* spds) override;
    bool getEncoderAcceleration(int j, double* spds) override;
    bool getEncoderAccelerations(double* accs) override;

    // IEncodersTimed interface
    bool getEncoderTimed(int j, double* encs, double* time) override;
    bool getEncodersTimed(double* encs, double* time) override;

    /*// IPositionControl Interface
    bool stop() override;
    bool stop(int j) override;
    // bool getAxes(int* ax) override;
    bool positionMove(int j, double ref) override;
    bool positionMove(const double* refs) override;
    bool setRefSpeed(int j, double sp) override;
    bool setRefSpeeds(const double* spds) override;
    bool getRefSpeed(int j, double* ref) override;
    bool getRefSpeeds(double* spds) override;
    bool relativeMove(int j, double delta) override;
    bool relativeMove(const double* deltas) override;
    bool checkMotionDone(int j, bool* flag) override;
    bool checkMotionDone(bool* flag) override;

    // IVelocityControl interface
    bool velocityMove(int j, double sp) override;
    bool velocityMove(const double* sp) override;
    bool setRefAcceleration(int j, double acc) override;
    bool setRefAccelerations(const double* accs) override;
    bool getRefAcceleration(int j, double* acc) override;
    bool getRefAccelerations(double* accs) override;

    // ITorqueControl interface
    bool setRefTorque(int j, double t) override;
    bool setRefTorques(const double* t) override;
    bool setTorqueMode();
    bool getRefTorque(int j, double* t) override;
    bool getRefTorques(double* t) override;
    bool setRefTorques(const int n_joint, const int* joints, const double* t) override;
    bool getTorque(int j, double* t) override;
    bool getTorques(double* t) override;

    bool getBemfParam(int j, double* bemf);
    bool setBemfParam(int j, double bemf);
    bool getTorqueRange(int j, double* min, double* max) override;
    bool getTorqueRanges(double* min, double* max) override;
    bool getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params) override;
    bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;*/
};

#endif // HDE_DEVICES_HUMANCONTROLBOARD
