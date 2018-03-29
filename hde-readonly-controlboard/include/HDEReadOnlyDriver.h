#ifndef HDEREADONLYDRIVER_H
#define HDEREADONLYDRIVER_H

#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <cmath>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/ITorqueControl.h>

namespace yarp
{
    namespace dev
    {
        class HDEReadOnlyDriver;
    }
}

class yarp::dev::HDEReadOnlyDriver:
    public yarp::dev::DeviceDriver,
    public yarp::dev::IAxisInfo,
    public yarp::dev::IPositionControl,
    public yarp::dev::IVelocityControl,
    public yarp::dev::IEncodersTimed,
    public yarp::dev::ITorqueControl
{

private:

public:

    int number_of_dofs;
    std::vector<std::string> joint_name_list;

    yarp::sig::Vector joint_positions;
    yarp::sig::Vector joint_velocities;
    yarp::sig::Vector joint_accelerations;

    yarp::sig::Vector joint_positions_rad;
    yarp::sig::Vector joint_velocities_rad;
    yarp::sig::Vector joint_accelerations_rad;

    yarp::sig::Vector joint_torques;

    HDEReadOnlyDriver() {};
    ~HDEReadOnlyDriver() {};

    //Device Driver
    bool open(yarp::os::Searchable& config) override
    {
        return true;
    }

    bool close() override
    {
        return yarp::dev::DeviceDriver::close();
    }

    //Axis Info
    bool getAxisName(int axis, yarp::os::ConstString& name) override;
    bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override;

    //Position Control
    bool stop() override;
    bool stop(int j) override;
    bool getAxes(int *ax) override;
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

    //Velocity Control
    bool velocityMove(int j, double sp) override;
    bool velocityMove(const double* sp) override;
    bool setRefAcceleration(int j, double acc) override;
    bool setRefAccelerations(const double* accs) override;
    bool getRefAcceleration(int j, double* acc) override;
    bool getRefAccelerations(double* accs) override;

    //Encoders
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

    //Encoders Timed
    bool getEncoderTimed(int j, double* encs, double* time) override;
    bool getEncodersTimed(double* encs, double* time) override;

    //Torque Control
    bool setRefTorque(int j, double t) override;
    bool setRefTorques(const double *t) override;
    bool setTorqueMode();
    bool getRefTorque(int j, double *t) override;
    bool getRefTorques(double *t) override;
    bool setRefTorques(const int n_joint, const int *joints, const double *t) override;
    bool getTorque(int j, double *t) override;
    bool getTorques(double *t) override;

    bool getBemfParam(int j, double *bemf) override;
    bool setBemfParam(int j, double bemf) override;
    bool getTorqueRange(int j, double *min, double *max) override;
    bool getTorqueRanges(double *min, double *max) override;
    bool getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params) override;
    bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;

};

#endif // HDEREADONLYDRIVER_H
