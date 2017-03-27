namespace yarp human

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

struct LinkDynamicsEstimation {

    1: string linkName;
    // link spatial accelerations (vector 6D)
    2: Vector spatialAcceleration;

    // net spatial wrench on body (vector 6D)
    3: Vector netWrench;

    // external wrench acting on body (vector 6D)
    4: Vector externalWrench;

}



struct JointDynamicsEstimation {

    1: string jointName;
    // spatial wrench transmitted to body from his father (vector 6D)
    2: Vector transmittedWrench;

    // joint torque
    3: Vector torque;

    // joint acceleration
    4: Vector acceleration;

}

struct HumanDynamics {
    1: list<LinkDynamicsEstimation> linkVariables;
    2: list<JointDynamicsEstimation> jointVariables;
}

