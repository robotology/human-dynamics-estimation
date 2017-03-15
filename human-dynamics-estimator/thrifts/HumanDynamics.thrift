namespace yarp human

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

struct LinkDynamicsEstimation {
    
    // link spatial accelerations (vector 6D)
    1: Vector spatialAcceleration;
    
    // net spatial wrench on body (vector 6D)
    2: Vector netWrench;
    
    // spatial wrench transmitted to body from his father (vector 6D)
    3: Vector transmittedWrench;
     
    // external wrench acting on body (vector 6D)
    4: Vector externalWrench;
    
    // TODO: decide if include or not the name
    // link name
    // 5: string internlinkName;
}



struct JointDynamicsEstimation {

    // TODO: let's consider in the future to use a vector of double
    // joint torque
    1: double torque;
    
    // TODO: let's consider in the future to use a vector of double
    // joint acceleration
    2: double acceleration;
    
    // TODO: decide if include or not the name
    // joint name
    // 3: string internjointName;
}

struct HumanDynamics {
    1: map<string,LinkDynamicsEstimation> linkVariables;
    2: map<string,JointDynamicsEstimation> jointVariables;
}





