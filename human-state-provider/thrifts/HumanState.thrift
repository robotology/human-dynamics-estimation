namespace yarp human

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

/**
 * Representation of a 3D vector
 */
struct Vector3 {
    1: double x;
    2: double y;
    3: double z;
}

/**
 * Representation of a Quaternion
 */
struct Quaternion {
    1: double w;
    2: Vector3 imaginary;
}

/**
 * Information about generalized coordinates of human
 * The name of the object is for "compatibility" with YARP
 * even if this is not properly a state of a dynamic system.
 * \todo decide if put the base pose as explicit or not
 */
struct HumanState {
    1: Vector positions;
    2: Vector velocities;

    3: Vector3 baseOriginWRTGlobal;
    4: Quaternion baseOrientationWRTGlobal;
    5: Vector baseVelocityWRTGlobal;
}
