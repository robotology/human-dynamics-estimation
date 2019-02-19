namespace yarp human

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
 * Representation of the IHumanState interface
 */
struct HumanState {
    1: list<string> jointNames;
    2: list<double> positions;
    3: list<double> velocities;

    4: Vector3 baseOriginWRTGlobal;
    5: Quaternion baseOrientationWRTGlobal;
    6: list<double> baseVelocityWRTGlobal;
    //TODO add the name of the base link
}
