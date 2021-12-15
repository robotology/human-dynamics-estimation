namespace yarp hde.msgs

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

enum KinematicTargetType {
    NONE,
    POSE,
    POSEANDVELOCITY,
    POSITION,
    POSITIONANDVELOCITY,
    ORIENTATION,
    ORIENTATIONANDVELOCITY,
}

struct WearableTarget {
    1: string linkName;
    2: KinematicTargetType type;
    3: Vector3 position;
    4: Quaternion orientation;
    5: Vector3 linearVelocity;
    6: Vector3 angularVelocity;
}

struct WearableTargets {
    1: map<string,WearableTarget> targets;
}
