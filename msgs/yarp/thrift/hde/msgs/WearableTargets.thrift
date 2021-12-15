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
    1: string wearableSensorName;
    2: string linkName;
    3: KinematicTargetType type;
    4: Vector3 position;
    5: Quaternion orientation;
    6: Vector3 linearVelocity;
    7: Vector3 angularVelocity;
}

struct WearableTargets {
    1: map<string,WearableTarget> targets;
}
