namespace yarp xsens

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
 * Data characterizing a segment.
 * It is composed of origin position, orientation in quaternion,
 * linear and angular velocity and acceleration.\
 * All the quantities are written with respect a general world frame.
 */
struct XsensSegmentData {
    //Linear quantities
    /** Position of the origin of the segment frame */
    1:  Vector3 position;
    /** linear velocity of the segment frame */
    2:  Vector3 velocity;
    /** linear acceleration of the segment frame */
    3:  Vector3 acceleration;
    
    //Angular quantities
    /** orientation of the segment frame in quaternion */
    4:  Quaternion orientation;
    /** angular velocity of the segment frame */
    5:  Vector3 angularVelocity;
    /** angular acceleration of the segment frame */
    6:  Vector3 angularAcceleration;
}

enum XsensStatus {
    OK = 0,
    ERROR = 1,
    NO_DATA = 2,
    TIMEOUT = 4
}

/**
 * Data characterizing sensor data.
 * It is composed of orientation in quaternion,
 * angular velocity acceleration and magnetometer information
 */
struct XsensSensorData {
    /** sensor acceleration in m / s^2 */
    1:  Vector3 acceleration;
    /** sensor angular velocity in deg / s */
    2:  Vector3 angularVelocity;
    /** The magnetometer data in arbitrary units */
    3:  Vector3 magnetometer;
    /** The sensor computed orientation */
    4:  Quaternion orientation;
}

/** Frame output from Xsens with segments data
 */
struct XsensSegmentsFrame {
    1: required XsensStatus status;
    /** segments data */
    2: optional list<XsensSegmentData> segmentsData;
}

/** Frame output from Xsens with sensors raw data
 */
struct XsensSensorsFrame {
    1: required XsensStatus status;
    /** sensors data */
    2: optional list<XsensSensorData> sensorsData
}
        