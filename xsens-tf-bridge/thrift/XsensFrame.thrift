namespace yarp xsens

/**
 * Representation of a 3D vector
 */
struct Vector3 {
    1: double c1;
    2: double c2;
    3: double c3;
}

/**
 * Representation of a 4D vector
 */
struct Vector4 {
    1: double c1;
    2: double c2;
    3: double c3;
    4: double c4;
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
    4:  Vector4 orientation;
    /** angular velocity of the segment frame */
    5:  Vector3 angularVelocity;
    /** angular acceleration of the segment frame */
    6:  Vector3 angularAcceleration;
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
    4:  Vector4 orientation;
}

/** Frame output from Xsens
 */
struct XsensFrame {
    /** absolute time in seconds */
    1: double time;
    /** segments data */
    2: list<XsensSegmentData> segmentsData;
    /** sensors data */
    3: list<XsensSensorData> sensorsData
}