namespace yarp human

/**
 * Information about generalized coordinates of human
 * The name of the object is for "compatibility" with YARP
 * even if this is not properly a state of a dynamic system.
 * \todo decide if put the base pose as explicit or not
 */
struct HumanState {
    1: list<double> positions;
    2: list<double> velocities;
    3: list<double> accelerations;
}

