namespace yarp human

/**
 * Representation of the IHumanDynamics interface
 */
struct HumanDynamics {
    1: list<string> jointNames;
    2: list<double> torques;
}
