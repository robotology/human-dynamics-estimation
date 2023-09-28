namespace yarp hde.msgs

/**
 * Representation of the IHumanWrench interface
 */
struct HumanWrench {
    1: list<string> wrenchSourceNames;
    2: list<double> wrenches;
}
