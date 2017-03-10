namespace yarp human

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

/**
 * Information about generalized coordinates of human
 * The name of the object is for "compatibility" with YARP
 * even if this is not properly a state of a dynamic system.
 * \todo decide if put the base pose as explicit or not
 */
struct HumanState {
    1: Vector positions;
    2: Vector velocities;
    3: Vector accelerations;
}
