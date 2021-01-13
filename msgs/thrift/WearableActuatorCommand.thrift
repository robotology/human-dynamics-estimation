namespace yarp wearable.msg

// ==================
// Actuators metadata
// ==================

enum ActuatorType {
  HAPTIC,
  MOTOR,
  HEATER,
  UNKNOWN,
}

// ==========================
// Actuator Command data type
// ==========================

struct WearableActuatorCommand {
  1: ActuatorType type = ActuatorType.UNKNOWN;
  2: double value;
  3: double duration;
}
