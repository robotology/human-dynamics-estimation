namespace yarp wearable.msg

// ==================
// Actuators metadata
// ==================

enum ActuatorType {
  HAPTIC,
  MOTOR,
  HEATER,
}

struct ActuatorInfo {
  1: string name;
  2: ActuatorType type;
}

// ==========================
// Actuator Command data type
// ==========================

struct WearableActuatorCommand {
  1: ActuatorInfo info;
  2: double value;
  3: double duration;
}
