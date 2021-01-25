namespace yarp wearable.msg

// ==================
// Actuators metadata
// ==================

enum ActuatorStatus {
  ERROR,
  OK
  UNKNOWN,
}

enum ActuatorType {
  HAPTIC,
  MOTOR,
  HEATER,
}

struct ActuatorInfo {
  1: string name;
  2: ActuatorType type;
  3: ActuatorStatus status;
}

// =====================
// Actuators data struct
// ======================

//TODO: Update relevant data struct for each actuator,
//that needs to be communication to a data port

struct Haptic {
  1: ActuatorInfo info;
  2: double value;
}

struct Motor {
  1: ActuatorInfo info;
  2: double value;
}

struct Heater {
  1: ActuatorInfo info;
  2: double value;
}

// ============================
// WearableActuatorsData struct
// ============================

struct WearableActuatorsData {
1: required string producerName;
2: optional map<string, Haptic> hapticActuators;
3: optional map<string, Motor> motorActuators;
4: optional map<string, Heater> heaterActuators;
}

// ==========================
// Actuator Command data type
// ==========================

struct WearableActuatorCommand {
  1: ActuatorInfo info;
  2: double value;
  3: double duration;
}
