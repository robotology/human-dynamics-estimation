// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
  2: list<double> forceValue;
  3: list<double> vibroTactileValue;
  4: double motorValue;
}
