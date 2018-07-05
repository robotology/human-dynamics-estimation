namespace yarp wearable.msg

// ======================
// Possible sensor states
// ======================

enum SensorStatus {
  ERROR,
  OK,
  CALIBRATING,
  DATA_OVERFLOW,
  TIMEOUT,
  UNKNOWN,
  WAITING_FOR_FIRST_READ,
}

// ========================
// Implemented sensor types
// ========================

enum SensorType{
  ACCELEROMETER,
  EMG_SENSOR,
  FORCE_3D_SENSOR,
  FORCE_TORQUE_6D_SENSOR,
  FREE_BODY_ACCELERATION_SENSOR,
  GYROSCOPE,
  MAGNETOMETER,
  ORIENTATION_SENSOR,
  POSE_SENSOR,
  POSITION_SENSOR,
  SKIN_SENSOR,
  TEMPERATURE_SENSOR,
  TORQUE_3D_SENSOR,
  VIRTUAL_LINK_KIN_SENSOR,
  VIRTUAL_SPHERICAL_JOINT_KIN_SENSOR,
}

// ======================
// Common sensor metadata
// ======================

struct SensorInfo{
  1: string name;
  2: SensorType type;
  3: SensorStatus status = SensorStatus.UNKNOWN;
}

// =====================
// Elementary data types
// =====================

struct VectorXYZ {
  1: double x;
  2: double y;
  3: double z;
}

struct VectorRPY {
  1: double r;
  2: double p;
  3: double y;
}

struct Quaternion {
  1: double w;
  2: VectorXYZ imaginary;
}

// ==========================
// Composed sensor data types
// ==========================

struct ForceTorque6DSensorData {
  1: VectorXYZ force,
  2: VectorXYZ torque
}

struct PoseSensorData {
  1: Quaternion orientation,
  2: VectorXYZ position
}

struct VirtualLinkKinSensorData {
  1: Quaternion orientation,
  2: VectorXYZ position,
  3: VectorXYZ linearVelocity,
  4: VectorXYZ angularVelocity,
  5: VectorXYZ linearAcceleration,
  6: VectorXYZ angularAcceleration
}

struct VirtualSphericalJointKinSensorData {
  1: VectorRPY angle,
  2: VectorXYZ velocity,
  3: VectorXYZ acceleration
}

// =================
// Sensor structures
// =================

struct Accelerometer {
  1: SensorInfo info,
  2: VectorXYZ data
}

struct EmgSensor {
  1: SensorInfo info,
  2: double data
}

struct FreeBodyAccelerationSensor {
  1: SensorInfo info,
  2: VectorXYZ data
}

struct Force3DSensor {
  1: SensorInfo info,
  2: VectorXYZ data
}

struct ForceTorque6DSensor {
  1: SensorInfo info,
  2: ForceTorque6DSensorData data
}

struct Gyroscope {
  1: SensorInfo info,
  2: VectorXYZ data
}

struct Magnetometer {
  1: SensorInfo info,
  2: VectorXYZ data
}

struct OrientationSensor {
  1: SensorInfo info,
  2: Quaternion data
}

struct PoseSensor {
  1: SensorInfo info,
  2: PoseSensorData data
}

struct PositionSensor {
  1: SensorInfo info,
  2: VectorXYZ data
}

struct SkinSensor {
  1: SensorInfo info,
  2: VectorXYZ data
}

struct TemperatureSensor {
  1: SensorInfo info,
  2: double data
}

struct Torque3DSensor {
  1: SensorInfo info,
  2: VectorXYZ data
}

struct VirtualLinkKinSensor {
  1: SensorInfo info,
  2: VirtualLinkKinSensorData data
}

struct VirtualSphericalJointKinSensor {
  1: SensorInfo info,
  2: VirtualSphericalJointKinSensorData data
}

// =========================
// Complete wear data struct
// =========================

struct WearData{
    1: required string producerName;
    2: optional list<Accelerometer> accelerometers;
    3: optional list<EmgSensor> emgSensors;
    4: optional list<Force3DSensor> force3DSensors;
    5: optional list<ForceTorque6DSensor> forceTorque6DSensors;
    6: optional list<FreeBodyAccelerationSensor> freeBodyAccelerationSensors,
    7: optional list<Gyroscope> gyroscopes,
    8: optional list<Magnetometer> magnetometers,
    9: optional list<OrientationSensor> orientationSensors,
    10: optional list<PoseSensor> poseSensors,
    11: optional list<PositionSensor> positionSensors,
    12: optional list<SkinSensor> skinSensors,
    13: optional list<TemperatureSensor> temperatureSensors,
    14: optional list<Torque3DSensor> torque3DSensors;
    15: optional list<VirtualLinkKinSensor> virtualLinkKinSensors,
    16: optional list<VirtualSphericalJointKinSensor> virtualSphericalJointKinSensors
}
