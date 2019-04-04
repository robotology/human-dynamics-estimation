namespace yarp wearable.msg

// ================
// Sensors metadata
// ================

enum SensorStatus {
  ERROR,
  OK,
  CALIBRATING,
  DATA_OVERFLOW,
  TIMEOUT,
  UNKNOWN,
  WAITING_FOR_FIRST_READ,
}

struct SensorInfo{
  1: string name;
  2: SensorStatus status = SensorStatus.UNKNOWN;
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

struct QuaternionWXYZ {
  1: double w;
  2: double x;
  3: double y;
  4: double z;
}

// ==========================
// Composed sensor data types
// ==========================

struct ForceTorque6DSensorData {
  1: VectorXYZ force;
  2: VectorXYZ torque;
}

struct PoseSensorData {
  1: QuaternionWXYZ orientation;
  2: VectorXYZ position;
}

struct VirtualLinkKinSensorData {
  1: QuaternionWXYZ orientation;
  2: VectorXYZ position;
  3: VectorXYZ linearVelocity;
  4: VectorXYZ angularVelocity;
  5: VectorXYZ linearAcceleration;
  6: VectorXYZ angularAcceleration;
}

struct VirtualJointKinSensorData {
  1: double position;
  2: double velocity;
  3: double acceleration;
}

struct VirtualSphericalJointKinSensorData {
  1: VectorRPY angle;
  2: VectorXYZ velocity;
  3: VectorXYZ acceleration;
}

struct EmgData {
  1: double value;
  2: double normalization;
}

// =================
// Sensor structures
// =================

struct Accelerometer {
  1: SensorInfo info;
  2: VectorXYZ data;
}

struct EmgSensor {
  1: SensorInfo info;
  2: EmgData data;
}

struct FreeBodyAccelerationSensor {
  1: SensorInfo info;
  2: VectorXYZ data;
}

struct Force3DSensor {
  1: SensorInfo info;
  2: VectorXYZ data;
}

struct ForceTorque6DSensor {
  1: SensorInfo info;
  2: ForceTorque6DSensorData data;
}

struct Gyroscope {
  1: SensorInfo info;
  2: VectorXYZ data;
}

struct Magnetometer {
  1: SensorInfo info;
  2: VectorXYZ data;
}

struct OrientationSensor {
  1: SensorInfo info;
  2: QuaternionWXYZ data;
}

struct PoseSensor {
  1: SensorInfo info;
  2: PoseSensorData data;
}

struct PositionSensor {
  1: SensorInfo info;
  2: VectorXYZ data;
}

struct SkinSensor {
  1: SensorInfo info;
  2: VectorXYZ data;
}

struct TemperatureSensor {
  1: SensorInfo info;
  2: double data;
}

struct Torque3DSensor {
  1: SensorInfo info;
  2: VectorXYZ data;
}

struct VirtualLinkKinSensor {
  1: SensorInfo info;
  2: VirtualLinkKinSensorData data;
}

struct VirtualJointKinSensor {
  1: SensorInfo info;
  2: VirtualJointKinSensorData data;
}

struct VirtualSphericalJointKinSensor {
  1: SensorInfo info;
  2: VirtualSphericalJointKinSensorData data;
}

// ========================
// Complete WearData struct
// ========================

struct WearableData {
1: required string producerName;
2: optional map<string,Accelerometer> accelerometers;
3: optional map<string,EmgSensor> emgSensors;
4: optional map<string,Force3DSensor> force3DSensors;
5: optional map<string,ForceTorque6DSensor> forceTorque6DSensors;
6: optional map<string,FreeBodyAccelerationSensor> freeBodyAccelerationSensors;
7: optional map<string,Gyroscope> gyroscopes;
8: optional map<string,Magnetometer> magnetometers;
9: optional map<string,OrientationSensor> orientationSensors;
10: optional map<string,PoseSensor> poseSensors;
11: optional map<string,PositionSensor> positionSensors;
12: optional map<string,SkinSensor> skinSensors;
13: optional map<string,TemperatureSensor> temperatureSensors;
14: optional map<string,Torque3DSensor> torque3DSensors;
15: optional map<string,VirtualLinkKinSensor> virtualLinkKinSensors;
16: optional map<string,VirtualJointKinSensor> virtualJointKinSensors;
17: optional map<string,VirtualSphericalJointKinSensor> virtualSphericalJointKinSensors;
}

// ============
// RPC metadata
// ============

struct WearableSensorMetadata {
    1: string name;
}

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
  VIRTUAL_JOINT_KIN_SENSOR,
  VIRTUAL_SPHERICAL_JOINT_KIN_SENSOR,
}

service WearableMetadataService {
  map<SensorType,list<WearableSensorMetadata>> getMetadata();
}
