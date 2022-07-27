/**
 * @file WearableData.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include "thrift/Accelerometer.h"
#include "thrift/EmgData.h"
#include "thrift/QuaternionWXYZ.h"
#include "thrift/SensorInfo.h"
#include "thrift/VectorXYZ.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <thrift/WearableData.h>

#include <Wearable/bindings/msgs/BufferedPort.h>
#include <Wearable/bindings/msgs/WearableData.h>

namespace wearables {
    namespace bindings {
        namespace msgs {

            void CreateSensorStatus(pybind11::module& module)
            {
                namespace py = ::pybind11;

                py::enum_<::wearable::msg::SensorStatus>(module, "SensorStatus")
                    .value("ERROR", ::wearable::msg::ERROR)
                    .value("OK", ::wearable::msg::OK)
                    .value("CALIBRATING", ::wearable::msg::CALIBRATING)
                    .value("DATA_OVERFLOW", ::wearable::msg::DATA_OVERFLOW)
                    .value("TIMEOUT", ::wearable::msg::TIMEOUT)
                    .value("UNKNOWN", ::wearable::msg::UNKNOWN)
                    .value("WAITING_FOR_FIRST_READ", ::wearable::msg::WAITING_FOR_FIRST_READ)
                    .export_values();
            }

            void CreateSensorInfo(pybind11::module& module)
            {
                namespace py = ::pybind11;

                py::class_<::wearable::msg::SensorInfo>(module, "SensorInfo")
                                        .def(py::init())
                    .def_readwrite("name", &::wearable::msg::SensorInfo::name)
                    .def_readwrite("status", &::wearable::msg::SensorInfo::status);
            }

            void CreateVectors(pybind11::module& module)
            {
                namespace py = ::pybind11;

                py::class_<::wearable::msg::VectorXYZ>(module, "VectorXYZ")
                                        .def(py::init())
                    .def_readwrite("x", &::wearable::msg::VectorXYZ::x)
                    .def_readwrite("y", &::wearable::msg::VectorXYZ::y)
                    .def_readwrite("z", &::wearable::msg::VectorXYZ::z);

                py::class_<::wearable::msg::VectorRPY>(module, "VectorRPY")
                                        .def(py::init())
                    .def_readwrite("r", &::wearable::msg::VectorRPY::r)
                    .def_readwrite("p", &::wearable::msg::VectorRPY::p)
                    .def_readwrite("y", &::wearable::msg::VectorRPY::y);

                py::class_<::wearable::msg::QuaternionWXYZ>(module, "QuaternonWXYZ")
                                        .def(py::init())
                    .def_readwrite("w", &::wearable::msg::QuaternionWXYZ::w)
                    .def_readwrite("x", &::wearable::msg::QuaternionWXYZ::x)
                    .def_readwrite("y", &::wearable::msg::QuaternionWXYZ::y)
                    .def_readwrite("z", &::wearable::msg::QuaternionWXYZ::z);
            }

            void CreateSensorData(pybind11::module& module)
            {
                namespace py = ::pybind11;

                py::class_<::wearable::msg::ForceTorque6DSensorData>(module,
                                                                     "ForceTorque6DSensorData")
                    .def(py::init())
                    .def_readwrite("force", &::wearable::msg::ForceTorque6DSensorData::force)
                    .def_readwrite("torque", &::wearable::msg::ForceTorque6DSensorData::torque);

                py::class_<::wearable::msg::PoseSensorData>(module, "PoseSensorData")
                    .def(py::init())
                    .def_readwrite("orientation", &::wearable::msg::PoseSensorData::orientation)
                    .def_readwrite("position", &::wearable::msg::PoseSensorData::position);

                py::class_<::wearable::msg::VirtualLinkKinSensorData>(module,
                                                                      "VirtualLinkKinSensorData")
                                        .def(py::init())
                    .def_readwrite("orientation",
                                   &::wearable::msg::VirtualLinkKinSensorData::orientation)
                    .def_readwrite("position", &::wearable::msg::VirtualLinkKinSensorData::position)
                    .def_readwrite("linearVelocity",
                                   &::wearable::msg::VirtualLinkKinSensorData::linearVelocity)
                    .def_readwrite("angularVelocity",
                                   &::wearable::msg::VirtualLinkKinSensorData::angularVelocity)
                    .def_readwrite("linearAcceleration",
                                   &::wearable::msg::VirtualLinkKinSensorData::linearAcceleration)
                    .def_readwrite("angularAcceleration",
                                   &::wearable::msg::VirtualLinkKinSensorData::angularAcceleration);

                py::class_<::wearable::msg::VirtualJointKinSensorData>(module,
                                                                       "VirtualJointKinSensorData")
                                        .def(py::init())
                    .def_readwrite("acceleration",
                                   &::wearable::msg::VirtualJointKinSensorData::acceleration)
                    .def_readwrite("velocity",
                                   &::wearable::msg::VirtualJointKinSensorData::velocity)
                    .def_readwrite("position",
                                   &::wearable::msg::VirtualJointKinSensorData::position);

                py::class_<::wearable::msg::VirtualSphericalJointKinSensorData>(
                    module, "VirtualSphericalJointKinSensorData")
                                        .def(py::init())
                    .def_readwrite(
                        "acceleration",
                        &::wearable::msg::VirtualSphericalJointKinSensorData::acceleration)
                    .def_readwrite("velocity",
                                   &::wearable::msg::VirtualSphericalJointKinSensorData::velocity)
                    .def_readwrite("angle",
                                   &::wearable::msg::VirtualSphericalJointKinSensorData::angle);

                py::class_<::wearable::msg::EmgData>(module, "EmgData")
                                        .def(py::init())
                    .def_readwrite("value", &::wearable::msg::EmgData::value)
                    .def_readwrite("normalization", &::wearable::msg::EmgData::normalization);
            }

            template <typename Sensor>
            void CreateSensorStructure(pybind11::module& module, const std::string& name)
            {
                namespace py = ::pybind11;

                py::class_<Sensor>(module, name.c_str())
                                        .def(py::init())
                    .def_readwrite("info", &Sensor::info)
                    .def_readwrite("data", &Sensor::data)
                    .def("__str__", &Sensor::toString)
                    .def("toString", &Sensor::toString);
            }

            void CreateSensorsStructure(pybind11::module& module)
            {
                using namespace ::wearable::msg;
                CreateSensorStructure<Accelerometer>(module, "Accelerometer");
                CreateSensorStructure<EmgSensor>(module, "EmgSensor");
                CreateSensorStructure<FreeBodyAccelerationSensor>(module,
                                                                  "FreeBodyAccelerationSensor");
                CreateSensorStructure<Force3DSensor>(module, "Force3DSensor");
                CreateSensorStructure<ForceTorque6DSensor>(module, "ForceTorque6DSensor");
                CreateSensorStructure<Gyroscope>(module, "Gyroscope");
                CreateSensorStructure<Magnetometer>(module, "Magnetometer");
                CreateSensorStructure<OrientationSensor>(module, "OrientationSensor");
                CreateSensorStructure<PoseSensor>(module, "PoseSensor");
                CreateSensorStructure<PositionSensor>(module, "PositionSensor");
                CreateSensorStructure<SkinSensor>(module, "SkinSensor");
                CreateSensorStructure<TemperatureSensor>(module, "TemperatureSensor");
                CreateSensorStructure<Torque3DSensor>(module, "Torque3DSensor");
                CreateSensorStructure<VirtualLinkKinSensor>(module, "VirtualLinkKinSensor");
                CreateSensorStructure<VirtualJointKinSensor>(module, "VirtualJointKinSensor");
                CreateSensorStructure<VirtualSphericalJointKinSensor>(
                    module, "VirtualSphericalJointKinSensor");
            }

            void CreateWearableData(pybind11::module& module)
            {
                namespace py = ::pybind11;
                using namespace ::wearable::msg;

                CreateSensorStatus(module);
                CreateSensorInfo(module);
                CreateVectors(module);
                CreateSensorData(module);
                CreateSensorsStructure(module);

                py::class_<WearableData>(module, "WearableData")
                    .def(py::init())
                    .def_readwrite("producerName", &WearableData::producerName)
                    .def_readwrite("accelerometers", &WearableData::accelerometers)
                    .def_readwrite("emgSensors", &WearableData::emgSensors)
                    .def_readwrite("force3DSensors", &WearableData::force3DSensors)
                    .def_readwrite("forceTorque6DSensors", &WearableData::forceTorque6DSensors)
                    .def_readwrite("freeBodyAccelerationSensors", &WearableData::freeBodyAccelerationSensors)
                    .def_readwrite("gyroscopes", &WearableData::gyroscopes)
                    .def_readwrite("magnetometers", &WearableData::magnetometers)
                    .def_readwrite("orientationSensors", &WearableData::orientationSensors)
                    .def_readwrite("poseSensors", &WearableData::poseSensors)
                    .def_readwrite("positionSensors", &WearableData::positionSensors)
                    .def_readwrite("skinSensors", &WearableData::skinSensors)
                    .def_readwrite("temperatureSensors", &WearableData::temperatureSensors)
                    .def_readwrite("torque3DSensors", &WearableData::torque3DSensors)
                    .def_readwrite("virtualLinkKinSensors", &WearableData::virtualLinkKinSensors)
                    .def_readwrite("virtualJointKinSensors", &WearableData::virtualJointKinSensors)
                    .def_readwrite("virtualSphericalJointKinSensors", &WearableData::virtualSphericalJointKinSensors)
                    .def("__str__", &WearableData::toString)
                    .def("toString", &WearableData::toString);

                CreateBufferedPort<WearableData>(module, "BufferedPortWearableData");
            }
        } // namespace msgs
    } // namespace bindings
} // namespace wearables
