// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <hde/msgs/HumanState.h>
// #include <thrift/WearableData.h>

#include <hde/bindings/msgs/BufferedPort.h>
#include <hde/bindings/msgs/HumanState.h>
// #include <Wearable/bindings/msgs/BufferedPort.h>
// #include <Wearable/bindings/msgs/WearableData.h>

namespace hde {
    namespace bindings {
        namespace msgs {

            void CreateHumanState(pybind11::module& module)
            {
                namespace py = ::pybind11;
                using namespace ::hde::msgs;

                py::class_<HumanState>(module, "HumanState")
                    .def(py::init())
                    .def_readwrite("jointNames", &HumanState::jointNames)
                    .def_readwrite("positions", &HumanState::positions);

                CreateBufferedPort<HumanState>(module, "BufferedPortHumanState");
            }
        } // namespace msgs
    } // namespace bindings
} // namespace hde