// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_BINDINGS_MSGS_BUFFERED_PORT_H
#define HDE_BINDINGS_MSGS_BUFFERED_PORT_H

#include <string>

#include <pybind11/detail/common.h>
#include <pybind11/pybind11.h>

#include <yarp/os/BufferedPort.h>
// #include "/home/ittbmp014lw003/robotology-superbuild/build/install/include/yarp/os/BufferedPort.h"

namespace hde {
    namespace bindings {
        namespace msgs {

            template <typename T>
            void CreateBufferedPort(pybind11::module& module, const std::string& name)
            {
                namespace py = ::pybind11;
                py::class_<::yarp::os::BufferedPort<T>>(module, name.c_str())
                    .def(py::init())
                    .def("open",
                         py::overload_cast<const std::string&>(&::yarp::os::BufferedPort<T>::open),
                         py::arg("name"))
                    .def("close", &::yarp::os::BufferedPort<T>::close)
                    .def("isClosed", &::yarp::os::BufferedPort<T>::isClosed)
                    .def("prepare",
                         &::yarp::os::BufferedPort<T>::prepare,
                         py::return_value_policy::reference_internal)
                    .def("write",
                         &::yarp::os::BufferedPort<T>::write,
                         py::arg("forceStrict") = false)
                    .def("read",
                         &::yarp::os::BufferedPort<T>::read,
                         py::arg("shouldWait") = true,
                         py::return_value_policy::reference_internal);
            }
        } // namespace msgs
    } // namespace bindings
} // namespace hde

#endif // HDE_BINDINGS_MSGS_BUFFERED_PORT_H