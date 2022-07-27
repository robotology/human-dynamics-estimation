/**
 * @file BufferedPort.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef WEARABLES_BINDINGS_MSGS_BUFFERED_PORT_H
#define WEARABLES_BINDINGS_MSGS_BUFFERED_PORT_H

#include <string>

#include <pybind11/detail/common.h>
#include <pybind11/pybind11.h>

#include <yarp/os/BufferedPort.h>

namespace wearables {
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
} // namespace wearables

#endif // WEARABLES_BINDINGS_MSGS_BUFFERED_PORT_H
