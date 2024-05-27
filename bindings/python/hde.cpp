// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <pybind11/pybind11.h>

#include <hde/bindings/msgs/Module.h>

// Create the Python module
PYBIND11_MODULE(bindings, m)
{
    namespace py = ::pybind11;

    m.doc() = "Human dynamics estimation bindings";

    py::module msgModule = m.def_submodule("msg");
    hde::bindings::msgs::CreateModule(msgModule);
}