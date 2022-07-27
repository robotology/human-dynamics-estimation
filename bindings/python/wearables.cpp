/**
 * @file wearables.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <Wearable/bindings/msgs/Module.h>

// Create the Python module
PYBIND11_MODULE(bindings, m)
{
    namespace py = ::pybind11;

    m.doc() = "wearables bindings";

    py::module msgModule = m.def_submodule("msg");
    wearables::bindings::msgs::CreateModule(msgModule);
}
