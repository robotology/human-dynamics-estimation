// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <pybind11/pybind11.h>

#include <hde/bindings/msgs/HumanState.h>
// #include <Wearable/bindings/msgs/WearableData.h>

namespace hde {
    namespace bindings {
        namespace msgs {

            void CreateModule(pybind11::module& module)
            {
                module.doc() = "YarpUtilities module.";

                CreateHumanState(module);
            }
        } // namespace msgs
    } // namespace bindings
} // namespace hde