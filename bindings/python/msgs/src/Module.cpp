/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <Wearable/bindings/msgs/WearableData.h>

namespace wearables {
    namespace bindings {
        namespace msgs {

            void CreateModule(pybind11::module& module)
            {
                module.doc() = "YarpUtilities module.";

                CreateWearableData(module);
            }
        } // namespace msgs
    } // namespace bindings
} // namespace wearables
