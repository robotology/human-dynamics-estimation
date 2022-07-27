/**
 * @file VectorsCollection.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef WEARABLES_BINDINGS_MSGS_WEARABLE_DATA_H
#define WEARABLES_BINDINGS_MSGS_WEARABLE_DATA_H

#include <pybind11/pybind11.h>

namespace wearables {
    namespace bindings {
        namespace msgs {

            void CreateWearableData(pybind11::module& module);

        } // namespace msgs
    } // namespace bindings
} // namespace wearables

#endif // WEARABLES_BINDINGS_MSGS_WEARABLE_DATA_H
