// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WEARABLES_BINDINGS_MSGS_MODULE_H
#define WEARABLES_BINDINGS_MSGS_MODULE_H

#include <pybind11/pybind11.h>

namespace wearables {
    namespace bindings {
        namespace msgs {

            void CreateModule(pybind11::module& module);

        } // namespace msgs
    } // namespace bindings
} // namespace wearables

#endif // BIPEDAL_LOCOMOTION_BINDINGS_YARP_UTILITES_MODULE_H
