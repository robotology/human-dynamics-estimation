// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_BINDINGS_MSGS_MODULE_H
#define HDE_BINDINGS_MSGS_MODULE_H

#include <pybind11/pybind11.h>

namespace hde {
    namespace bindings {
        namespace msgs {

            void CreateModule(pybind11::module& module);

        } // namespace msgs
    } // namespace bindings
} // namespace hde

#endif // HDE_BINDINGS_MSGS_MODULE_H