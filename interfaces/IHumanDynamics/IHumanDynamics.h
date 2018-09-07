/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_INTERFACES_IHUMANDYNAMICS
#define HDE_INTERFACES_IHUMANDYNAMICS

namespace hde {
    namespace interfaces {
        class IHumanDynamics;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IHumanDynamics
{
public:
    virtual ~IHumanDynamics() = default;
};

#endif // HDE_INTERFACES_IHUMANDYNAMICS
