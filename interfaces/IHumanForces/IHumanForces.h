/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_INTERFACES_IHUMANFORCES
#define HDE_INTERFACES_IHUMANFORCES

//#include <array>
//#include <string>
//#include <vector>

namespace hde {
    namespace interfaces {
        class IHumanForces;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IHumanForces
{
public:
    virtual ~IHumanState() = default;
};

#endif // HDE_INTERFACES_IHUMANFORCES
