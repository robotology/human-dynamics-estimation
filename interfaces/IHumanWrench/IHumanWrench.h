/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_INTERFACES_IHUMANWRENCH
#define HDE_INTERFACES_IHUMANWRENCH

#include <string>
#include <vector>
#include <map>

namespace hde {
    namespace interfaces {
        class IHumanWrench;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IHumanWrench
{
public:
    virtual ~IHumanWrench() = default;

    enum class WrenchSourceType
    {
        Fixed,
        Robot,
        Dummy, // TODO
    };

    virtual std::map<std::string, WrenchSourceType> getWrenchSourceNameAndTypeMap() const = 0;
    virtual std::vector<std::string> getWrenchSourceNames() const = 0;
    virtual size_t getNumberOfWrenchSources() const = 0;

    virtual std::vector<double> getWrenches() const = 0;

};

#endif // HDE_INTERFACES_IHUMANWRENCH
