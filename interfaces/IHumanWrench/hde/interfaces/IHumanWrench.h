// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_INTERFACES_IHUMANWRENCH
#define HDE_INTERFACES_IHUMANWRENCH

#include <string>
#include <vector>

namespace hde {
    namespace interfaces {
        class IHumanWrench;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IHumanWrench
{
public:
    virtual ~IHumanWrench() = default;

    virtual std::vector<std::string> getWrenchSourceNames() const = 0;
    virtual size_t getNumberOfWrenchSources() const = 0;

    virtual std::vector<double> getWrenches() const = 0;

};

#endif // HDE_INTERFACES_IHUMANWRENCH
