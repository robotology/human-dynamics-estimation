// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_IMPL_IWRENCHFRAMETRANSFORMERS
#define HDE_DEVICES_IMPL_IWRENCHFRAMETRANSFORMERS

#include <iDynTree/Transform.h>
#include <iDynTree/Wrench.h>

#include <mutex>
#include <string>

namespace hde {
    namespace devices {
        namespace impl {
            class IWrenchFrameTransformer;
        } // namespace impl
    } // namespace devices
} // namespace hde

class hde::devices::impl::IWrenchFrameTransformer
{
public:
    iDynTree::Transform transform;

    IWrenchFrameTransformer() = default;
    ~IWrenchFrameTransformer() = default;
    bool transformWrenchFrame(const iDynTree::Wrench inputWrench,
                                      iDynTree::Wrench& transformedWrench);
};

#endif // HDE_DEVICES_IMPL_IWRENCHFRAMETRANSFORMERS
