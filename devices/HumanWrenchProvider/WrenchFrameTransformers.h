/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_IMPL_IWRENCHFRAMETRANSFORMERS
#define HDE_DEVICES_IMPL_IWRENCHFRAMETRANSFORMERS

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Wrench.h>

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
