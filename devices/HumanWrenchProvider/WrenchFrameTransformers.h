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
            class FixedFrameWrenchTransformer;
            class RobotFrameWrenchTransformer;
            class WorldWrenchTransformer;
        } // namespace impl
    } // namespace devices
} // namespace hde

class hde::devices::impl::IWrenchFrameTransformer
{
public:
    virtual ~IWrenchFrameTransformer() = default;
    virtual bool transformWrenchFrame(const iDynTree::Wrench inputWrench,
                                      iDynTree::Wrench& transformedWrench) = 0;
};

class hde::devices::impl::FixedFrameWrenchTransformer final
    : public hde::devices::impl::IWrenchFrameTransformer
{
public:
    std::string originExpressedFrame;
    std::string transformedExpressedFrame;
    iDynTree::Transform transform;

    FixedFrameWrenchTransformer() = default;
    ~FixedFrameWrenchTransformer() override = default;

    bool transformWrenchFrame(const iDynTree::Wrench inputWrench,
                              iDynTree::Wrench& transformedWrench) override;
};

class hde::devices::impl::RobotFrameWrenchTransformer final
    : public hde::devices::impl::IWrenchFrameTransformer
{
public:
    iDynTree::Transform transform = iDynTree::Transform::Identity();
    iDynTree::Transform fixedTransform;
    std::mutex _mutex;

    RobotFrameWrenchTransformer() = default;
    ~RobotFrameWrenchTransformer() override = default;

    bool transformWrenchFrame(const iDynTree::Wrench inputWrench,
                              iDynTree::Wrench& transformedWrench) override;
};

class hde::devices::impl::WorldWrenchTransformer final
    : public hde::devices::impl::IWrenchFrameTransformer
{
public:
    iDynTree::Transform transform;
    iDynTree::Wrench wrench;
    std::mutex _mutex;

    WorldWrenchTransformer() = default;
    ~WorldWrenchTransformer() override = default;

    bool transformWrenchFrame(const iDynTree::Wrench inputWrench,
                              iDynTree::Wrench& transformedWrench) override;
};

#endif // HDE_DEVICES_IMPL_IWRENCHFRAMETRANSFORMERS
