// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <WrenchFrameTransformers.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Link.h>

using namespace hde::devices::impl;

bool IWrenchFrameTransformer::transformWrenchFrame(const iDynTree::Wrench inputWrench,
                                                   iDynTree::Wrench& transformedWrench)
{
    transformedWrench = transform * inputWrench;
    return true;
}
