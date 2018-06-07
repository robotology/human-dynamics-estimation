/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "FTShoes.h"

using namespace wear::suit;

class FTShoes::Impl
{};

FTShoes::FTShoes()
    : pImpl{new Impl()}
{}

// Without this destructor here, the linker complains for
// undefined reference to vtable
FTShoes::~FTShoes() = default;
