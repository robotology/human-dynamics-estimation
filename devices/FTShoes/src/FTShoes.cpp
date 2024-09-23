// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
