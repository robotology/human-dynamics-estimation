//
//  HumanStateProviderPrivate.cpp
//  human-state-provider
//
//  Created by Francesco Romano on 20/02/17.
//  Copyright © 2017 Francesco Romano. All rights reserved.
//

#include "HumanStateProviderPrivate.h"
//The following is needed by the unique_ptr as it needs to know the delete function
//And at this point without the include it only knows the forward declaration
#include "HumanIKWorkerPool.h"

namespace human {
    HumanStateProvider::HumanStateProviderPrivate::HumanStateProviderPrivate()
    : m_frameProvider(0) {}

    HumanStateProvider::HumanStateProviderPrivate::~HumanStateProviderPrivate() {}
}
