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
    : m_frameProvider(nullptr) {}

    HumanStateProvider::HumanStateProviderPrivate::~HumanStateProviderPrivate() {}

    std::vector<std::string> HumanStateProvider::HumanStateProviderPrivate::joints()
    {
        return m_humanJointNames;
    }

    std::string HumanStateProvider::HumanStateProviderPrivate::baseLink()
    {
        std::lock_guard<std::mutex> guard(m_objectMutex);
        if (m_baseLink >= m_segments.size()) return "";
        return m_segments[m_baseLink].segmentName;
    }

    bool HumanStateProvider::HumanStateProviderPrivate::setBaseLink(const std::string &baseLink)
    {
        std::lock_guard<std::mutex> guard(m_objectMutex);
        for (size_t index = 0; index < m_segments.size(); ++index) {
            if (m_segments[index].segmentName == baseLink) {
                m_baseLink = index;
                return true;
            }
        }
        return false;
    }

}
