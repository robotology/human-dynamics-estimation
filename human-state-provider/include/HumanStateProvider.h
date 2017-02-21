//
//  HumanStateProvider.h
//  human-state-provider
//
//  Created by Francesco Romano on 20/02/17.
//  Copyright © 2017 Francesco Romano. All rights reserved.
//

#ifndef HUMANSTATEPROVIDER_H
#define HUMANSTATEPROVIDER_H

#include <yarp/os/RFModule.h>

namespace human {
    class HumanStateProvider;
}

/**
 * This module is responsible of collecting information from 
 * the Xsens suit and provide the configuration, velocity
 * and acceleration of each generalized coordinate composing
 * the human model.
 */
class human::HumanStateProvider : public yarp::os::RFModule
{
    class HumanStateProviderPrivate;
    HumanStateProviderPrivate* m_pimpl;

public:

    HumanStateProvider();
    virtual ~HumanStateProvider();

    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();

};


#endif /* end of include guard: HUMANSTATEPROVIDER_H */
