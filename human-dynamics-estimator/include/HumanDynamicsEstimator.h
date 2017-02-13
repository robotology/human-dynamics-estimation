
//
//  HumanDynamicsEstimator.h
//  HumanDynamicsEstimator
//
//  Created by Claudia Latella on 03/02/17.
//  Copyright © 2017 Claudia Latella. All rights reserved.

#ifndef HUMANDYNAMICSESTIMATOR_H
#define HUMANDYNAMICSESTIMATOR_H

#include <yarp/os/RFModule.h>
#include <iDynTree/Estimation/BerdyHelper.h>


class HumanDynamicsEstimator:public yarp::os::RFModule {

private:
    std::string modelFilename;
    int count;
    
public:
    double getPeriod();
    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();
    bool interruptModule(); // interrupt, e.g., the ports
    bool close();

};


#endif /* end of include guard: HUMANDYNAMICSESTIMATOR_H */
