//
//  HumanStateProviderPrivate.h
//  human-state-provider
//
//  Created by Francesco Romano on 20/02/17.
//  Copyright © 2017 Francesco Romano. All rights reserved.
//

#ifndef HUMANSTATEPROVIDERPRIVATE_H
#define HUMANSTATEPROVIDERPRIVATE_H

#include "HumanStateProvider.h"

#include "HumanIKWorkerPool.h"

#include <thrifts/HumanState.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <vector>


namespace human {
    struct IKSolverData;
    class HumanStateProvider;
    class HumanIKWorkerPool;
}

namespace yarp {
    namespace dev {
        class IHumanSkeleton;
    }
}

class InverseKinematics;

struct human::IKSolverData {
    InverseKinematics* solver;
    iDynTree::VectorDynSize solution;
    std::pair<std::string, std::string> frameNames;
    std::pair<unsigned, unsigned> frameIndeces;
    std::vector<std::pair<unsigned, unsigned> > consideredJointLocations;
};

class human::HumanStateProvider::HumanStateProviderPrivate {
public:
    double m_period;
    yarp::os::BufferedPort<human::HumanState> m_outputPort;
    yarp::dev::PolyDriver m_humanDriver;
    yarp::dev::IHumanSkeleton *m_human;

    std::vector<IKSolverData> m_solvers;

    struct {
        std::vector<yarp::sig::Vector> poses;
        std::vector<yarp::sig::Vector> velocities;
        std::vector<yarp::sig::Vector> accelerations;
        std::vector<iDynTree::Transform> linkPoseWRTWorld;

        iDynTree::VectorDynSize jointsConfiguration;
    } m_buffers;

    human::HumanIKWorkerPool* m_ikPool;

    HumanStateProviderPrivate();
};


#endif /* end of include guard: HUMANSTATEPROVIDERPRIVATE_H */
