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

#include <thrifts/HumanState.h>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/KinDynComputations.h>
#include <inversekinematics/InverseKinematics.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <Eigen/QR>

#include <string>
#include <vector>
#include <memory>

namespace human {
    struct IKSolverData;
    struct SegmentInfo;
    struct LinkPairInfo;

    class HumanStateProvider;
    class HumanIKWorkerPool;

}

namespace yarp {
    namespace experimental {
        namespace dev {
            class IFrameProvider;
            class IXsensMVNInterface;
        }
    }
}

namespace iDynTree {
    class KinDynComputations;
}

class InverseKinematics;

/*!
 * Relevant information on the segment input
 * This saves the information coming from a FrameProvider
 */
struct human::SegmentInfo {
    iDynTree::Transform poseWRTWorld;
    iDynTree::VectorDynSize velocities;
    //TODO if not needed acceleration delete them
    yarp::sig::Vector accelerations;
};

/*!
 * Relevant information on the submodel between two links (segments)
 * Needed to compute inverse kinematics, velocities, etc
 */
struct human::LinkPairInfo {
    //variables representing the DoFs between the two frames
    iDynTree::VectorDynSize jointConfigurations;
    iDynTree::VectorDynSize jointVelocities;

    //IK elements (i.e. compute joints)
    std::unique_ptr<InverseKinematics> ikSolver;

    //Velocity-related elements
    iDynTree::MatrixDynSize parentJacobian;
    iDynTree::MatrixDynSize childJacobian;
    iDynTree::MatrixDynSize relativeJacobian;
    std::unique_ptr<iDynTree::KinDynComputations> kinDynComputations;
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > jacobianDecomposition;

    //Mapping from link pair to full model. Needed to map from small to complete problem
    std::string parentFrameName; //name of the parent frame
    unsigned parentFrameModelIndex; //index of the frame in the iDynTree Model
    unsigned parentFrameSegmentsIndex; //index of the parent frame in the segment list

    std::string childFrameName; //name of the child frame
    unsigned childFrameModelIndex; //index of the frame in the iDynTree Model
    unsigned childFrameSegmentsIndex; //index of the child frame in the segment list

    std::vector<std::pair<unsigned, unsigned> > consideredJointLocations; /*!< For each joint connecting the pair: first = offset in the full model , second = dofs of joint */
};



class human::HumanStateProvider::HumanStateProviderPrivate {
public:
    double m_period;
    yarp::os::BufferedPort<human::HumanState> m_outputPort;
    yarp::dev::PolyDriver m_humanDriver;
    yarp::experimental::dev::IFrameProvider* m_frameProvider;

    std::vector<human::LinkPairInfo> m_linkPairs;
    std::vector<human::SegmentInfo> m_segments;

    struct {
        //These vectors have the size of # of segments
        //Needed for reading inputs from the IFrameProviders object
        std::vector<yarp::sig::Vector> poses;
        std::vector<yarp::sig::Vector> velocities;
        std::vector<yarp::sig::Vector> accelerations;

        //Size of # Dofs
        //Needed for output
        yarp::sig::Vector jointsConfiguration;
        yarp::sig::Vector jointsVelocity;
    } m_buffers;

    std::unique_ptr<human::HumanIKWorkerPool> m_ikPool; //this should be transformed into unique_ptr, but it needs the include. Check if this creates a look or not

    HumanStateProviderPrivate();
    ~HumanStateProviderPrivate();
};


#endif /* end of include guard: HUMANSTATEPROVIDERPRIVATE_H */
