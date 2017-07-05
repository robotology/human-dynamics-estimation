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
#include <thrifts/HumanStateProviderService.h>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Model/Indeces.h>
#include <iDynTree/KinDynComputations.h>
#include <human-ik/InverseKinematics.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

#include <Eigen/QR>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace human {
    struct IKSolverData;
    struct SegmentInfo;
    struct LinkPairInfo;

    class HumanStateProvider;
    class HumanIKWorkerPool;
    class InverseKinematics;

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

/*!
 * Relevant information on the segment input
 * This saves the information coming from a FrameProvider
 */
struct human::SegmentInfo {
    std::string segmentName;

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
    std::unique_ptr<human::InverseKinematics> ikSolver;

    //Velocity-related elements
    iDynTree::MatrixDynSize parentJacobian;
    iDynTree::MatrixDynSize childJacobian;
    iDynTree::MatrixDynSize relativeJacobian;
    std::unique_ptr<iDynTree::KinDynComputations> kinDynComputations;
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > jacobianDecomposition;

    //Mapping from link pair to full model. Needed to map from small to complete problem
    std::string parentFrameName; //name of the parent frame
    iDynTree::FrameIndex parentFrameModelIndex; //index of the frame in the iDynTree Model
    iDynTree::FrameIndex parentFrameSegmentsIndex; //index of the parent frame in the segment list

    std::string childFrameName; //name of the child frame
    iDynTree::FrameIndex childFrameModelIndex; //index of the frame in the iDynTree Model
    iDynTree::FrameIndex childFrameSegmentsIndex; //index of the child frame in the segment list

    std::vector<std::pair<size_t, size_t> > consideredJointLocations; /*!< For each joint connecting the pair: first = offset in the full model , second = dofs of joint */

    LinkPairInfo() = default;
#if defined(_MSC_VER) && _MSC_VER < 1900
    LinkPairInfo(LinkPairInfo&& rvalue)
    : jointConfigurations(std::move(rvalue.jointConfigurations))
    , jointVelocities(std::move(rvalue.jointVelocities))
    , ikSolver(std::move(rvalue.ikSolver))
    , parentJacobian(std::move(rvalue.parentJacobian))
    , childJacobian(std::move(rvalue.childJacobian))
    , relativeJacobian(std::move(rvalue.relativeJacobian))
    , kinDynComputations(std::move(rvalue.kinDynComputations))
    , jacobianDecomposition(std::move(rvalue.jacobianDecomposition))
    , parentFrameName(std::move(rvalue.parentFrameName))
    , parentFrameModelIndex(rvalue.parentFrameModelIndex)
    , parentFrameSegmentsIndex(rvalue.parentFrameSegmentsIndex)
    , childFrameName(std::move(rvalue.childFrameName))
    , childFrameModelIndex(rvalue.childFrameModelIndex)
    , childFrameSegmentsIndex(rvalue.childFrameSegmentsIndex)
    , consideredJointLocations(std::move(rvalue.consideredJointLocations))
    {}
#else
    LinkPairInfo(LinkPairInfo&&) = default;
#endif
    LinkPairInfo(const LinkPairInfo&) = delete;

};



class human::HumanStateProvider::HumanStateProviderPrivate
: public human::HumanStateProviderService
{
public:
    double m_period;

    yarp::os::BufferedPort<human::HumanState> m_outputPort;
    yarp::os::Port m_rpcPort;
    std::vector<std::string> m_humanJointNames; //cached list of joint names

    yarp::dev::PolyDriver m_humanDriver;
    yarp::experimental::dev::IFrameProvider* m_frameProvider;
    yarp::dev::IPreciselyTimed* m_frameProviderTimed;

    std::vector<human::LinkPairInfo> m_linkPairs;
    std::vector<human::SegmentInfo> m_segments;

    size_t m_baseLink;

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

    std::mutex m_objectMutex;

    HumanStateProviderPrivate();
    ~HumanStateProviderPrivate();

    virtual std::vector<std::string> joints();
    virtual std::string baseLink();
    virtual bool setBaseLink(const std::string &baseLink);
};


#endif /* end of include guard: HUMANSTATEPROVIDERPRIVATE_H */
