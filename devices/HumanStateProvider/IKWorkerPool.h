// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IKWORKERPOOL_H
#define IKWORKERPOOL_H

#include <vector>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <unordered_map>

#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Model.h>
#include <iDynTree/Indices.h>
#include <iDynTree/JointState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/InverseKinematics.h>

#include <Eigen/QR>

#include <yarp/sig/Vector.h>

/*!
 * Relevant information on the submodel between two links (segments)
 * Needed to compute inverse kinematics, velocities, etc
 */
struct LinkPairInfo {
    // Variables representing the DoFs between the two frames
    iDynTree::VectorDynSize jointConfigurations;
    iDynTree::VectorDynSize jointVelocities;

    // Transformation variable
    iDynTree::Transform relativeTransformation; // TODO: If this is wrt global frame

    // IK elements (i.e. compute joints)
    std::shared_ptr<iDynTree::InverseKinematics> ikSolver;
    double positionTargetWeight;
    double rotationTargetWeight;
    double costRegularization;

    // Initial joint positions
    iDynTree::VectorDynSize sInitial;

    // Reduced model of link pair
    iDynTree::Model pairModel;

    // Floating base variables
    iDynTree::LinkIndex floatingBaseIndex;
    iDynTree::Transform floatingBaseTransform;


    // Velocity-related elements
    iDynTree::MatrixDynSize relativeJacobian;
    std::unique_ptr<iDynTree::KinDynComputations> kinDynComputations;

    // Mapping from link pair to full model. Needed to map from small to complete problem
    std::string parentFrameName; //name of the parent frame
    iDynTree::FrameIndex parentFrameModelIndex; //index of the frame in the iDynTree Model
    iDynTree::FrameIndex parentFrameSegmentsIndex; //index of the parent frame in the segment list

    std::string childFrameName; //name of the child frame
    iDynTree::FrameIndex childFrameModelIndex; //index of the frame in the iDynTree Model
    iDynTree::FrameIndex childFrameSegmentsIndex; //index of the child frame in the segment list

    std::vector<std::pair<size_t, size_t>> consideredJointLocations; /*!< For each joint connecting the pair: first = offset in the full model , second = dofs of joint */

    LinkPairInfo() = default;
#if defined(_MSC_VER) && _MSC_VER < 1900
    LinkPairInfo(LinkPairInfo&& rvalue)
    : jointConfigurations(std::move(rvalue.jointConfigurations))
    , jointVelocities(std::move(rvalue.jointVelocities))
    , ikSolver(std::move(rvalue.ikSolver))
    , relativeJacobian(std::move(rvalue.relativeJacobian))
    , kinDynComputations(std::move(rvalue.kinDynComputations))
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

// Relevant information on the segment input
struct SegmentInfo {
    std::string segmentName;

    iDynTree::Transform poseWRTWorld;
    iDynTree::Twist velocities;

    // TODO: if not needed acceleration delete them
    yarp::sig::Vector accelerations;
};

/*!
 * Handles computations related to a single frame pairs.
 * This computations at the current stage are:
 * - IK given the relative transform
 * - joint velocities given the relative (angular) velocity
 */
class IKWorkerPool {

    struct WorkerTaskData {
        LinkPairInfo& pairInfo;
        SegmentInfo& parentFrameInfo;
        SegmentInfo& childFrameInfo;
        iDynTree::VectorDynSize& sInitial;
        long identifier;
    };

    //I need the following: size of pool
    // with special: -1 : 1:1
    // 0: only one thread (i.e. same as 1)
    // > 0: pool

    //Then I need references to the following objects:
    //std::vector ok IKSolverData
    //resulting buffers: - vectors of final joint configuration
    //                   - vector of poses read from the Xsens


    // References to work data
    std::vector<LinkPairInfo> &m_linkPairs;
    std::vector<SegmentInfo> &m_segments;

    // Objects to manage the pool
    std::queue<WorkerTaskData> m_tasks;
    std::condition_variable m_inputSynchronizer;
    std::mutex m_inputMutex;

    std::unordered_map<long, bool> m_results;
    std::condition_variable m_outputSynchronizer;
    std::mutex m_outputMutex;


    // This is synchronized with inputMutex
    bool m_shouldTerminate;
    std::vector<bool> m_terminateCounter;

    unsigned m_poolSize;

    int computeIK(WorkerTaskData& task);
    void computeJointVelocities(WorkerTaskData& task, iDynTree::Twist& relativeVelocity);

    void worker();

public:
    IKWorkerPool(int size,
                      std::vector<LinkPairInfo> &linkPairs,
                      std::vector<SegmentInfo> &segments);

    ~IKWorkerPool();

    /*!
     * Perform a "full" run of the implemented operations.
     * Currently: IK and joint velocities computation
     * @note this function is blocking
     * @note as this function does not perform a copy of the objects
     * it assumes nobody touches the objects.
     */
    void runAndWait();

    int closeIKWorkerPool();

};

#endif // IKWORKERPOOL_H
