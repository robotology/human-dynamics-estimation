/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HUMANIKWORKERPOOL_H
#define HUMANIKWORKERPOOL_H

#include <vector>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <unordered_map>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Model/Indices.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/InverseKinematics.h>

#include <Eigen/QR>

#include <yarp/sig/Vector.h>

/*!
 * Relevant information on the submodel between two links (segments)
 * Needed to compute inverse kinematics, velocities, etc
 */
struct LinkPairInfo {
    //variables representing the DoFs between the two frames
    iDynTree::VectorDynSize jointConfigurations;
    iDynTree::VectorDynSize jointVelocities;

    // Transformation variable
    iDynTree::Transform relativeTransformation; // TODO: If this is wrt global frame

    //IK elements (i.e. compute joints)
    std::unique_ptr<iDynTree::InverseKinematics> ikSolver;

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

    LinkPairInfo(const LinkPairInfo&) = delete;
};

/*!
 * Relevant information on the segment input
 * This saves the information coming from a FrameProvider
 */
struct SegmentInfo {
    std::string segmentName;

    iDynTree::Transform poseWRTWorld;
    iDynTree::VectorDynSize velocities;
    //TODO if not needed acceleration delete them
    yarp::sig::Vector accelerations;
};

/*!
 * Handles computations related to a single frame pairs.
 * This computations at the current stage are:
 * - IK given the relative transform
 * - joint velocities given the relative (angular) velocity
 */
class HumanIKWorkerPool {

    struct WorkerTaskData {
        LinkPairInfo& pairInfo;
        SegmentInfo& parentFrameInfo;
        SegmentInfo& childFrameInfo;
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


    //References to work data
    std::vector<LinkPairInfo> &m_linkPairs;
    std::vector<SegmentInfo> &m_segments;

    //objects to manage the pool
    std::queue<WorkerTaskData> m_tasks;
    std::condition_variable m_inputSynchronizer;
    std::mutex m_inputMutex;


    std::unordered_map<long, bool> m_results;
    std::condition_variable m_outputSynchronizer;
    std::mutex m_outputMutex;


    //This is synchronized with inputMutex
    bool m_shouldTerminate;
    std::vector<bool> m_terminateCounter;

    unsigned m_poolSize;

    int computeIK(WorkerTaskData& task);
    void computeJointVelocities(WorkerTaskData& task, iDynTree::VectorDynSize& relativeVelocity);

    void worker();

public:
    HumanIKWorkerPool(int size,
                      std::vector<LinkPairInfo> &linkPairs,
                      std::vector<SegmentInfo> &segments);

    ~HumanIKWorkerPool();

    /*!
     * Perform a "full" run of the implemented operations.
     * Currently: IK and joint velocities computation
     * @note this function is blocking
     * @note as this function does not perform a copy of the objects
     * it assumes nobody touches the objects.
     */
    void runAndWait();

};

#endif // HUMANIKWORKERPOOL_H
