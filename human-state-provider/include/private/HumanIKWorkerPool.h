//
//  HumanIKWorker.h
//  human-state-provider
//
//  Created by Francesco Romano on 20/02/17.
//  Copyright © 2017 Francesco Romano. All rights reserved.
//

#ifndef HUMANIKWORKER_H
#define HUMANIKWORKER_H

#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <queue>
#include <vector>


namespace human {
    class HumanIKWorkerPool;
    struct LinkPairInfo;
    struct SegmentInfo;
}

namespace iDynTree {
    class Transform;
    class VectorDynSize;
}

/*! 
 * Handles computations related to a single frame pairs.
 * This computations at the current stage are:
 * - IK given the relative transform
 * - joint velocities given the relative (angular) velocity
 */
class human::HumanIKWorkerPool {

    struct WorkerTaskData {
        human::LinkPairInfo& pairInfo;
        human::SegmentInfo& parentFrameInfo;
        human::SegmentInfo& childFrameInfo;
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
    std::vector<human::LinkPairInfo> &m_linkPairs;
    std::vector<human::SegmentInfo> &m_segments;

    //objects to manage the pool
    std::queue<WorkerTaskData> m_tasks;
    std::condition_variable m_inputSynchronizer;
    std::mutex m_inputMutex;


    std::unordered_map<unsigned, bool> m_results;
    std::condition_variable m_outputSynchronizer;
    std::mutex m_outputMutex;


    //This is synchronized with inputMutex
    bool m_shouldTerminate;
    std::vector<bool> m_terminateCounter;

    unsigned m_poolSize;

    int computeIK(WorkerTaskData& task);
    void computeJointVelocities(WorkerTaskData& task);

    void worker();

public:
    HumanIKWorkerPool(int size,
                      std::vector<human::LinkPairInfo> &linkPairs,
                      std::vector<human::SegmentInfo> &segments);

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



#endif /* end of include guard: HUMANIKWORKER_H */
