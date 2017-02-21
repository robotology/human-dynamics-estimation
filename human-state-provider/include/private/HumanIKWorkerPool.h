//
//  HumanIKWorker.h
//  human-state-provider
//
//  Created by Francesco Romano on 20/02/17.
//  Copyright © 2017 Francesco Romano. All rights reserved.
//

#ifndef HUMANIKWORKER_H
#define HUMANIKWORKER_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <queue>
#include <vector>
#include <atomic>


namespace human {
    class HumanIKWorkerPool;
    struct IKSolverData;
}

namespace iDynTree {
    class Transform;
    class VectorDynSize;
}

class human::HumanIKWorkerPool {
    //I need the following: size of pool
    // with special: -1 : 1:1
    // 0: only one thread (i.e. same as 1)
    // > 0: pool

    //Then I need references to the following objects:
    //std::vector ok IKSolverData
    //resulting buffers: - vectors of final joint configuration
    //                   - vector of poses read from the Xsens

    //References to work data
    std::vector<human::IKSolverData> &m_solvers;
    std::vector<iDynTree::Transform>& m_linkPoseWRTWorld;
    iDynTree::VectorDynSize& m_jointsConfiguration;

    //objects to manage the pool
    std::queue<IKSolverData*> m_tasks;
    std::condition_variable m_inputSynchronizer;
    std::mutex m_inputMutex;


    std::unordered_map<IKSolverData*, int> m_results;
    std::condition_variable m_outputSynchronizer;
    std::mutex m_outputMutex;


    std::atomic<bool> m_shouldTerminate; //to facilitate check
    std::vector<bool> m_terminateCounter;
    std::condition_variable m_terminationSynchronizer;
    std::mutex m_terminationMutex;

    std::vector<std::thread> m_workers;

    void worker();

public:
    HumanIKWorkerPool(int size,
                      std::vector<human::IKSolverData> &solvers,
                      std::vector<iDynTree::Transform>& linkPoseWRTWorld,
                      iDynTree::VectorDynSize& jointsConfiguration);

    ~HumanIKWorkerPool();

    void runAndWait();

};



#endif /* end of include guard: HUMANIKWORKER_H */
