//
//  HumanIKWorker.cpp
//  human-state-provider
//
//  Created by Francesco Romano on 20/02/17.
//  Copyright © 2017 Francesco Romano. All rights reserved.
//

#include "HumanIKWorkerPool.h"
#include "HumanStateProviderPrivate.h"

#include <inversekinematics/InverseKinematics.h>

#include <thread>

namespace human {


    HumanIKWorkerPool::HumanIKWorkerPool(int size,
                                         std::vector<human::IKSolverData> &solvers,
                                         std::vector<iDynTree::Transform>& linkPoseWRTWorld,
                                         iDynTree::VectorDynSize& jointsConfiguration)
    : m_solvers(solvers)
    , m_linkPoseWRTWorld(linkPoseWRTWorld)
    , m_jointsConfiguration(jointsConfiguration)
    , m_shouldTerminate(false)
    {
        if (size == -1) {
            size = solvers.size();
        } else if (size == 0) {
            size = 1;
        }

        m_workers.reserve(size);
        m_terminateCounter.reserve(size);
        for (unsigned i = 0; i < static_cast<unsigned>(size); ++i) {
            m_workers.push_back(std::thread(&HumanIKWorkerPool::runAndWait, this));
        }

    }

    HumanIKWorkerPool::~HumanIKWorkerPool()
    {
        //to be safe (probably it is not needed)
        //wait for all the threads to return
        std::unique_lock<std::mutex> lock(m_terminationMutex);
        m_terminateCounter.resize(m_workers.size());
        m_terminateCounter.assign(m_workers.size(), true); //dummy
        m_shouldTerminate = true;
        m_terminationSynchronizer.wait(lock, [&]() { return m_terminateCounter.empty(); });

    }

    void HumanIKWorkerPool::runAndWait()
    {
        std::unique_lock<std::mutex> guard(m_inputMutex);
        for (std::vector<IKSolverData>::iterator iterator(m_solvers.begin());
             iterator != m_solvers.end(); ++iterator) {
            //now push each element
            m_tasks.push(&(*iterator));
        }
        m_inputSynchronizer.notify_all();
        //as this call is blocking I have to wait for all the results
        std::unique_lock<std::mutex> outputGuard(m_outputMutex);
        m_outputSynchronizer.wait(outputGuard, [&]() { return m_results.size() >= m_solvers.size(); });
        for (std::unordered_map<IKSolverData*, int>::const_iterator result(m_results.begin());
             result != m_results.end(); ++result) {
            if (result->second != 0) {
                yError("Failed to compute IK for frames %s, %s with error %d", result->first->frameNames.first.c_str(), result->first->frameNames.second.c_str(), result->second);
                continue;
            }
            //Copy solution to the global vector
            for (std::vector<std::pair<unsigned, unsigned> >::const_iterator location(result->first->consideredJointLocations.begin());
                 location != result->first->consideredJointLocations.end(); ++location) {

                //copy from location.first for location.second elements into the final vector
                for (unsigned i = 0; i < location->second; ++i) {
                    m_jointsConfiguration(location->first + i) = result->first->solution(i);
                }
            }
        }
        m_results.clear();
        m_outputSynchronizer.notify_all();
    }


    void HumanIKWorkerPool::worker() {

        while (!m_shouldTerminate) {
            std::unique_lock<std::mutex> guard(m_inputMutex);
            m_inputSynchronizer.wait(guard, [&](){ return !m_tasks.empty(); } );

            IKSolverData *task = m_tasks.front();
            m_tasks.pop();
            guard.unlock();

            //Now I can work with task as nobody will take it
            //This for loop can be parallelized.
            //Theoretically there is no shared data here. Also the access to the final vector should not cause a
            //data race as joints should not be shared between solvers
            task->solver->setDesiredParentFrameAndEndEffectorTransformations(m_linkPoseWRTWorld[task->frameIndeces.first],
                                                                             m_linkPoseWRTWorld[task->frameIndeces.second]);
            int result = task->solver->runIK();

            std::unique_lock<std::mutex> outputGuard(m_outputMutex);
            m_outputSynchronizer.wait(outputGuard, [&](){ return m_results.find(task) == m_results.end(); });

            task->solution = task->solver->getLastSolution();
            //insert
            m_results.insert(std::unordered_map<IKSolverData*, int>::value_type(task, result));
            m_outputSynchronizer.notify_all();
        }

        std::unique_lock<std::mutex> lock(m_terminationMutex);
        if (m_terminateCounter.empty()) {
            yError("Trying to remove a thread which was not expected to exist");
        } else {
            m_terminateCounter.pop_back();
        }
        m_terminationSynchronizer.notify_one();

    }

}
