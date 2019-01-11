/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanIKWorkerPool.h"

#include <yarp/os/LogStream.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <thread>

const std::string IKLogPrefix = "HumanIKWorkerPool :";

HumanIKWorkerPool::HumanIKWorkerPool(int size,
                                     std::vector<LinkPairInfo> &linkPairs,
                                     std::vector<SegmentInfo> &segments)
    : m_linkPairs(linkPairs)
    , m_segments(segments)
    , m_shouldTerminate(false)
{
    if (size < 0) {
        size = static_cast<int>(linkPairs.size());
    }
    else if (size == 0) {
        size = 1;
    }

    yInfo() << IKLogPrefix << "IK thread pool initialized with " << size << " threads";
    m_poolSize = static_cast<unsigned>(size);
    m_terminateCounter.reserve(m_poolSize);

    //TODO: queue reserve memory
    //queue does not allow to reserve memory.
    //If we find that allocation is an issue, we should manually move to
    //another container and manually manage the FIFO behaviour

    for (unsigned i = 0; i < static_cast<unsigned>(size); ++i) {
        std::thread(&HumanIKWorkerPool::worker, this).detach();
    }
}

HumanIKWorkerPool::~HumanIKWorkerPool()
{
    m_terminateCounter.resize(m_poolSize);
    m_terminateCounter.assign(m_poolSize, true); //dummy

    std::unique_lock<std::mutex> lock(m_inputMutex);
    m_shouldTerminate = true;

    m_inputSynchronizer.notify_all();
    m_inputSynchronizer.wait(lock, [&]() {return m_terminateCounter.empty(); });
}

void HumanIKWorkerPool::runAndWait()
{
    // Fill data for a thread
    {
        std::unique_lock<std::mutex> guard(m_inputMutex);
        for (auto& linkPair : m_linkPairs) {
            // Create a new struct of type WorkerData and pass it to the pool
            WorkerTaskData taskData = {
                linkPair,
                m_segments[static_cast<size_t>(linkPair.parentFrameSegmentsIndex)],
                m_segments[static_cast<size_t>(linkPair.childFrameSegmentsIndex)],
                linkPair.sInitial,
                std::distance(&*(m_linkPairs.begin()), &linkPair) //this is not properly clear with the range-based iterators
            };
            m_tasks.push(taskData);
            //yInfo() << "task, parent name : " << taskData.pairInfo.parentFrameName << ", child name : " << taskData.pairInfo.childFrameName;
            //yInfo() << "initial positions : " << taskData.sInitial.toString();
        }
        m_inputSynchronizer.notify_all();
    }

    // Blocked call
    std::unique_lock<std::mutex> outputGuard(m_outputMutex);
    m_outputSynchronizer.wait(outputGuard, [&]() { return m_results.size() >= m_linkPairs.size(); });

    m_results.clear();
    m_outputSynchronizer.notify_all();
}

int HumanIKWorkerPool::computeIK(WorkerTaskData& task)
{
    // Get floating base transformation from the pair model
    task.pairInfo.floatingBaseTransform = task.pairInfo.pairModel.getFrameTransform(task.pairInfo.floatingBaseIndex).inverse();

    // Set full initial condition for the pair model
    task.pairInfo.ikSolver->setFullJointsInitialCondition(&(task.pairInfo.floatingBaseTransform), &(task.sInitial));

    // Get the relative transformation between the parent and child frames
    iDynTree::Transform parent_H_target = task.parentFrameInfo.poseWRTWorld.inverse() * task.childFrameInfo.poseWRTWorld;

    // Update ik target
    task.pairInfo.ikSolver->updateTarget(task.childFrameInfo.segmentName, parent_H_target);

    // Solve ik problem
    auto tick = std::chrono::high_resolution_clock::now();

    int result = task.pairInfo.ikSolver->solve();

    if (!result) {
        yError() << "Failed to solve IK";
        return -1;
    }

    auto tock = std::chrono::high_resolution_clock::now();

    // Get the last ik solution
    // TODO: Verify if relativeTransformation is stored correctly,
    // It gets baseTransformSolution	solution for the base position from getReducedSolution()
    task.pairInfo.ikSolver->getFullJointsSolution(task.pairInfo.relativeTransformation, task.pairInfo.jointConfigurations);

    yDebug() << "IK took"
             << std::chrono::duration_cast<std::chrono::milliseconds>(tock - tick).count() << "ms"
             << " Joint configuration solution is : " << task.pairInfo.jointConfigurations.toString();

    return result;
}

void HumanIKWorkerPool::computeJointVelocities(WorkerTaskData& task, iDynTree::VectorDynSize& relativeVelocity)
{
    // Update kinDynComputations object
    iDynTree::Vector3 worldGravity;
    worldGravity.zero();
    worldGravity(2) = -9.81;

    //Obtain the pointer to the kynDyn object, just as an alias
    iDynTree::KinDynComputations *computations = task.pairInfo.kinDynComputations.get();
    //As we computed the IK, set the state
    task.pairInfo.jointVelocities.zero();
    computations->setRobotState(task.pairInfo.jointConfigurations,
                                task.pairInfo.jointVelocities,
                                worldGravity);

    //Parent, child and relative jacobians
    iDynTree::MatrixDynSize& relativeJacobian = task.pairInfo.relativeJacobian;
    computations->getRelativeJacobian(task.pairInfo.parentFrameModelIndex,
                                      task.pairInfo.childFrameModelIndex,
                                      relativeJacobian);

    //Pseudo-invert the Jacobian.
    //Compute the QR decomposition
    task.pairInfo.jacobianDecomposition.compute(iDynTree::toEigen(relativeJacobian));
    //the solve method on the decomposition directly solves the associated least-squares problem
    relativeVelocity = task.childFrameInfo.velocities;
    iDynTree::toEigen(relativeVelocity) -= iDynTree::toEigen(task.parentFrameInfo.velocities);
    iDynTree::toEigen(task.pairInfo.jointVelocities) = task.pairInfo.jacobianDecomposition.solve(iDynTree::toEigen(relativeVelocity));
}

void HumanIKWorkerPool::worker() {
    //Preallocate some thread-local variables to be used in the computation
    iDynTree::VectorDynSize relativeVelocity(6);

    while (true) {
#ifdef EIGEN_RUNTIME_NO_MALLOC
        Eigen::internal::set_is_malloc_allowed(false);
#endif

        std::unique_lock<std::mutex> guard(m_inputMutex);
        m_inputSynchronizer.wait(guard, [&](){ return m_shouldTerminate || !m_tasks.empty(); } );
        if (m_shouldTerminate) {
            if (m_terminateCounter.empty()) {
                yError() << IKLogPrefix << "Trying to remove a thread which was not expected to exist";
            } else {
                m_terminateCounter.pop_back();
            }
            m_inputSynchronizer.notify_one();
            break;
        }

        WorkerTaskData task = m_tasks.front();
        m_tasks.pop();
        guard.unlock();

        //Do computations
        //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        int ikResult = computeIK(task);
        //std::cerr << "IK took " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t1).count() << "ms" << std::endl;

        if (ikResult < 0) {
            yError() << IKLogPrefix << "Failed to compute IK for " << task.pairInfo.parentFrameName.c_str() << ", "
                     << task.pairInfo.childFrameName.c_str() << " with error " << ikResult;
        }
        computeJointVelocities(task, relativeVelocity);

        //Notify caller
        std::unique_lock<std::mutex> outputGuard(m_outputMutex);
        m_outputSynchronizer.wait(outputGuard, [&](){ return m_results.find(task.identifier) == m_results.end(); });

        //insert
        m_results.insert(std::unordered_map<unsigned, int>::value_type(task.identifier, ikResult > 0));
        m_outputSynchronizer.notify_all();

#ifdef EIGEN_RUNTIME_NO_MALLOC
        Eigen::internal::set_is_malloc_allowed(true);
#endif
    }
}


