//
//  HumanStateProvider.cpp
//  human-state-provider
//
//  Created by Francesco Romano on 20/02/17.
//  Copyright © 2017 Francesco Romano. All rights reserved.
//
#include "HumanStateProvider.h"

#include "HumanStateProviderPrivate.h"
#include "HumanIKWorkerPool.h"

#include <thrifts/HumanState.h>
#include <thrifts/HumanStateProviderService.h>

#include <inversekinematics/InverseKinematics.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameProvider.h>
#include <yarp/dev/IXsensMVNInterface.h>
#include <yarp/sig/Vector.h>

#include <Eigen/QR>

#include <algorithm>
#include <cassert>
#include <string>
#include <vector>
#include <stack>
#include <thread>

namespace human {

    /*!
     * @brief analyze model and list of segments to create all possible segment pairs
     *
     * @param[in] model the full model
     * @param[in] humanSegments list of segments on which look for the possible pairs
     * @param[out] framePairs resulting list of all possible pairs. First element is parent, second is child
     * @param[out] framePairIndeces indeces in the humanSegments list of the pairs in framePairs
     */
    static void createEndEffectorsPairs(const iDynTree::Model& model,
                                        const std::vector<yarp::experimental::dev::FrameReference>& humanSegments,
                                        std::vector<std::pair<std::string, std::string> > &framePairs,
                                        std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex> > &framePairIndeces);


    HumanStateProvider::HumanStateProvider()
    : m_pimpl(new HumanStateProviderPrivate()) {}

    HumanStateProvider::~HumanStateProvider()
    {
        delete m_pimpl;
        m_pimpl = 0;
    }

    double HumanStateProvider::getPeriod()
    {
        assert(m_pimpl);
        return m_pimpl->m_period;
    }

    bool HumanStateProvider::updateModule()
    {
        assert(m_pimpl);

        //read from Xsens
        if (!m_pimpl->m_frameProvider) return false;

        //Read goes to buffers
        yarp::experimental::dev::IFrameProviderStatus status = m_pimpl->m_frameProvider->getFrameInformation(m_pimpl->m_buffers.poses,
                                                                                                             m_pimpl->m_buffers.velocities,
                                                                                                             m_pimpl->m_buffers.accelerations);
        if (status != yarp::experimental::dev::IFrameProviderStatusOK) {
            //TODO: instead of return true we should handle timeout
            return true;
        }

        //process incoming data
        //In this first part we associate the data in the buffers into the respective segments
        for (unsigned index = 0; index < m_pimpl->m_buffers.poses.size(); ++index) {
            iDynTree::Position position(m_pimpl->m_buffers.poses[index](0),
                                        m_pimpl->m_buffers.poses[index](1),
                                        m_pimpl->m_buffers.poses[index](2));
            iDynTree::Rotation orientation = iDynTree::Rotation::RotationFromQuaternion(iDynTree::Vector4(m_pimpl->m_buffers.poses[index].data() + 3, 4));

            human::SegmentInfo &segmentInfo = m_pimpl->m_segments[index];
            segmentInfo.poseWRTWorld = iDynTree::Transform(orientation, position);
            for (unsigned i = 0; i < m_pimpl->m_buffers.velocities[index].size(); ++i) {
                segmentInfo.velocities(i) = m_pimpl->m_buffers.velocities[index](i);
            }
        }
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        m_pimpl->m_ikPool->runAndWait();
        std::cerr << "Data process took " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t1).count() << "ms" << std::endl;

        // Now reconstruct everything
        //Data saved in jointsConfiguration and jointsVelocity of the various pairInfo
        // Probably this can be done also at the single thread level
        for (auto &pair : m_pimpl->m_linkPairs) {
            size_t jointIndex = 0;
            for (auto &pairJoint : pair.consideredJointLocations) {
                Eigen::Map<Eigen::VectorXd> jointPositions(m_pimpl->m_buffers.jointsConfiguration.data(), m_pimpl->m_buffers.jointsConfiguration.size());
                jointPositions.segment(pairJoint.first, pairJoint.second) = iDynTree::toEigen(pair.jointConfigurations).segment(jointIndex, pairJoint.second);
                Eigen::Map<Eigen::VectorXd> jointVelocities(m_pimpl->m_buffers.jointsVelocity.data(), m_pimpl->m_buffers.jointsVelocity.size());
                jointVelocities.segment(pairJoint.first, pairJoint.second) = iDynTree::toEigen(pair.jointVelocities).segment(jointIndex, pairJoint.second);

                jointIndex += pairJoint.second;
            }
        }


        human::HumanState &currentState = m_pimpl->m_outputPort.prepare();
        currentState.positions = m_pimpl->m_buffers.jointsConfiguration;
        currentState.velocities = m_pimpl->m_buffers.jointsVelocity;

        {
            std::lock_guard<std::mutex> guard(m_pimpl->m_objectMutex);
            if (m_pimpl->m_baseLink < m_pimpl->m_segments.size()) {
                //Output base link
                yarp::sig::Vector &basePose = m_pimpl->m_buffers.poses[m_pimpl->m_baseLink];
                yarp::sig::Vector &baseVelocity = m_pimpl->m_buffers.velocities[m_pimpl->m_baseLink];

                currentState.baseOriginWRTGlobal.x = basePose[0];
                currentState.baseOriginWRTGlobal.y = basePose[1];
                currentState.baseOriginWRTGlobal.z = basePose[2];

                currentState.baseOrientationWRTGlobal.w = basePose[3];
                currentState.baseOrientationWRTGlobal.imaginary.x = basePose[4];
                currentState.baseOrientationWRTGlobal.imaginary.y = basePose[5];
                currentState.baseOrientationWRTGlobal.imaginary.z = basePose[6];
                currentState.baseVelocityWRTGlobal = baseVelocity;
            }
        }

        m_pimpl->m_outputPort.write();

        return true;
    }

    bool HumanStateProvider::configure(yarp::os::ResourceFinder &rf)
    {
        assert(m_pimpl);
        std::lock_guard<std::mutex> guard(m_pimpl->m_objectMutex);

        //Name of the module
        std::string name = rf.check("name", yarp::os::Value("human-state-provider"), "Checking module name").asString();
        setName(name.c_str());

        //Period to compute the state
        int period = rf.check("period", yarp::os::Value(100), "Checking period in [ms]").asInt();
        m_pimpl->m_period = period / 1000.0;

        yarp::os::Value falseValue; falseValue.fromString("false");
        bool playbackMode = rf.check("playback", falseValue, "Checking playback mode").asBool();

        //Open the IFrameProvider device
        std::string remote = rf.check("xsens_remote_name", yarp::os::Value("/xsens"), "Checking xsens driver port prefix").asString();
        std::string local = rf.check("xsens_local_name", yarp::os::Value("/" + name + "/xsens"), "Checking xsens local port prefix").asString();

        yarp::os::Property driverOptions;
        if (playbackMode) {
            driverOptions.put("device", "xsens_mvn_remote_light");
            driverOptions.put("remote", remote);
            driverOptions.put("local", local);
            driverOptions.put("segments", rf.find("segments"));
            driverOptions.put("autoconnect", "false");

        } else {
            driverOptions.put("device", "xsens_mvn_remote");
            driverOptions.put("remote", remote);
            driverOptions.put("local", local);
        }


        if (!m_pimpl->m_humanDriver.open(driverOptions)) {
            yError("Could not create connection to human driver");
            return false;
        }

        if (!m_pimpl->m_humanDriver.view(m_pimpl->m_frameProvider) || !m_pimpl->m_frameProvider) {
            yError("Specified driver does not support FrameProvider interface");
            close();
            return false;
        }

        if (!m_pimpl->m_humanDriver.view(m_pimpl->m_frameProviderTimed) || !m_pimpl->m_frameProviderTimed) {
            yError("Specified driver does not support PreciselyTimed interface");
            close();
            return false;
        }

        //get URDF model which will be needed for the IK
        if (!rf.check("human_urdf", "Checking subject URDF file")) {
            yError("Could not find \"human_urdf\" parameter");
            close();
            return false;

        }
        std::string urdfFile = rf.findFile(rf.find("human_urdf").asString());
        if (urdfFile.empty()) {
            yError("Could not find urdf file %s", rf.find("human_urdf").asString().c_str());
            close();
            return false;
        }

        //Load the full model
        iDynTree::ModelLoader modelLoader;
        if (!modelLoader.loadModelFromFile(urdfFile) || !modelLoader.isValid()) {
            yError("Could not load URDF model %s", urdfFile.c_str());
            close();
            return false;
        }
        //Copy this model as we will change the one loaded my the loader
        iDynTree::Model humanModel = modelLoader.model();

        //get all joints
        //If a joint has multiple dofs, it will appear multiple times
        m_pimpl->m_humanJointNames.resize(humanModel.getNrOfDOFs());
        for (iDynTree::JointIndex index = 0; static_cast<size_t>(index) < humanModel.getNrOfJoints(); ++index) {
            iDynTree::IJointPtr joint = humanModel.getJoint(index);
            std::string jointName = humanModel.getJointName(index);

            size_t offset = joint->getDOFsOffset();
            if (offset >= humanModel.getNrOfDOFs() || joint->getNrOfDOFs() == 0)
                continue;
            for (unsigned dof = 0; dof < joint->getNrOfDOFs(); ++dof) {
                m_pimpl->m_humanJointNames[offset + dof] = jointName;
            }
        }

        // Get the base link
        std::string baseLink = rf.check("base_link", yarp::os::Value(""), "Checking base link").asString();
        //TODO: decide if we want to have also a different link and we should perform a transformation in the output phase

        // Resize internal data
        // 1) data of size # of segments (i.e. input data)
        std::vector<yarp::experimental::dev::FrameReference> segments = m_pimpl->m_frameProvider->frames();
        m_pimpl->m_baseLink = segments.size(); //initialized to end, i.e. no base

        m_pimpl->m_buffers.poses.resize(segments.size());
        m_pimpl->m_buffers.velocities.resize(segments.size());
        m_pimpl->m_buffers.accelerations.resize(segments.size());
        for (unsigned index = 0; index < segments.size(); ++index) {
            m_pimpl->m_buffers.poses[index].resize(7, 0.0);
            m_pimpl->m_buffers.velocities[index].resize(6, 0.0);
            m_pimpl->m_buffers.accelerations[index].resize(6, 0.0);
        }

        // 2) output buffer: size of # Dofs
        m_pimpl->m_buffers.jointsConfiguration.resize(humanModel.getNrOfDOFs());
        m_pimpl->m_buffers.jointsConfiguration.zero();
        m_pimpl->m_buffers.jointsVelocity.resize(humanModel.getNrOfDOFs());
        m_pimpl->m_buffers.jointsVelocity.zero();

        // 3) segments information
        m_pimpl->m_segments.resize(segments.size());
        for (size_t index = 0; index < segments.size(); ++index) {
            m_pimpl->m_segments[index].velocities.resize(6);
            m_pimpl->m_segments[index].velocities.zero();
            //TODO if needed acceleration do it

            m_pimpl->m_segments[index].segmentName = segments[index].frameName;
            if (baseLink == m_pimpl->m_segments[index].segmentName) {
                m_pimpl->m_baseLink = index;
                yInfo("Using base link %s", baseLink.c_str());
            }
        }
        if (m_pimpl->m_baseLink == segments.size()) {
            yInfo("No base link selected");
        }

        // 4) Get all the possible pairs composing the model
        std::vector<std::pair<std::string, std::string> > pairNames;
        std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex> > pairSegmentIndeces;
        createEndEffectorsPairs(humanModel, segments, pairNames, pairSegmentIndeces);

        m_pimpl->m_linkPairs.reserve(pairNames.size());

        std::string solverName = rf.check("ik_linear_solver", yarp::os::Value("ma27"), "Checking Linear solver name for IPOPT").asString();

        for (unsigned index = 0; index < pairNames.size(); ++index) {
            LinkPairInfo pairInfo;

            pairInfo.parentFrameName = pairNames[index].first;
            pairInfo.parentFrameSegmentsIndex = pairSegmentIndeces[index].first;

            pairInfo.childFrameName = pairNames[index].second;
            pairInfo.childFrameSegmentsIndex = pairSegmentIndeces[index].second;

            // Allocate the solver which provides also useful methods
            pairInfo.ikSolver = std::unique_ptr<InverseKinematics>(new InverseKinematics(solverName));
            pairInfo.ikSolver->setVerbosityLevel(0);

            if (!pairInfo.ikSolver->setModel(humanModel, pairInfo.parentFrameName, pairInfo.childFrameName)) {
                yWarning("Could not configure IK solver for frames %s,%s. Skipping pair", pairInfo.parentFrameName.c_str(), pairInfo.childFrameName.c_str());
                continue;
            }
            //now we have to obtain the information needed to map the IK solution
            //back to the total Dofs vector we need to output
            std::vector<std::string> solverJoints;
            pairInfo.ikSolver->getConsideredJoints(solverJoints);

            pairInfo.consideredJointLocations.reserve(solverJoints.size());
            for (std::vector<std::string>::const_iterator jointName(solverJoints.begin());
                 jointName != solverJoints.end(); ++jointName) {
                iDynTree::JointIndex jointIndex = humanModel.getJointIndex(*jointName);
                if (jointIndex == iDynTree::JOINT_INVALID_INDEX) {
                    yWarning("IK considered joint %s not found in the complete model", jointName->c_str());
                    continue;
                }
                iDynTree::IJointConstPtr joint = humanModel.getJoint(jointIndex);
                //Save location and length of each DoFs
                pairInfo.consideredJointLocations.push_back(std::pair<size_t, size_t>(joint->getDOFsOffset(), joint->getNrOfDOFs()));
            }

            pairInfo.jointConfigurations.resize(solverJoints.size());
            pairInfo.jointConfigurations.zero();

            //same size and initialization
            pairInfo.jointVelocities = pairInfo.jointConfigurations;
            

            //Now configure the kinDynComputation objects
            modelLoader.loadReducedModelFromFullModel(humanModel, solverJoints);
            const iDynTree::Model &reducedModel = modelLoader.model();
            //save the indeces
            //TODO: check if link or frame
            pairInfo.parentFrameModelIndex = reducedModel.getFrameIndex(pairInfo.parentFrameName);
            pairInfo.childFrameModelIndex = reducedModel.getFrameIndex(pairInfo.childFrameName);

            pairInfo.kinDynComputations = std::unique_ptr<iDynTree::KinDynComputations>(new iDynTree::KinDynComputations());

            pairInfo.kinDynComputations->loadRobotModel(modelLoader.model());
            pairInfo.parentJacobian.resize(6, modelLoader.model().getNrOfDOFs());
            pairInfo.parentJacobian.zero();
            pairInfo.childJacobian = pairInfo.relativeJacobian = pairInfo.parentJacobian;
            //overwriting the default contructed object
            pairInfo.jacobianDecomposition = Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(pairInfo.relativeJacobian.rows(), pairInfo.relativeJacobian.cols());

            m_pimpl->m_linkPairs.push_back(std::move(pairInfo));

        }

        //Thread pool
        int poolSize = -1;
        yarp::os::Value poolOption = rf.check("ikpool", yarp::os::Value(-1), "Checking size of pool");
        if (poolOption.isString()) {
            if (poolOption.asString() != "auto") {
                yWarning("Pool option not recognized. Do not using pool");
            } else {
                poolSize = static_cast<int>(std::thread::hardware_concurrency());
            }

        } else {
            poolSize = poolOption.asInt();
        }
        m_pimpl->m_ikPool = std::unique_ptr<human::HumanIKWorkerPool>(new human::HumanIKWorkerPool(poolSize,
                                                         m_pimpl->m_linkPairs,
                                                         m_pimpl->m_segments));

        if (!m_pimpl->m_ikPool) {
            yError("Could not create IK thread pool");
            close();
            return false;
        }

        std::string outputPortName = "/" + getName() + "/state:o";

        if (!m_pimpl->m_outputPort.open(outputPortName)) {
            yError("Could not open %s output port", outputPortName.c_str());
            close();
            return false;
        }

        human::HumanState &tempOutput = m_pimpl->m_outputPort.prepare();
        tempOutput.positions.resize(m_pimpl->m_buffers.jointsConfiguration.size());
        tempOutput.velocities.resize(m_pimpl->m_buffers.jointsConfiguration.size());
        tempOutput.baseVelocityWRTGlobal.resize(6);
        tempOutput.baseVelocityWRTGlobal.zero();
        tempOutput.baseOriginWRTGlobal.x = 0;
        tempOutput.baseOriginWRTGlobal.y = 0;
        tempOutput.baseOriginWRTGlobal.z = 0;
        tempOutput.baseOrientationWRTGlobal.w = 0;
        tempOutput.baseOrientationWRTGlobal.imaginary.x = 0;
        tempOutput.baseOrientationWRTGlobal.imaginary.y = 0;
        tempOutput.baseOrientationWRTGlobal.imaginary.z = 0;
        m_pimpl->m_outputPort.unprepare();

        std::string rpcPortName = "/" + getName() + "/rpc";
        m_pimpl->yarp().attachAsServer(m_pimpl->m_rpcPort);
        if (!m_pimpl->m_rpcPort.open(rpcPortName)) {
            yError("Could not open %s RPC port", rpcPortName.c_str());
            close();
            return false;
        }


        return true;

    }

    bool HumanStateProvider::close()
    {
        assert(m_pimpl);
        std::lock_guard<std::mutex> guard(m_pimpl->m_objectMutex);

        //explitily clear the pool
        m_pimpl->m_ikPool.reset();

        m_pimpl->m_outputPort.close();
        m_pimpl->m_rpcPort.close();
        m_pimpl->m_humanDriver.close();

        return true;
    }

    static void createEndEffectorsPairs(const iDynTree::Model& model,
                                        const std::vector<yarp::experimental::dev::FrameReference>& humanSegments,
                                        std::vector<std::pair<std::string, std::string> > &framePairs,
                                        std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex> > &framePairIndeces)
    {
        //for each element in human segments
        //extract it from the vector (to avoid duplications)
        //Look for it in the model and get neighbours.

        std::vector<yarp::experimental::dev::FrameReference> segments(humanSegments);
        size_t segmentCount = segments.size();

        while (!segments.empty()) {
            yarp::experimental::dev::FrameReference segment = segments.back();
            segments.pop_back();
            segmentCount--;

            iDynTree::LinkIndex linkIndex = model.getLinkIndex(segment.frameName);
            if (linkIndex < 0 || static_cast<unsigned>(linkIndex) >= model.getNrOfLinks()) {
                yWarning("Segment %s not found in the URDF model", segment.frameName.c_str());
                continue;
            }

            //this for loop should not be necessary, but this can help keeps the backtrace short
            //as we do not assume that we can go back further that this node
            for (unsigned neighbourIndex = 0; neighbourIndex < model.getNrOfNeighbors(linkIndex); ++neighbourIndex) {
                //remember the "biforcations"
                std::stack<iDynTree::LinkIndex> backtrace;
                //and the visited nodes
                std::vector<iDynTree::LinkIndex> visited;

                //I've already visited the starting node
                visited.push_back(linkIndex);
                iDynTree::Neighbor neighbour = model.getNeighbor(linkIndex, neighbourIndex);
                backtrace.push(neighbour.neighborLink);

                while (!backtrace.empty()) {
                    iDynTree::LinkIndex currentLink = backtrace.top();
                    backtrace.pop();
                    //add the current link to the visited
                    visited.push_back(currentLink);

                    std::string linkName = model.getLinkName(currentLink);

                    // check if this is a human segment
                    std::vector<yarp::experimental::dev::FrameReference>::iterator foundSegment = std::find_if(segments.begin(),
                                                                                                               segments.end(),
                                                                                                               [&](yarp::experimental::dev::FrameReference& frame){ return frame.frameName == linkName; });
                    if (foundSegment != segments.end()) {
                        std::vector<yarp::experimental::dev::FrameReference>::difference_type foundLinkIndex = std::distance(segments.begin(), foundSegment);
                        //Found! This is a segment
                        framePairs.push_back(std::pair<std::string, std::string>(segment.frameName, linkName));
                        framePairIndeces.push_back(std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>(segmentCount, foundLinkIndex));
                        break;
                    }
                    //insert all non-visited neighbours
                    for (unsigned i = 0; i < model.getNrOfNeighbors(currentLink); ++i) {
                        iDynTree::LinkIndex link = model.getNeighbor(currentLink, i).neighborLink;
                        //check if we already visited this segment
                        if (std::find(visited.begin(), visited.end(), link) != visited.end()) {
                            //Yes => skip
                            continue;
                        }
                        backtrace.push(link);
                    }
                }

            }
        }

    }

}
