//
//  HumanStateProvider.cpp
//  human-state-provider
//
//  Created by Francesco Romano on 20/02/17.
//  Copyright © 2017 Francesco Romano. All rights reserved.
//
#include "HumanStateProvider.h"

#include "HumanStateProviderPrivate.h"

#include <thrifts/HumanState.h>

#include <inversekinematics/InverseKinematics.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/Model.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IHumanSkeleton.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <cassert>
#include <string>
#include <vector>
#include <stack>
#include <thread>

namespace human {

    static void createEndEffectorsPairs(const iDynTree::Model& model,
                                        const std::vector<std::string>& humanSegments,
                                        std::vector<std::pair<std::string, std::string> > &framePairs,
                                        std::vector<std::pair<unsigned, unsigned> > &framePairIndeces);


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
        if (!m_pimpl->m_human) return false;

        if (!m_pimpl->m_human->getSegmentInformation(m_pimpl->m_buffers.poses,
                                                     m_pimpl->m_buffers.velocities,
                                                     m_pimpl->m_buffers.accelerations)) {
            yError("Failed to read subject segment information");
            return true; //do not close the module for now
            //TODO: put a countdown?
        }

        //process incoming data
        for (unsigned index = 0; index < m_pimpl->m_buffers.linkPoseWRTWorld.size(); ++index) {
            iDynTree::Position position(m_pimpl->m_buffers.poses[index](0),
                                              m_pimpl->m_buffers.poses[index](1),
                                              m_pimpl->m_buffers.poses[index](2));
            iDynTree::Rotation orientation = iDynTree::Rotation::RotationFromQuaternion(iDynTree::Vector4(m_pimpl->m_buffers.poses[index].data() + 3, 4));
            m_pimpl->m_buffers.linkPoseWRTWorld[index] = iDynTree::Transform(orientation, position);
        }

        //now to each solver pass the correct information
        for (std::vector<IKSolverData>::iterator iterator(m_pimpl->m_solvers.begin());
             iterator != m_pimpl->m_solvers.end(); ++iterator) {
            //This for loop can be parallelized.
            //Theoretically there is no shared data here. Also the access to the final vector should not cause a
            //data race as joints should not be shared between solvers
            iterator->solver->setDesiredParentFrameAndEndEffectorTransformations(m_pimpl->m_buffers.linkPoseWRTWorld[iterator->frameIndeces.first],
                                                                                 m_pimpl->m_buffers.linkPoseWRTWorld[iterator->frameIndeces.second]);
            int result = iterator->solver->runIK(iterator->solution);
            if (result != 0) {
                yError("Failed to compute IK for frames %s, %s", iterator->frameNames.first.c_str(), iterator->frameNames.second.c_str());
                continue;
            }
            //Copy solution to the global vector
            for (std::vector<std::pair<unsigned, unsigned> >::const_iterator location(iterator->consideredJointLocations.begin());
                 location != iterator->consideredJointLocations.end(); ++location) {
                //copy from location.first for location.second elements into the final vector
                for (unsigned i = 0; i < location->second; ++i) {
                    m_pimpl->m_buffers.jointsConfiguration(location->first + i) = iterator->solution(i);
                }
            }
        }

        return true;
    }

    bool HumanStateProvider::configure(yarp::os::ResourceFinder &rf)
    {
        assert(m_pimpl);

        std::string name = rf.check("name", yarp::os::Value("human-state-provider"), "Checking module name").asString();
        setName(name.c_str());

        int period = rf.check("period", yarp::os::Value(100), "Checking period in [ms]").asInt();
        m_pimpl->m_period = period / 1000.0;

        std::string remote = rf.check("xsens_remote_name", yarp::os::Value("/xsens"), "Checking xsens driver port prefix").asString();
        std::string local = rf.check("xsens_local_name", yarp::os::Value("/xsens_remote"), "Checking xsens local port prefix").asString();

        yarp::os::Property driverOptions;
        driverOptions.put("device", "xsens_mvn_remote");
        driverOptions.put("remote", remote);
        driverOptions.put("local", local);

        if (!m_pimpl->m_humanDriver.open(driverOptions)) {
            yError("Could not create connection to human driver");
            return false;
        }

        if (!m_pimpl->m_humanDriver.view(m_pimpl->m_human) || !m_pimpl->m_human) {
            yError("Specified driver does not support human interface");
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

        iDynTree::ModelLoader modelLoader;
        if (!modelLoader.loadModelFromFile(urdfFile) || !modelLoader.isValid()) {
            yError("Could not load URDF model %s", urdfFile.c_str());
            close();
            return false;
        }

        m_pimpl->m_buffers.jointsConfiguration.resize(modelLoader.model().getNrOfDOFs());
        m_pimpl->m_buffers.jointsConfiguration.zero();

        std::vector<std::string> segments = m_pimpl->m_human->segmentNames();

        m_pimpl->m_buffers.poses.resize(segments.size());
        m_pimpl->m_buffers.velocities.resize(segments.size());
        m_pimpl->m_buffers.accelerations.resize(segments.size());
        m_pimpl->m_buffers.linkPoseWRTWorld.resize(segments.size());
        for (unsigned index = 0; index < segments.size(); ++index) {
            m_pimpl->m_buffers.poses[index].resize(7, 0.0);
            m_pimpl->m_buffers.velocities[index].resize(6, 0.0);
            m_pimpl->m_buffers.accelerations[index].resize(6, 0.0);
        }

        //configure IK solvers
        //Get all the possible pairs for which we need an IK
        std::vector<std::pair<std::string, std::string> > pairNames;
        std::vector<std::pair<unsigned, unsigned> > pairIndeces;
        createEndEffectorsPairs(modelLoader.model(), segments, pairNames, pairIndeces);

        //#pair = #solvers
        m_pimpl->m_solvers.reserve(pairNames.size());
        for (std::vector<std::pair<std::string, std::string> >::iterator iterator(pairNames.begin());
             iterator != pairNames.end(); ++iterator) {
            //create solver structure
            IKSolverData solver;
            //save which frames (parent and target) the solver will consider
            solver.frameNames = *iterator;
            //and their indeces in the segments vector
            solver.frameIndeces = pairIndeces[std::distance(pairNames.begin(), iterator)];

            solver.solver = new InverseKinematics();
            if (!solver.solver->setModel(modelLoader.model(), iterator->first, iterator->second)) {
                yWarning("Could not configure IK solver for frames %s,%s. Skipping pair", iterator->first.c_str(), iterator->second.c_str());
                continue;
            }
            //now we have to obtain the information needed to map the IK solution
            //back to the total Dofs vector we need to output
            std::vector<std::string> solverJoints;
            solver.solver->getConsideredJoints(solverJoints);
            solver.consideredJointLocations.reserve(solverJoints.size());
            for (std::vector<std::string>::const_iterator jointName(solverJoints.begin());
                 jointName != solverJoints.end(); ++jointName) {
                iDynTree::JointIndex jointIndex = modelLoader.model().getJointIndex(*jointName);
                if (jointIndex == iDynTree::JOINT_INVALID_INDEX) {
                    yWarning("IK considered joint %s not found in the complete model", jointName->c_str());
                    continue;
                }
                iDynTree::IJointConstPtr joint = modelLoader.model().getJoint(jointIndex);
                //Save location and length of each DoFs
                solver.consideredJointLocations.push_back(std::pair<unsigned, unsigned>(joint->getDOFsOffset(), joint->getNrOfDOFs()));
            }

            solver.solution.resize(solverJoints.size());
            solver.solution.zero();

            m_pimpl->m_solvers.push_back(solver);
        }

        //Thread pool
        int poolSize = -1;
        yarp::os::Value poolOption = rf.check("ikpool", yarp::os::Value(-1), "Checking size of pool");
        if (poolOption.isString()) {
            if (poolOption.asString() != "auto") {
                yWarning("Pool option not recognized. Do not using pool");
            } else {
                poolSize = std::thread::hardware_concurrency();
            }

        } else {
            poolSize = poolOption.asInt();
        }
        m_pimpl->m_ikPool = new human::HumanIKWorkerPool(poolSize,
                                                         m_pimpl->m_solvers,
                                                         m_pimpl->m_buffers.linkPoseWRTWorld,
                                                         m_pimpl->m_buffers.jointsConfiguration);

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

        return true;

    }

    bool HumanStateProvider::close()
    {
        assert(m_pimpl);

        m_pimpl->m_outputPort.close();

        delete m_pimpl->m_ikPool;

        for (std::vector<IKSolverData>::iterator iterator(m_pimpl->m_solvers.begin());
             iterator != m_pimpl->m_solvers.end(); ++iterator) {
            delete iterator->solver;
        }
        m_pimpl->m_solvers.clear();

        m_pimpl->m_humanDriver.close();

        return true;
    }


    static void createEndEffectorsPairs(const iDynTree::Model& model,
                                        const std::vector<std::string>& humanSegments,
                                        std::vector<std::pair<std::string, std::string> > &framePairs,
                                        std::vector<std::pair<unsigned, unsigned> > &framePairIndeces)
    {
        //for each element in human segments
        //extract it from the vector (to avoid duplications)
        //Look for it in the model and get neighbours.

        std::vector<std::string> segments(humanSegments);
        unsigned segmentCount = segments.size();

        while (!segments.empty()) {
            std::string segment = segments.back();
            segments.pop_back();
            segmentCount--;

            iDynTree::LinkIndex linkIndex = model.getLinkIndex(segment);
            if (linkIndex < 0 || linkIndex >= model.getNrOfLinks()) {
                yWarning("Segment %s not found in the URDF model", segment.c_str());
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
                    std::vector<std::string>::iterator foundSegment = std::find(segments.begin(), segments.end(), linkName);
                    if (foundSegment != segments.end()) {
                        unsigned foundLinkIndex = std::distance(segments.begin(), foundSegment);
                        //Found! This is a segment
                        framePairs.push_back(std::pair<std::string, std::string>(segment, linkName));
                        framePairIndeces.push_back(std::pair<unsigned, unsigned>(segmentCount, foundLinkIndex));
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
