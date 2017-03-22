#include "InverseKinematics.h"

#include "InverseKinematicsV2IPOPT.h"

#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>

namespace human {

    class InverseKinematics::InverseKinematicsPrivate {
    public:

        InverseKinematicsPrivate()
        : updated(false)
        , initialized(false)
        , alreadyOptimized(false)
        , verbosityLevel(2)
        , maxIterations(3000)
        {
            solverPointer = new InverseKinematicsV2IPOPT();
            iDynTree::toEigen(solverPointer->gains) << 100, 100, 0.01;
            loader = IpoptApplicationFactory();
            loader->Options()->SetStringValue("hessian_approximation", "limited-memory");
        }

        Ipopt::SmartPtr< InverseKinematicsV2IPOPT > solverPointer;
        Ipopt::SmartPtr< Ipopt::IpoptApplication > loader;
        bool updated;
        bool initialized;
        bool alreadyOptimized;
        std::string _parentFrame;
        std::string _targetFrame;
        unsigned verbosityLevel;
        unsigned maxIterations;
        std::string m_solverName;

        bool update();
        bool getReducedModel(const iDynTree::Model& fullModel, const std::vector< std::string >& consideredJoints, iDynTree::Model& modelOutput);
        bool getFrameIndeces(const std::string& parentFrame, const std::string& endEffectorFrame, const iDynTree::Model model, iDynTree::FrameIndex& parentFrameIndex, iDynTree::FrameIndex& endEffectorFrameIndex);
        void removeUnsupportedJoints(const iDynTree::Model& modelInput, iDynTree::Model& modelOutput);
        bool autoSelectJointsFromTraversal(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, iDynTree::Model& modelOutput);

        iDynTree::VectorDynSize tempGuess; //auxiliary variable for random initialization
        iDynTree::VectorDynSize lastSolution;

    };

    bool InverseKinematics::InverseKinematicsPrivate::getReducedModel(const iDynTree::Model& fullModel, const std::vector< std::string >& consideredJoints, iDynTree::Model& modelOutput)
    {
        iDynTree::ModelLoader loader;
        bool success;

        success = loader.loadReducedModelFromFullModel(fullModel, consideredJoints);

        if (!success){
            std::cerr << "[ERROR IK] Cannot select joints: " ;
            for (std::vector< std::string >::const_iterator i = consideredJoints.begin(); i != consideredJoints.end(); ++i){
                std::cerr << *i << ' ';
            }
            std::cerr << std::endl;
            return false;
        }
        modelOutput = loader.model();

        return true;
    }

    bool InverseKinematics::InverseKinematicsPrivate::getFrameIndeces(const std::string& parentFrame, const std::string& endEffectorFrame, const iDynTree::Model model, iDynTree::FrameIndex& parentFrameIndex, iDynTree::FrameIndex& endEffectorFrameIndex)
    {
        parentFrameIndex = model.getFrameIndex(parentFrame);
        endEffectorFrameIndex = model.getFrameIndex(endEffectorFrame);

        if(parentFrameIndex == iDynTree::FRAME_INVALID_INDEX){
            std::cerr<<"[ERROR IK] Invalid parent frame: "<<parentFrame<< std::endl;
            return false;
        }
        else if(endEffectorFrameIndex == iDynTree::FRAME_INVALID_INDEX){
            std::cerr<<"[ERROR IK] Invalid End Effector Frame: "<<endEffectorFrame<< std::endl;
            return false;
        }

        return true;
    }

    void InverseKinematics::InverseKinematicsPrivate::removeUnsupportedJoints(const iDynTree::Model& modelInput, iDynTree::Model& modelOutput)
    {
        iDynTree::ModelLoader loader;
        std::vector< std::string > consideredJoints;

        for(int i=0; i < modelInput.getNrOfJoints(); ++i){
            if(modelInput.getJoint(i)->getNrOfDOFs() == 1){
                consideredJoints.push_back(modelInput.getJointName(i));
            }
            else std::cerr << "Joint " << modelInput.getJointName(i) << " ignored (" << modelInput.getJoint(i)->getNrOfDOFs() << " DOF)" << std::endl;
        }
        loader.loadReducedModelFromFullModel(modelInput, consideredJoints);
        modelOutput = loader.model();
    }

    bool InverseKinematics::InverseKinematicsPrivate::autoSelectJointsFromTraversal(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, iDynTree::Model& modelOutput)
    {
        iDynTree::FrameIndex parent;
        iDynTree::FrameIndex endEffector;
        bool success;

        success = getFrameIndeces(parentFrame, endEffectorFrame, modelInput, parent, endEffector);
        if(!success)
            return false;

        std::vector< std::string > consideredJointsAuto;
        iDynTree::Traversal traversal;

        modelInput.computeFullTreeTraversal(traversal, modelInput.getFrameLink(parent));

        iDynTree::LinkIndex visitedLink = modelInput.getFrameLink(endEffector);

        iDynTree::LinkIndex parentLinkIdx;
        iDynTree::IJointConstPtr joint;

        while (visitedLink != traversal.getBaseLink()->getIndex())
        {
            parentLinkIdx = traversal.getParentLinkFromLinkIndex(visitedLink)->getIndex();
            joint = traversal.getParentJointFromLinkIndex(visitedLink);

            consideredJointsAuto.insert(consideredJointsAuto.begin(), modelInput.getJointName(joint->getIndex()));

            visitedLink = parentLinkIdx;
        }
        
        success = getReducedModel(modelInput, consideredJointsAuto, modelOutput);
        if(!success)
            return false;
        
        return true;
    }

    bool InverseKinematics::InverseKinematicsPrivate::update()
    {
        updated = solverPointer->update();
        return updated;
    }


InverseKinematics::InverseKinematics()
: m_pimpl(new InverseKinematicsPrivate())
{
}

InverseKinematics::InverseKinematics(const std::string& solverName)
: m_pimpl(new InverseKinematicsPrivate())
{
    assert(m_pimpl);
    m_pimpl->m_solverName = solverName;
}


InverseKinematics::~InverseKinematics()
{
    delete m_pimpl;
    m_pimpl = 0;
}

InverseKinematics::InverseKinematics(const InverseKinematics&) { assert(false); }
InverseKinematics& InverseKinematics::operator=(const InverseKinematics&) { assert(false); return *this;}




bool InverseKinematics::setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame, const std::vector< std::string >& consideredJoints)
{
    assert(m_pimpl);

    m_pimpl->alreadyOptimized = false;

    bool success = false;
    iDynTree::ModelLoader loader;
    iDynTree::FrameIndex parent;
    iDynTree::FrameIndex endEffector;
    iDynTree::Model model;

    success = loader.loadModelFromFile(URDFfilename);
    
    if (!success) {
        std::cerr << "[ERROR] Error loading URDF model from " << URDFfilename << std::endl;
        return false;
    }

    model = loader.model();
        
    if ((!consideredJoints.empty()) && ((consideredJoints[0].compare("All") == 0) || (consideredJoints[0].compare("ALL") == 0) || (consideredJoints[0].compare("all") == 0))) {
        
        m_pimpl->removeUnsupportedJoints(model.copy(),model);
        success = m_pimpl->getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false; 

    } else if (!consideredJoints.empty()){
        success = m_pimpl->getReducedModel(model, consideredJoints, model);
        if(!success)
            return false;
        
        m_pimpl->removeUnsupportedJoints(model.copy(),model);
        
        success = m_pimpl->getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false;

    } else {

        success = m_pimpl->autoSelectJointsFromTraversal(model.copy(), parentFrame, endEffectorFrame, model);
        if(!success)
            return false;
        
        m_pimpl->removeUnsupportedJoints(model.copy(),model);
        
        success = m_pimpl->getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false;
        
    }
    
    m_pimpl->_parentFrame = parentFrame;
    m_pimpl->_targetFrame = endEffectorFrame;
    m_pimpl->tempGuess.resize(model.getNrOfDOFs());
    m_pimpl->lastSolution.resize(model.getNrOfDOFs());
    
    return m_pimpl->solverPointer->loadFromModel(model, parent, endEffector);
}

bool InverseKinematics::setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame)
{
    m_pimpl->alreadyOptimized = false;
    
    std::vector< std::string > emptyVector;
    
    emptyVector.resize(0);
    
    return setModelfromURDF(URDFfilename, parentFrame, endEffectorFrame, emptyVector);
} 



bool InverseKinematics::setModel(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, const bool autoSelectJoints)
{
    iDynTree::FrameIndex parentFrameIndex;
    iDynTree::FrameIndex endEffectorFrameIndex;
    iDynTree::Model model;
    
    m_pimpl->alreadyOptimized = false;
        
    if (autoSelectJoints){
        if(!m_pimpl->autoSelectJointsFromTraversal(modelInput, parentFrame, endEffectorFrame, model))
            return false;
    }
    
    m_pimpl->removeUnsupportedJoints(model.copy(),model);
    
    if (!m_pimpl->getFrameIndeces(parentFrame,endEffectorFrame, model, parentFrameIndex, endEffectorFrameIndex))
        return false;

    m_pimpl->_parentFrame = parentFrame;
    m_pimpl->_targetFrame = endEffectorFrame;
    m_pimpl->tempGuess.resize(model.getNrOfDOFs());
    m_pimpl->lastSolution.resize(model.getNrOfDOFs());
    
    return m_pimpl->solverPointer->loadFromModel(model, parentFrameIndex, endEffectorFrameIndex);
}

void InverseKinematics::setWeights(const iDynTree::Vector3& weights)
{
    m_pimpl->solverPointer->gains = weights;
    m_pimpl->updated = false;
}

void InverseKinematics::setDesiredPosition(const iDynTree::Position& desiredPosition)
{
    m_pimpl->solverPointer->desiredPosition = desiredPosition;
    m_pimpl->updated = false;
}

void InverseKinematics::setDesiredQuaternion(const iDynTree::Vector4& desiredQuaternion)
{
    m_pimpl->solverPointer->desiredQuaternion = desiredQuaternion;
    m_pimpl->updated = false;
}

void InverseKinematics::setDesiredTransformation(const iDynTree::Transform& p_H_t)
{
    setDesiredPosition(p_H_t.getPosition());
    setDesiredQuaternion(p_H_t.getRotation().asQuaternion());
}

void InverseKinematics::setDesiredParentFrameAndEndEffectorTransformations(const iDynTree::Transform& w_H_p, const iDynTree::Transform& w_H_t)
{
    setDesiredTransformation(w_H_p.inverse() * w_H_t);
}

void InverseKinematics::setDesiredJointPositions(const iDynTree::VectorDynSize& desiredJoints)
{
    m_pimpl->solverPointer->desiredJoints = desiredJoints;
    m_pimpl->updated = false;
}

void InverseKinematics::setGuess(const iDynTree::VectorDynSize& guess)
{
    m_pimpl->solverPointer->guess = guess;
}

bool InverseKinematics::setRandomGuess(const double feed, iDynTree::VectorDynSize& guess)
{
    return m_pimpl->solverPointer->randomInitialization(feed, guess);
}

bool InverseKinematics::setRandomGuess(const double feed, iDynTree::VectorDynSize& guess, const iDynTree::VectorDynSize& oldGuess, double minDistance, const int maxIter)
{
    bool success = false;
    int iter = 0;
    double norm = 0;

    do {
        success = setRandomGuess(feed + iter + 1, m_pimpl->tempGuess);
        if((m_pimpl->tempGuess.size() != oldGuess.size())|| !success ||(m_pimpl->tempGuess.size() == 0))
            return false;
        norm = (iDynTree::toEigen(m_pimpl->tempGuess) - iDynTree::toEigen(oldGuess)).norm()/m_pimpl->tempGuess.size();
        //std::cerr << "[IK - setRandomGuess] Norm:"<< norm << std::endl;
        iter++;
    } while((iter < maxIter)&&(norm < minDistance));
    
    if(iter == maxIter)
        std::cerr << "[IK - setRandomGuess] Maximum iteration reached." << std::endl;
    
    guess = m_pimpl->tempGuess;
    
    return true;
}


bool InverseKinematics::update(const iDynTree::Vector3& weights, const iDynTree::Position& desiredPosition, const iDynTree::Vector4& desiredQuaternion, const iDynTree::VectorDynSize& desiredJoints)
{
    m_pimpl->updated = m_pimpl->solverPointer->update(weights, desiredPosition, desiredQuaternion, desiredJoints);
    return m_pimpl->updated;
}

bool InverseKinematics::getErrors(iDynTree::Position& positionError, iDynTree::Rotation& rotationError, double* angleError)
{
    if(!m_pimpl->alreadyOptimized)
        return false;
    
    m_pimpl->solverPointer->computeErrors(positionError,rotationError,angleError);
    
    return true;
}

void InverseKinematics::getFrames(std::string& parentFrame, std::string& endEffectorFrame)
{
    parentFrame = m_pimpl->_parentFrame;
    endEffectorFrame = m_pimpl->_targetFrame;
}

bool InverseKinematics::getConsideredJoints(std::vector< std::string >& consideredJoints)
{
    iDynTree::Model _model;
    if(!m_pimpl->solverPointer->getModel(_model))
        return false;
    
    consideredJoints.resize(_model.getNrOfJoints());
    for(int i=0; i<_model.getNrOfJoints(); ++i){
        consideredJoints[i] = _model.getJointName(i);
    }
    
    return true;
}


signed int InverseKinematics::runIK(iDynTree::VectorDynSize& jointsOut)
{
    int result = runIK();
    jointsOut = m_pimpl->lastSolution;
    return result;
}

signed int InverseKinematics::runIK()
{
    if (!m_pimpl->initialized){
        if (!initialize()) return -6;
    }

    if(!m_pimpl->updated){
        bool success;
        success = m_pimpl->update();
        if(!success){
            std::cerr << "[ERROR] Error when trying to update IK data" << std::endl;
            return -6;
        }
    }

    if (m_pimpl->alreadyOptimized){
        // std::cerr << "Reoptimize!!" << std::endl;
        m_pimpl->loader->ReOptimizeTNLP(m_pimpl->solverPointer);
        m_pimpl->lastSolution = m_pimpl->solverPointer->jointResult;
        return m_pimpl->solverPointer->exitCode;
    } else {
        //std::cerr << "Optimize!!" << std::endl;
        m_pimpl->loader->OptimizeTNLP(m_pimpl->solverPointer);
        if (m_pimpl->solverPointer->exitCode >=0
            || m_pimpl->solverPointer->exitCode == -2) {
                m_pimpl->alreadyOptimized = true;
        }
        m_pimpl->lastSolution = m_pimpl->solverPointer->jointResult;
        return m_pimpl->solverPointer->exitCode;
    }
}

iDynTree::VectorDynSize& InverseKinematics::getLastSolution()
{
    return m_pimpl->lastSolution;
}

void InverseKinematics::setVerbosityLevel(unsigned int level) { m_pimpl->verbosityLevel = level; }

bool InverseKinematics::initialize()
{
    if (!m_pimpl->initialized) {
        Ipopt::ApplicationReturnStatus status;
//        loader->Options()->SetStringValue("derivative_test", "first-order");
//        loader->Options()->SetStringValue("derivative_test_print_all", "yes");
        m_pimpl->loader->Options()->SetIntegerValue("print_level", m_pimpl->verbosityLevel); //default 5
        m_pimpl->loader->Options()->SetNumericValue("tol", 1e-6); //default 1e-8
        m_pimpl->loader->Options()->SetNumericValue("acceptable_tol", 1e-3); //default 1e-6
        m_pimpl->loader->Options()->SetIntegerValue("acceptable_iter", 5); //default 15
        m_pimpl->loader->Options()->SetIntegerValue("max_iter", m_pimpl->maxIterations);

        status = m_pimpl->loader->Initialize();

        if (status != Ipopt::Solve_Succeeded){
            std::cerr<<"[ERROR] Error during IPOPT solver initialization"<< std::endl;
            return false;
        }
        m_pimpl->initialized = true;
    }
    return true;
}
}
