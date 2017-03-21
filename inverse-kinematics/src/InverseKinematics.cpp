#include "InverseKinematics.h"
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>

InverseKinematics::InverseKinematics()
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

InverseKinematics::InverseKinematics(const std::string& solverName)
: updated(false)
, initialized(false)
, alreadyOptimized(false)
, verbosityLevel(2)
, maxIterations(3000)
{
    solverPointer = new InverseKinematicsV2IPOPT();
    loader = IpoptApplicationFactory();
    loader->Options()->SetStringValue("hessian_approximation", "limited-memory");
    loader->Options()->SetStringValue("linear_solver", solverName);
}


InverseKinematics::~InverseKinematics()
{
}

InverseKinematics::InverseKinematics(const InverseKinematics&) { assert(false); }
InverseKinematics& InverseKinematics::operator=(const InverseKinematics&) { assert(false); return *this;}

bool InverseKinematics::getReducedModel(const iDynTree::Model& fullModel, const std::vector< std::string >& consideredJoints, iDynTree::Model& modelOutput)
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

bool InverseKinematics::getFrameIndeces(const std::string& parentFrame, const std::string& endEffectorFrame, const iDynTree::Model model, iDynTree::FrameIndex& parentFrameIndex, iDynTree::FrameIndex& endEffectorFrameIndex)
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

void InverseKinematics::removeUnsupportedJoints(const iDynTree::Model& modelInput, iDynTree::Model& modelOutput)
{
    iDynTree::ModelLoader loader;
    std::vector< std::string > consideredJoints;
    //int selectedJoints = 0;
    //jointMap.zero();
    
    for(int i=0; i < modelInput.getNrOfJoints(); ++i){
        if(modelInput.getJoint(i)->getNrOfDOFs() == 1){
            
            consideredJoints.push_back(modelInput.getJointName(i));
            
            //jointMap.resize(jointMap.size() + 1);
            //jointMap(selectedJoints) = i;
            //++selectedJoints;
        }
        else std::cerr << "Joint " << modelInput.getJointName(i) << " ignored (" << modelInput.getJoint(i)->getNrOfDOFs() << " DOF)" << std::endl;
    }
    loader.loadReducedModelFromFullModel(modelInput, consideredJoints);
    modelOutput = loader.model();
}

bool InverseKinematics::autoSelectJointsFromTraversal(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, iDynTree::Model& modelOutput)
{
    iDynTree::FrameIndex parent;
    iDynTree::FrameIndex endEffector;
    bool success;
    
    
    //std::cerr << "[IK] Automatically select joints between "<< parentFrame << " and " << endEffectorFrame << std::endl;
    
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
    
//    std::cerr << "[IK] Considered joints are:"<< std::endl;
//    for (std::vector< std::string >::const_iterator i = consideredJointsAuto.begin(); i != consideredJointsAuto.end(); ++i){
//        std::cerr <<"->"<< *i << std::endl;
//    }
//    std::cerr << std::endl;
    
    success = getReducedModel(modelInput, consideredJointsAuto, modelOutput);
    if(!success)
        return false;
    
    return true;
}



bool InverseKinematics::setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame, const std::vector< std::string >& consideredJoints)
{
    alreadyOptimized = false;

    bool success= false;
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
        
    if ((!consideredJoints.empty())&&((consideredJoints[0].compare("All") == 0)||(consideredJoints[0].compare("ALL") == 0)||(consideredJoints[0].compare("all") == 0))){
        
        removeUnsupportedJoints(model.copy(),model);
        success = getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false; 

    } else if (!consideredJoints.empty()){
        
        success = getReducedModel(model, consideredJoints, model);
        if(!success)
            return false;
        
        removeUnsupportedJoints(model.copy(),model);
        
        success = getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false;

    } else {

        success = autoSelectJointsFromTraversal(model.copy(), parentFrame, endEffectorFrame, model);
        if(!success)
            return false;
        
        removeUnsupportedJoints(model.copy(),model);
        
        success = getFrameIndeces(parentFrame, endEffectorFrame, model, parent, endEffector);
        if(!success)
            return false;
        
    }
    
    _parentFrame = parentFrame;
    _targetFrame = endEffectorFrame;
    tempGuess.resize(model.getNrOfDOFs());
    lastSolution.resize(model.getNrOfDOFs());
    
    return solverPointer->loadFromModel(model, parent, endEffector);
}

bool InverseKinematics::setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame)
{
    alreadyOptimized = false;
    
    std::vector< std::string > emptyVector;
    
    emptyVector.resize(0);
    
    return setModelfromURDF(URDFfilename, parentFrame, endEffectorFrame, emptyVector);
} 



bool InverseKinematics::setModel(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, const bool autoSelectJoints)
{
    iDynTree::FrameIndex parentFrameIndex;
    iDynTree::FrameIndex endEffectorFrameIndex;
    iDynTree::Model model;
    
    alreadyOptimized = false;
        
    if(autoSelectJoints){
        if(!autoSelectJointsFromTraversal(modelInput, parentFrame, endEffectorFrame, model))
            return false;
    }
    
    removeUnsupportedJoints(model.copy(),model);
    
    if(!getFrameIndeces(parentFrame,endEffectorFrame, model, parentFrameIndex, endEffectorFrameIndex))
        return false;

    _parentFrame = parentFrame;
    _targetFrame = endEffectorFrame;
    tempGuess.resize(model.getNrOfDOFs());
    lastSolution.resize(model.getNrOfDOFs());
    
    return solverPointer->loadFromModel(model, parentFrameIndex, endEffectorFrameIndex);
}

void InverseKinematics::setWeights(const iDynTree::Vector3& weights)
{
    solverPointer->gains = weights;
    updated = false;
}

void InverseKinematics::setDesiredPosition(const iDynTree::Position& desiredPosition)
{
    solverPointer->desiredPosition = desiredPosition;
    updated = false;
}

void InverseKinematics::setDesiredQuaternion(const iDynTree::Vector4& desiredQuaternion)
{
    solverPointer->desiredQuaternion = desiredQuaternion;
    updated = false;
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
    solverPointer->desiredJoints = desiredJoints;
    updated = false;
}

void InverseKinematics::setGuess(const iDynTree::VectorDynSize& guess)
{
    solverPointer->guess = guess;
}

bool InverseKinematics::setRandomGuess(const double feed, iDynTree::VectorDynSize& guess)
{
    return solverPointer->randomInitialization(feed, guess);
}

bool InverseKinematics::setRandomGuess(const double feed, iDynTree::VectorDynSize& guess, const iDynTree::VectorDynSize& oldGuess, double minDistance, const int maxIter)
{
    bool success = false;
    int iter = 0;
    double norm = 0;

    do{
        success = setRandomGuess(feed + iter + 1, tempGuess);
        if((tempGuess.size() != oldGuess.size())|| !success ||(tempGuess.size() == 0))
            return false;
        norm = (iDynTree::toEigen(tempGuess) - iDynTree::toEigen(oldGuess)).norm()/tempGuess.size();
        //std::cerr << "[IK - setRandomGuess] Norm:"<< norm << std::endl;
        iter++;
    }while((iter < maxIter)&&(norm < minDistance));
    
    if(iter == maxIter)
        std::cerr << "[IK - setRandomGuess] Maximum iteration reached." << std::endl;
    
    guess = tempGuess;
    
    return true;
}


bool InverseKinematics::update(const iDynTree::Vector3& weights, const iDynTree::Position& desiredPosition, const iDynTree::Vector4& desiredQuaternion, const iDynTree::VectorDynSize& desiredJoints)
{
    updated = solverPointer->update(weights, desiredPosition, desiredQuaternion, desiredJoints);
    return updated;
}

bool InverseKinematics::update()
{
    updated = solverPointer->update();
    return updated;
}

bool InverseKinematics::getErrors(iDynTree::Position& positionError, iDynTree::Rotation& rotationError, double* angleError)
{
    if(!alreadyOptimized)
        return false;
    
    solverPointer->computeErrors(positionError,rotationError,angleError);
    
    return true;
}

void InverseKinematics::getFrames(std::string& parentFrame, std::string& endEffectorFrame)
{
    parentFrame = _parentFrame;
    endEffectorFrame = _targetFrame;
}

bool InverseKinematics::getConsideredJoints(std::vector< std::string >& consideredJoints)
{
    iDynTree::Model _model;
    if(!solverPointer->getModel(_model))
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
    jointsOut = lastSolution;
    return result;
}

signed int InverseKinematics::runIK()
{
    if (!initialized){
        if (!initialize()) return -6;
    }

    if(!updated){
        bool success;
        success = update();
        if(!success){
            std::cerr << "[ERROR] Error when trying to update IK data" << std::endl;
            return -6;
        }
    }

    if (alreadyOptimized){
        // std::cerr << "Reoptimize!!" << std::endl;
        loader->ReOptimizeTNLP(solverPointer);
        lastSolution = solverPointer->jointResult;
        return solverPointer->exitCode;
    } else {
        //std::cerr << "Optimize!!" << std::endl;
        loader->OptimizeTNLP(solverPointer);
        if (solverPointer->exitCode >=0 
            || solverPointer->exitCode == -2) {
                alreadyOptimized = true;
        }
        lastSolution = solverPointer->jointResult;
        return solverPointer->exitCode;
    }
}

iDynTree::VectorDynSize& InverseKinematics::getLastSolution()
{
    return lastSolution;
}

void InverseKinematics::setVerbosityLevel(unsigned int level) { verbosityLevel = level; }

bool InverseKinematics::initialize()
{
    if (!initialized) {
        Ipopt::ApplicationReturnStatus status;
//        loader->Options()->SetStringValue("derivative_test", "first-order");
//        loader->Options()->SetStringValue("derivative_test_print_all", "yes");
        loader->Options()->SetIntegerValue("print_level", verbosityLevel); //default 5
        loader->Options()->SetNumericValue("tol", 1e-6); //default 1e-8
        loader->Options()->SetNumericValue("acceptable_tol", 1e-3); //default 1e-6
        loader->Options()->SetIntegerValue("acceptable_iter", 5); //default 15
        loader->Options()->SetIntegerValue("max_iter", maxIterations);

        status = loader->Initialize();

        if (status != Ipopt::Solve_Succeeded){
            std::cerr<<"[ERROR] Error during IPOPT solver initialization"<< std::endl;
            return false;
        }
        initialized = true;
    }
    return true;
}
