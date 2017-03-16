#include "InverseKinematicsV2IPOPT.h"
#include <iDynTree/Core/EigenHelpers.h>
#include <string>
#include <vector>
#include <cmath>
#include <cassert>

using namespace std;
using namespace iDynTree;
using namespace Ipopt;

InverseKinematicsV2IPOPT::InverseKinematicsV2IPOPT()
: modelLoaded(false)
, gainsLoaded(false)
, parentFrame(0)
, endEffectorFrame(0)
, kJoints(0)
, exitCode(-6)
{
    positionResult.zero();
    quaternionResult.zero();
    kF.zero();
}

InverseKinematicsV2IPOPT::~InverseKinematicsV2IPOPT()
{}

bool InverseKinematicsV2IPOPT::loadFromModel(const Model& modelInput, FrameIndex& parentFrameIn, FrameIndex& targetFrame)
{
    parentFrame = parentFrameIn;
    endEffectorFrame = targetFrame;
    model = modelInput;
    
    iKDC.loadRobotModel(model);
    
    jointsLimits.reserve(model.getNrOfDOFs());
    
    for (iDynTree::JointIndex j = 0; j < model.getNrOfDOFs(); j++) {
            std::pair<double, double> limits; //first is min, second max
            iDynTree::IJointPtr joint;
            joint = model.getJoint(j);
            if(joint->hasPosLimits()){
                joint->getPosLimits(0,limits.first,limits.second); //no joints with two DOF can be considered. Deal with it accordingly in the URDF
            }
            else {
                limits.first = -2*M_PI;
                limits.second = 2*M_PI;
            }
            jointsLimits.push_back(limits);
        }
    
    /*std::cerr << "Joints Limits:" << std::endl;
    for(int i = 0; i < jointsLimits.size(); ++i){
        std::cerr<< jointsLimits[i].first<<" < "<< model.getJointName(i) << " < " << jointsLimits[i].second << " ";
    }
    std::cerr << std::endl;*/
    
    desiredJoints.resize(model.getNrOfDOFs());
    desiredJoints.zero();
    
    jointResult.resize(model.getNrOfDOFs());
    jointResult.zero();
    
    jointsTemp.resize(model.getNrOfDOFs());
    
    totalDOF = model.getNrOfDOFs();
    
    p_J_wp.resize(iKDC.model());
    e_J_we.resize(iKDC.model());
    
    jacobian.resize(6,totalDOF);
    
    modelLoaded = true;
    return true;
}

bool InverseKinematicsV2IPOPT::getModel(Model& modelOuput)
{
    if(!modelLoaded)
        return false;

    modelOuput = model;
    return true;
}


bool InverseKinematicsV2IPOPT::update(const Vector3& gainsIn, const Position& desiredPositionIn, const Vector4& desiredQuaternionIn, const VectorDynSize& desiredJointsIn)
{
    if(!modelLoaded){
       std::cerr<<"[ERROR] First you have to load a model"<< std::endl;
       return false;
    }

    if(desiredJointsIn.size() != (totalDOF)){
        std::cerr<<"[ERROR] Dimension of desired joints vector different than the number of considered joints"<<desiredJointsIn.size()<<"!="<<model.getNrOfDOFs()<< std::endl;
        return false;
    }

    desiredPosition = desiredPositionIn;
    desiredJoints = desiredJointsIn;
    
    if( (toEigen(desiredQuaternionIn).norm() < 0.95) || (toEigen(desiredQuaternionIn).norm() > 1.05) ){
        std::cerr << "[ERROR] Not unitary quaternion" << std::endl;
        return false;
    }
        
    
    desiredQuaternion = desiredQuaternionIn;
    gains = gainsIn;
    
    kJoints = gains(2);
    
    toEigen(kF).block<3,3>(0,0) = toEigen(kF).block<3,3>(0,0).setIdentity()*gains(0);
    toEigen(kF).block<4,4>(3,3) = toEigen(kF).block<4,4>(3,3).setIdentity()*gains(1);
    
    toEigen(pDesired).segment<3>(0) = toEigen(desiredPosition);
    toEigen(pDesired).segment<4>(3) = toEigen(desiredQuaternion);
    
    //std::cerr << "pDesired = "<< pDesired.toString() << std::endl;
    
    gainsLoaded = true;
    return true;
}

bool InverseKinematicsV2IPOPT::update()
{
    return update(gains,desiredPosition,desiredQuaternion,desiredJoints);

}

bool InverseKinematicsV2IPOPT::randomInitialization(const double feed, VectorDynSize& guessOut)
{
    if(!modelLoaded)
        return false;
    
    srand (feed);
    guess.resize(totalDOF);
    for(int i=0; i < guess.size(); ++i){
        guess(i) = jointsLimits[i].first + ( (double) rand() / RAND_MAX )*(jointsLimits[i].second - jointsLimits[i].first);
    }
    guessOut = guess;
    return true;
}

void InverseKinematicsV2IPOPT::twistToQuaternionTwist(Vector4& quaternion, MatrixFixSize< 7, 6 >& mapOut)
{
    MatrixFixSize<7,6> map;
    MatrixFixSize<3,4> omegaToQuat;
    Eigen::Vector3d imQuat;
    IndexRange rowRange, columnRange;
    
    imQuat = toEigen(quaternion).tail(3);
    
    omegaToQuat.zero();
    
    toEigen(omegaToQuat).leftCols<1>() = -imQuat;
    toEigen(omegaToQuat).block<3,3>(0,1).setIdentity();
    toEigen(omegaToQuat).block<3,3>(0,1) *= quaternion(0);
    toEigen(omegaToQuat).block<3,3>(0,1) += skew(imQuat);
    toEigen(omegaToQuat) *= 0.5;
    
    map.zero();
    
    rowRange.offset = 0;
    rowRange.size = 3;
    columnRange.offset = 0;
    columnRange.size = 3;
    
    setSubMatrixToIdentity(map,rowRange,columnRange);
    
    rowRange.offset = 3;
    rowRange.size = 4;
    columnRange.offset = 3;
    columnRange.size = 3;
    setSubMatrix(map,rowRange,columnRange,toEigen(omegaToQuat).transpose());

    mapOut = map;
}

void InverseKinematicsV2IPOPT::relativeJacobian(const VectorDynSize& configuration, MatrixDynSize& jacobianOut)
{
    Matrix6x6 right2mixed, left2mixed;
    
    iKDC.setJointPos(configuration);
    iKDC.setFrameVelocityRepresentation(BODY_FIXED_REPRESENTATION); //left trivialized velocity
    
    left2mixed = iKDC.getRelativeTransformExplicit(endEffectorFrame,parentFrame,endEffectorFrame,endEffectorFrame).asAdjointTransform(); //is the adjoint transformation from left-trivialized velocity to mixed velocity with the origin on the target frame and the orientation of the parent frame
    right2mixed = iKDC.getRelativeTransformExplicit(endEffectorFrame,parentFrame,parentFrame,parentFrame).asAdjointTransform(); //is the adjoit trasnformation from right-trivialized velocity to mixed velocity. It comes from a multiplication of two adjoints: from left to mixed times from right to left
    
    iKDC.getFrameFreeFloatingJacobian(parentFrame, p_J_wp); //getting the jacobian from world to parent frame with left-trivialized velocity representation
    iKDC.getFrameFreeFloatingJacobian(endEffectorFrame, e_J_we);  //getting the jacobian from world to target frame with left-trivialized velocity representation
    
    toEigen(jacobianOut) = toEigen(left2mixed)*(toEigen(e_J_we).rightCols(totalDOF)) - toEigen(right2mixed)*(toEigen(p_J_wp).rightCols(totalDOF));
}


bool InverseKinematicsV2IPOPT::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    if(!modelLoaded){
        std::cerr<<"[ERROR] First you have to load the model"<< std::endl;
        return false;
    }
    if(!gainsLoaded){
        std::cerr<<"[ERROR] First you have to define cost function gains"<< std::endl;
        return false;
    }
    
    n = totalDOF; //position and orientation of the end effector (3 for position and 4 for the quaternion of the orientation) + joints'DoF, in this order.
    m = 0; //7 for forward kinematics (relates the 7 dofs of the end effector with the joint dofs) + 1 on the norm of the quaternion.
    
    nnz_jac_g = 0;
    nnz_h_lag = n*n; //dense
    
    index_style = Ipopt::TNLP::C_STYLE;
    
    return true;
}

bool InverseKinematicsV2IPOPT::get_bounds_info(Ipopt::Index n, Number* x_l, Number* x_u, Ipopt::Index m, Number* g_l, Number* g_u)
{
    
    for (Ipopt::Index j = 0; j < totalDOF; j++){
        x_l[j] = jointsLimits[j].first; x_u[j] = jointsLimits[j].second; 
    }
   
    return true;
}

bool InverseKinematicsV2IPOPT::get_starting_point(Ipopt::Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U, Ipopt::Index m, bool init_lambda, Number* lambda)
{
    if(init_z) return false;
    if(init_lambda) return false;
 
    if(init_x){
        if(guess.size() == totalDOF){
                for(int i = 0; i < totalDOF; i++){
                    x[i] = guess(i);
                }
            //guess.resize(0);
        }
        
        else{
            if(guess.size() >0){
                std::cerr << "[IK WARNING] The guess dimension is different from the number of DOFs: "<< guess.size() << "!=" <<totalDOF-7 <<". Guess ignored." << std::endl;
            }
                for(int i = 0; i < totalDOF; i++){
                    x[i] = desiredJoints(i);
                }
        }
    }
   /* std::cerr << "STARTING POINT " << std::endl;
    for(int i =0; i < totalDOF; ++i){
        std::cerr << x[i] << " ";
    }
    std::cerr <<std::endl<<std::endl;*/
    return true;
}

bool InverseKinematicsV2IPOPT::eval_f(Ipopt::Index n, const Number* x, bool new_x, Number& obj_value)
{
    Eigen::Map< const Eigen::VectorXd > x_in (x, totalDOF);
    iDynTree::VectorFixSize<7> p; //position + quaternion
    iDynTree::Transform p_H_e; //forward kinematics
    
    toEigen(jointsTemp) = x_in;
    
    //std::cerr << "Print of q: "<< jointsTemp.toString() << std::endl;
    
    iKDC.setJointPos(jointsTemp);
    p_H_e = iKDC.getRelativeTransform(parentFrame,endEffectorFrame);
    
    toEigen(p).segment<3>(0) = toEigen(p_H_e.getPosition());
    toEigen(p).segment<4>(3) = toEigen(p_H_e.getRotation().asQuaternion());
    
    //std::cerr << "Print of p: "<< p.toString() << std::endl;
    
    
    double obj_value1 = 0.5*toEigen(p).transpose() * toEigen(kF) * toEigen(p);
    double obj_value2 = - toEigen(pDesired).transpose() * toEigen(kF) * toEigen(p); 
    double obj_value3 =  0.5*kJoints* x_in.transpose() * x_in;
    double obj_value4 = - kJoints * toEigen(desiredJoints).transpose() * x_in;
    
    obj_value = obj_value1 + obj_value2 + obj_value3 + obj_value4; 
    return true;
}

bool InverseKinematicsV2IPOPT::eval_grad_f(Ipopt::Index n, const Number* x, bool new_x, Number* grad_f)
{
    Eigen::Map< const Eigen::VectorXd > x_in (x, totalDOF);
    Eigen::Map< Eigen::VectorXd > Grad_f (grad_f, totalDOF);
    MatrixFixSize<7,6> map;
    iDynTree::Transform p_H_e;
    iDynTree::VectorFixSize<7> p; //position + quaternion
    Vector4 quaternion;
    
    toEigen(jointsTemp) = x_in;
    iKDC.setJointPos(jointsTemp);
    p_H_e = iKDC.getRelativeTransform(parentFrame,endEffectorFrame);
    
    toEigen(p).segment<3>(0) = toEigen(p_H_e.getPosition());
    toEigen(p).segment<4>(3) = toEigen(p_H_e.getRotation().asQuaternion());
    
    quaternion = p_H_e.getRotation().asQuaternion();
    
    twistToQuaternionTwist(quaternion, map);
    relativeJacobian(jointsTemp,jacobian);
    
    Grad_f = toEigen(jacobian).transpose() * toEigen(map).transpose() * toEigen(kF) * (toEigen(p) - toEigen(pDesired)) + kJoints*(x_in -toEigen(desiredJoints)); 
    
    return true;
}

bool InverseKinematicsV2IPOPT::eval_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Number* g)
{

    return false;

}

bool InverseKinematicsV2IPOPT::eval_jac_g(Ipopt::Index n, const Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Number* values)
{
    return false;
}

void InverseKinematicsV2IPOPT::finalize_solution(SolverReturn status, Ipopt::Index n, const Number* x, const Number* z_L, const Number* z_U, Ipopt::Index m, const Number* g, const Number* lambda, Number obj_value, const IpoptData* ip_data, IpoptCalculatedQuantities* ip_cq)
{
    if((status == Ipopt::SUCCESS)||status == Ipopt::STOP_AT_ACCEPTABLE_POINT){
        Eigen::Map< const Eigen::VectorXd > x_in (x, totalDOF);
        iDynTree::Transform p_H_e;
        
        toEigen(jointResult) = x_in;
        
        iKDC.setJointPos(jointResult);
        p_H_e = iKDC.getRelativeTransform(parentFrame,endEffectorFrame);
        
        positionResult = p_H_e.getPosition();
        
        quaternionResult = p_H_e.getRotation().asQuaternion();
        
        //std::cerr << "[IK Result] Cost value: "<< obj_value << std::endl;

        if(status == Ipopt::SUCCESS){
            exitCode = 0;
        }
        else exitCode = 1;
    }
    else {
        switch(status){
            
            case Ipopt::LOCAL_INFEASIBILITY:
                exitCode = -1;
                break;
                
            case Ipopt::MAXITER_EXCEEDED:
            case Ipopt::CPUTIME_EXCEEDED:
            case Ipopt::STOP_AT_TINY_STEP: //PREMATURE STOP
                exitCode = -2;
                break;
                
            case Ipopt::INVALID_NUMBER_DETECTED: //WRONG DATA INSERTION
                exitCode = -3;
                break;
                
            case Ipopt::DIVERGING_ITERATES:
            case Ipopt::INTERNAL_ERROR:
            case Ipopt::ERROR_IN_STEP_COMPUTATION:
            case Ipopt::RESTORATION_FAILURE: //IPOPT INTERNAL PROBLEM
                exitCode = -4;
                break;
                
            case Ipopt::USER_REQUESTED_STOP: //USER REQUESTED STOP
                exitCode = -5;
                break;
                
            default:
                exitCode = -6;

        }
    }
    guess = desiredJoints;
}

bool InverseKinematicsV2IPOPT::eval_h(Ipopt::Index n, const Number* x, bool new_x, Number obj_factor, Ipopt::Index m, const Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Number* values)
{
    //return Ipopt::TNLP::eval_h(n, x, new_x, obj_factor, m, lambda, new_lambda, nele_hess, iRow, jCol, values);
    return false;
}

void InverseKinematicsV2IPOPT::computeErrors(Position& positionError, Rotation& rotationError, double* angleError)
{
    Rotation endEffectorOrientation, actualEndEffectorRotation;

    toEigen(positionError) = toEigen(desiredPosition) - toEigen(positionResult);

    endEffectorOrientation.fromQuaternion(desiredQuaternion);
    actualEndEffectorRotation.fromQuaternion(quaternionResult);
    rotationError = actualEndEffectorRotation*(endEffectorOrientation.inverse());

    *angleError = acos(abs(toEigen(quaternionResult).dot(toEigen(desiredQuaternion).transpose()))); //Metrics for 3D rotation by Du Q.Huynh
}
