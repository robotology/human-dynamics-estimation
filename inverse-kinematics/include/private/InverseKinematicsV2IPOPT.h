#ifndef INVERSEKINEMATICSV2IPOPT_H
#define INVERSEKINEMATICSV2IPOPT_H

#include <IpTNLP.hpp>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <IpIpoptApplication.hpp>
#include <Eigen/Core>

namespace human {
    class InverseKinematics;
}

class InverseKinematicsV2IPOPT : public Ipopt::TNLP {
    friend class human::InverseKinematics;

    iDynTree::Model model;
    bool modelLoaded;
    bool gainsLoaded;
    iDynTree::KinDynComputations iKDC;
    iDynTree::FrameIndex parentFrame, endEffectorFrame;
    std::vector< std::pair< double,double > > jointsLimits; //joints upper and lower limits
    int totalDOF;
    iDynTree::FrameFreeFloatingJacobian e_J_we, p_J_wp;
    iDynTree::MatrixDynSize jacobian;
    iDynTree::VectorDynSize jointsTemp;
    iDynTree::Transform p_H_e;
    iDynTree::Position positionResult;
    iDynTree::Vector4 quaternionResult;
    iDynTree::Vector3 kF;
    iDynTree::VectorFixSize<7> pDesired;

    iDynTree::VectorDynSize jointResult;
    iDynTree::VectorDynSize guess;
    iDynTree::Vector3 gains;
    iDynTree::Position desiredPosition;
    iDynTree::Vector4 desiredQuaternion;
    iDynTree::VectorDynSize desiredJoints;
    signed int exitCode;


    void twistToQuaternionTwist(iDynTree::Vector4 &quaternion, iDynTree::MatrixFixSize<7,6>& mapOut);

    void relativeJacobian(iDynTree::MatrixDynSize& jacobianOut);

    void updateAfterOptimizationStep(const Ipopt::Number* x);
    
public:
    
    InverseKinematicsV2IPOPT();
    
    virtual  ~InverseKinematicsV2IPOPT();

    bool loadFromModel(const iDynTree::Model& modelInput, iDynTree::FrameIndex& parentFrameIn, iDynTree::FrameIndex& targetFrame);
    
    bool getModel(iDynTree::Model& modelOuput);

    bool update(const iDynTree::Vector3& gainsIn, const iDynTree::Position &desiredPositionIn, const iDynTree::Vector4 &desiredQuaternionIn, const iDynTree::VectorDynSize &desiredJointsIn);

    bool update();
    
    bool randomInitialization(const double feed, iDynTree::VectorDynSize& guessOut);

    void computeErrors(iDynTree::Position& positionError, iDynTree::Rotation& rotationError, double* angleError);
    
    // IPOPT methods redefinition

    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);

    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda);

    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Number& obj_value);

    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                             Ipopt::Number* grad_f);

    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Index m, Ipopt::Number* g);

    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                            Ipopt::Index *jCol, Ipopt::Number* values);

    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values);

    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                   const Ipopt::Number* x, const Ipopt::Number* z_L,
                                   const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                   const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq);
};

#endif
