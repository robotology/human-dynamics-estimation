#ifndef HUMANDYNAMICSESTIMATOR_H
#define HUMANDYNAMICSESTIMATOR_H

#include <thrifts/HumanDynamics.h>
#include <thrifts/HumanState.h>
#include <thrifts/HumanForces.h>

#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Estimation/BerdyHelper.h>
#include <iDynTree/KinDynComputations.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

#include <unordered_map>

namespace human
{
    class HumanDynamics;
    class HumanState;
    class HumanForces;
}


class HumanDynamicsEstimator:public yarp::os::RFModule {

private:
    double m_period;
    bool m_firststep;
    
    // buffered port:i for reading from <human-state-provider> module
    yarp::os::BufferedPort<human::HumanState> m_humanJointConfiguration_port; 
//        yarp::os::BufferedPort<yarp::os::Bottle> m_humanJointConfiguration_port; 
    // buffered port:i for reading from <human-forces-provider> module
    yarp::os::BufferedPort<human::HumanForces> m_humanForces_port;
    // buffered port:o from <human-dynamics-estimator> module
    yarp::os::BufferedPort<human::HumanDynamics> m_outputPort;
    
    // Berdy
    iDynTree::BerdyHelper m_berdy;
    
    std::vector<std::string> m_linkName;
    std::vector<std::string> m_jointName;
    
    std::unordered_map<std::string,iDynTree::IndexRange> m_y_map;
    std::vector <iDynTree::BerdyDynamicVariable> m_berdyDynVariables; 
    typedef std::unordered_map<int,iDynTree::IndexRange> BerdyOutputRangeMap;
    typedef std::unordered_map<std::string, BerdyOutputRangeMap > BerdyOutputMap;
    BerdyOutputMap m_link_map;  
    BerdyOutputMap m_joint_map;
    
    // Dimensions
    size_t m_dynV;          // number of dynamic variables
    size_t m_eq;            // number of dynamic equations
    size_t m_meas;          // number of sensors measurements

    // Priors
    iDynTree::SparseMatrix m_sigmaD_prior;
    iDynTree::SparseMatrix m_sigmad_prior;
    iDynTree::SparseMatrix m_sigmay_prior;
    iDynTree::VectorDynSize m_mud_prior;            
    
    // Berdy matrices   
    iDynTree::SparseMatrix m_D;
    iDynTree::VectorDynSize m_bD; 
    iDynTree::SparseMatrix m_Y;
    iDynTree::VectorDynSize m_bY;

    // Measurements matrix
    iDynTree::VectorDynSize m_y;
    
    // Gravity
    iDynTree::Vector3 m_gravity;
    
    // Human state
    iDynTree::KinDynComputations m_humanComputations;    
    iDynTree::JointPosDoubleArray m_q;
    iDynTree::JointDOFsDoubleArray m_dq;

    // MAP matrices
    iDynTree::VectorDynSize m_rhs_BarD;
    iDynTree::VectorDynSize m_mu_BarD;    
    iDynTree::VectorDynSize m_rhs;    
    iDynTree::VectorDynSize m_mu_dgiveny; 
        
    // Cholesky matrices
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > m_CholeskyLU_sigma_BarD_inv;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > m_CholeskyLU_sigma_dgiveny_inv;
        
public:
    /*!
     * Default constructor and destructor
     */    
    HumanDynamicsEstimator();
    virtual ~HumanDynamicsEstimator();
    
    /*!
     * Return the module period.
     */
    double getPeriod();
    
    /*!
     * Module configuration 
     */    
    bool configure(yarp::os::ResourceFinder &rf);
     
    /*!
     * Module updating: at each timestamp it returns the MAP result
     */    
    bool updateModule();
        
     /*!
     * Close module and perform cleanup
     */
    bool close();

};


#endif /* end of include guard: HUMANDYNAMICSESTIMATOR_H */
