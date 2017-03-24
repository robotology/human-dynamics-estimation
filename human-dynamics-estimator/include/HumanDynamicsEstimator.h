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
#include <Eigen/SparseLU>

#include <functional>
#include <unordered_map>
#include <map>

namespace human
{
    class HumanDynamics;
    class HumanState;
    class HumanForces;
}


class HumanDynamicsEstimator : public yarp::os::RFModule {

    //TODO: decide if this map is ok
    //For the time being access the measurements directly using the type-id
    //If in the future we want to access serially to the measurements
    //e.g. all forces, all accelerometers, etc
    //change the map to be a map of map
    struct SensorKey {
        iDynTree::BerdySensorTypes type;
        std::string id;

        bool operator==(const SensorKey& other) const
        {
            return type == other.type && id == other.id;
        }
    };

    struct SensorKeyHash
    {
        std::size_t operator()(const SensorKey& k) const
        {
            // Using the hash as suggested in http://stackoverflow.com/questions/1646807/quick-and-simple-hash-code-combinations/1646913#1646913
            size_t result = 17;
            result = result * 31 + std::hash<int>()(static_cast<int>(k.type));
            result = result * 31 + std::hash<std::string>()(k.id);
            return result;
        }
    };

    struct BerdyDynamicVariablesTypesHash
    {
        std::size_t operator()(const iDynTree::BerdyDynamicVariablesTypes& k) const
        {
            return std::hash<int>()(static_cast<int>(k));
        }
    };


    double m_period;
    
    // input buffered port for reading from <human-state-provider> module
    yarp::os::BufferedPort<human::HumanState> m_humanJointConfigurationPort;
    // inut buffered port for reading from <human-forces-provider> module
    yarp::os::BufferedPort<human::HumanForces> m_humanForcesPort;
    // output buffered port for <human-dynamics-estimator> module
    yarp::os::BufferedPort<human::HumanDynamics> m_outputPort;
    
    // Berdy
    iDynTree::BerdyHelper m_berdy;

    typedef std::unordered_map<SensorKey, iDynTree::IndexRange, SensorKeyHash> BerdySensorsInputMap;
    //The int in the key is actually the iDynTree::BerdyDynamicVariablesTypes enumeration
    typedef std::unordered_map<iDynTree::BerdyDynamicVariablesTypes, iDynTree::IndexRange, BerdyDynamicVariablesTypesHash> BerdyOutputRangeMap;
    typedef std::map<std::string, BerdyOutputRangeMap> BerdyOutputMap;

    struct {
        BerdySensorsInputMap inputMeasurements;
        
        BerdyOutputMap outputLinks;
        BerdyOutputMap outputJoints;
    } m_inputOutputMapping;

    // Priors on regularization, measurements and dynamics constraints
    iDynTree::SparseMatrix m_priorDynamicsConstraintsCovarianceInverse; // Sigma_D
    iDynTree::SparseMatrix m_priorDynamicsRegularizationCovarianceInverse; // Sigma_d
    iDynTree::VectorDynSize m_priorDynamicsRegularizationExpectedValue; // mu_d
    iDynTree::SparseMatrix m_priorMeasurementsCovarianceInverse; // Sigma_y

    // Measurements vector
    iDynTree::VectorDynSize m_measurements;

    // Human state
    iDynTree::JointPosDoubleArray m_jointsConfiguration;
    iDynTree::JointDOFsDoubleArray m_jointsVelocity;

    // E[p(d|y)]: Expected value of A posteriori probability.
    iDynTree::VectorDynSize m_expectedDynamicsAPosteriori;
//    iDynTree::SparseMatrix m_covarianceDynamicsAPosterioriInverse;
    Eigen::SparseMatrix<double, Eigen::RowMajor> m_covarianceDynamicsAPosterioriInverse;

    /*! Full linear system matrices and vectors
     * \f[
     * \begin{bmatrix} D \\ Y \end{bmatrix} d +
     * \begin{bmatrix} bD \\ bY \end{bmatrix} =
     * \begin{bmatrix} 0 \\ y \end{bmatrix}
     * \f]
     */
    iDynTree::SparseMatrix m_dynamicsConstraintsMatrix;
    iDynTree::VectorDynSize m_dynamicsConstraintsBias;
    iDynTree::SparseMatrix m_measurementsMatrix;
    iDynTree::VectorDynSize m_measurementsBias;

    struct {
        // Decompositions of covariance matrices
//        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::RowMajor> > covarianceDynamicsPriorInverseDecomposition;
//        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::RowMajor> > covarianceDynamicsAPosterioriInverseDecomposition;
        Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::RowMajor> > covarianceDynamicsPriorInverseDecomposition;
        Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::RowMajor> > covarianceDynamicsAPosterioriInverseDecomposition;

        Eigen::SparseMatrix<double, Eigen::RowMajor> covarianceDynamicsPriorInverse;
//        iDynTree::SparseMatrix covarianceDynamicsPriorInverse;

        iDynTree::VectorDynSize expectedDynamicsPrior;

        iDynTree::VectorDynSize expectedDynamicsPriorRHS;
        iDynTree::VectorDynSize expectedDynamicsAPosterioriRHS;

    } m_intermediateQuantities;

    // Gravity
    iDynTree::Vector3 m_gravity;

    void computeMaximumAPosteriori(bool computePermutation = false);

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
