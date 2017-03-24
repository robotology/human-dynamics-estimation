#include "HumanDynamicsEstimator.h"

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <yarp/os/LogStream.h> 

#include <iostream>
#include <map>


#define VAR_TO_STR(x) #x

//---------------------------------------------------------------------------
// Utility function for parsing INI file
static bool parseFrameListOption(const yarp::os::Value &option, std::vector<std::string> &parsedJoints);
static bool parseCovarianceMatrixOption(const yarp::os::Value &option, size_t expectedMatrixSize, iDynTree::Triplets &parsedMatrix);


HumanDynamicsEstimator::HumanDynamicsEstimator()
: m_period(0.1) {}

HumanDynamicsEstimator::~HumanDynamicsEstimator() {}

double HumanDynamicsEstimator::getPeriod()
{
    return m_period;
}

bool HumanDynamicsEstimator::configure(yarp::os::ResourceFinder &rf)
{
    // generic module parameters section
    std::string moduleName = rf.check("name", yarp::os::Value("human-dynamics-estimator"), "Checking module name").asString();
    if (moduleName.empty()) {
        yError("Module name cannot be empty");
        return false;
    }
    setName(moduleName.c_str());

    int period = rf.check("period", yarp::os::Value(100), "Checking period in [ms]").asInt();
    m_period = period / 1000.0;

    yarp::os::Value falseValue;
    falseValue.fromString("false");
    bool autoconnect = rf.check("autoconnect", falseValue, "Checking autoconnect option").asBool();

    /*
     * ------Open a port:i for the human joint configuration and forces
     */
    if (!m_humanJointConfigurationPort.open("/" + getName() + "/humanState:i")) {
        yError("Unable to open port ", ("/" + getName() + "/humanState:i").c_str());
        close();
        return false;
    }

    if (!m_humanForcesPort.open("/" + getName() + "/humanForces:i")) {
        yError("Unable to open port ", ("/" + getName() + "/humanForces:i").c_str());
        close();
        return false;
    }

    /*
     * ------Open and prepare a port:o for the output
     */
    if (!m_outputPort.open("/" + getName() + "/dynamicsEstimation:o")) {
        yError("Unable to open port ", ("/" + getName() + "/dynamicsEstimation:o").c_str());
        close();
        return false;
    }


    if (autoconnect) {
        if (!rf.check("humanstateprovider_portname", "Checking state provider output port name")) {
            yError("Name of output port of human state provider has not been specified");
            close();
            return false;
        }

        std::string humanStateRemote = rf.find("humanstateprovider_portname").asString();
        if (!yarp::os::Network::connect(humanStateRemote, m_humanJointConfigurationPort.getName())) {
            yError("Cannot connect %s port to %s", humanStateRemote.c_str(), m_humanJointConfigurationPort.getName().c_str());
            close();
            return false;
        }

        if (!rf.check("humanforcesprovider_portname", "Checking forces provider output port name")) {
            yError("Name of output port of human forces provider has not been specified");
            close();
            return false;
        }
        std::string forcesRemote = rf.find("humanstateprovider_portname").asString();
        if (!yarp::os::Network::connect(forcesRemote, m_humanForcesPort.getName())) {
            yError("Cannot connect %s port to %s", forcesRemote.c_str(), m_humanForcesPort.getName().c_str());
            close();
            return false;
        }
    }
    
    /*
     * ------Human model loading 
     */  
    yarp::os::Value defaultOffline; defaultOffline.fromString("true");
    bool offline = rf.check("playback", defaultOffline, "Checking playback mode").asBool();
    
    std::vector<std::string> joints;

    if (offline && !parseFrameListOption(rf.find("jointsList"), joints)) {
        yError("Error while parsing 'jointsList' parameter");
        return false;
    } else {
        //TODO: read from RPC
    }

    std::string modelFilename = rf.findFile("urdf_model");
    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadReducedModelFromFile(modelFilename, joints)) {
        yError("Could not load the model at %s", modelFilename.c_str());
        close();
        return false;
    }
        
    const iDynTree::Model& humanModel = modelLoader.model();
    

    m_jointsConfiguration.resize(humanModel);
    m_jointsVelocity.resize(humanModel);

    std::string base = rf.find("baseLink").asString();      // base for the model

    /*
     * ------Model sensors initialization
     */
    iDynTree::SensorsList humanSensors = modelLoader.sensors();
    // Remove sensors that are placed on the base (not supported by Berdy)
    humanSensors.removeSensor(iDynTree::ACCELEROMETER, base + "_accelerometer");
    humanSensors.removeSensor(iDynTree::GYROSCOPE, base + "_gyro");

     
    /*
     * ------Setting options and inizialization for Berdy obj
     */
    iDynTree::BerdyOptions berdyOpts;
    berdyOpts.baseLink = base;
    berdyOpts.includeAllNetExternalWrenchesAsSensors          = true;
    berdyOpts.includeAllNetExternalWrenchesAsDynamicVariables = true;
    berdyOpts.includeAllJointAccelerationsAsSensors           = true;
    berdyOpts.includeAllJointTorquesAsSensors                 = false;
    berdyOpts.includeFixedBaseExternalWrench                  = true;
    berdyOpts.checkConsistency();    

    m_berdy.init(humanModel, humanSensors, berdyOpts);


    // Resize internal variables
    size_t numberOfDynVariables = m_berdy.getNrOfDynamicVariables();
    size_t numberOfDynEquations = m_berdy.getNrOfDynamicEquations();
    size_t numberOfMeasurements = m_berdy.getNrOfSensorsMeasurements();

    m_measurements.resize(numberOfMeasurements);
    m_measurements.zero();

    m_expectedDynamicsAPosteriori.resize(numberOfDynVariables);
    m_expectedDynamicsAPosteriori.zero();
    m_covarianceDynamicsAPosterioriInverse.resize(numberOfDynVariables, numberOfDynVariables);
    //TODO: How much reserve to the sparse matrix?!

    // Resize linear system matrices and bias vectors
    m_berdy.resizeAndZeroBerdyMatrices(m_dynamicsConstraintsMatrix, m_dynamicsConstraintsBias,
                                       m_measurementsMatrix, m_measurementsBias);

    m_intermediateQuantities.covarianceDynamicsPriorInverse.resize(numberOfDynVariables, numberOfDynVariables);
    m_intermediateQuantities.expectedDynamicsPrior.resize(numberOfDynVariables);
    m_intermediateQuantities.expectedDynamicsPriorRHS.resize(numberOfDynVariables);
    m_intermediateQuantities.expectedDynamicsAPosterioriRHS.resize(numberOfDynVariables);
    m_intermediateQuantities.expectedDynamicsAPosterioriRHS.zero();

    /*
     *  ------Creating a map to fill the y measurements vector with the order used in berdy
     */
    std::vector<iDynTree::BerdySensor> berdySensors = m_berdy.getSensorsOrdering();
    for (iDynTree::BerdySensor& sensor : berdySensors)
    {
        //construct a key.
        SensorKey key = { sensor.type, sensor.id };
        //check for unexpected presence of this key in the map
        if (m_inputOutputMapping.inputMeasurements.find(key) != m_inputOutputMapping.inputMeasurements.end()) {
            //Key already present. This is not possible
            yWarning("Duplicate sensor of type %s and id %s. Skipping insertion. This may lead to unexpected results", VAR_TO_STR(sensor.type), sensor.id.c_str());
            continue;
        }
        m_inputOutputMapping.inputMeasurements.insert(BerdySensorsInputMap::value_type(key, sensor.range));
    }

    // Resize priors and set them to identity.
    // If a prior is specified in config file they will be cleared after
    iDynTree::Triplets identityTriplets;
    m_priorDynamicsConstraintsCovarianceInverse.resize(numberOfDynEquations, numberOfDynEquations);
    identityTriplets.reserve(numberOfDynEquations);
    identityTriplets.setDiagonalMatrix(0, 0, 1.0, numberOfDynEquations);
    m_priorDynamicsConstraintsCovarianceInverse.setFromTriplets(identityTriplets);

    m_priorDynamicsRegularizationCovarianceInverse.resize(numberOfDynVariables, numberOfDynVariables);
    identityTriplets.clear();
    identityTriplets.reserve(numberOfDynVariables);
    identityTriplets.setDiagonalMatrix(0, 0, 1.0, numberOfDynVariables);
    m_priorDynamicsRegularizationCovarianceInverse.setFromTriplets(identityTriplets);

    m_priorMeasurementsCovarianceInverse.resize(numberOfMeasurements, numberOfMeasurements);
    identityTriplets.clear();
    identityTriplets.reserve(numberOfMeasurements);
    identityTriplets.setDiagonalMatrix(0, 0, 1.0, numberOfMeasurements);
    m_priorMeasurementsCovarianceInverse.setFromTriplets(identityTriplets);

    m_priorDynamicsRegularizationExpectedValue.resize(numberOfDynVariables);
    m_priorDynamicsRegularizationExpectedValue.zero();


    // load priors from config file
    yarp::os::Bottle &priorsGroup = rf.findGroup("PRIORS");
    if (!priorsGroup.isNull()) {
        //priors on dynamics constraints
        if (priorsGroup.check("cov_dyn_constraints", "Checking priors on dynamics constraints covariance: Sigma_D")) {
            yarp::os::Value& covDyn = priorsGroup.find("cov_dyn_constraints");
            iDynTree::Triplets triplets;
            if (!parseCovarianceMatrixOption(covDyn, numberOfDynEquations, triplets)) {
                yWarning("Malformed priors information for \"dynamics constraints covariance: Sigma_D\"");
            } else {
                m_priorDynamicsConstraintsCovarianceInverse.setFromTriplets(triplets);
            }
        }

        //priors on dynamics variables
        if (priorsGroup.check("cov_dyn_variables", "Checking priors on dynamics variables regularization covariance: Sigma_d")) {
            yarp::os::Value& covDyn = priorsGroup.find("cov_dyn_variables");
            iDynTree::Triplets triplets;
            if (!parseCovarianceMatrixOption(covDyn, numberOfDynVariables, triplets)) {
                yWarning("Malformed priors information for \"dynamics variables regularization covariance: Sigma_d\"");
            } else {
                m_priorDynamicsRegularizationCovarianceInverse.setFromTriplets(triplets);
            }
        }

        //priors on dynamics variables expected value
        if (priorsGroup.check("mu_dyn_variables", "Checking priors on dynamics variables regularization expected value: mu_d")) {
            yarp::os::Value& muDyn = priorsGroup.find("mu_dyn_variables");
            if (muDyn.isDouble()) {
                double value = muDyn.asDouble();
                for (size_t index = 0; index < m_priorDynamicsRegularizationExpectedValue.size(); ++index) {
                    m_priorDynamicsRegularizationExpectedValue(index) = value;
                }
            } else if (muDyn.isList() && muDyn.asList()->size() == numberOfDynVariables) {
                yarp::os::Bottle *list = muDyn.asList();
                for (int index = 0; index < list->size(); ++index) {
                    if (!list->get(index).isDouble()) {
                        yWarning("\"mu_d\" - Element at index %d is not a double value", index);
                        continue;
                    }
                    m_priorDynamicsRegularizationExpectedValue(index) = list->get(index).asDouble();
                }
            }
        }

        //priors on measurement equations
        //TODO: decide. This is not still perfect as the user has to know the measurements ordering. we should probably switch to single measurement type.
        // Do it later
        if (priorsGroup.check("cov_measurements", "Checking priors on measurements covariance: Sigma_Y")) {
            yarp::os::Value& covMeas = priorsGroup.find("cov_measurements");
            iDynTree::Triplets triplets;
            if (!parseCovarianceMatrixOption(covMeas, numberOfMeasurements, triplets)) {
                yWarning("Malformed priors information for \"measurements covariance: Sigma_Y\"");
            } else {
                m_priorMeasurementsCovarianceInverse.setFromTriplets(triplets);
            }
        }

    }

    m_gravity.zero(); 
    m_gravity(2) = -9.81;    

    // The permutation pattern should be the same for every configuration
    // Do a first MAP computation with zero state and zero measurements
    m_jointsConfiguration.zero();
    m_jointsVelocity.zero();

    m_berdy.updateKinematicsFromTraversalFixedBase(m_jointsConfiguration, m_jointsVelocity, m_gravity);
    computeMaximumAPosteriori(true);


    /* 
     * ------Creating map for the d dynamics variables vector
     */

    std::vector<iDynTree::BerdyDynamicVariable> berdyDynVariables = m_berdy.getDynamicVariablesOrdering();
    for (iDynTree::BerdyDynamicVariable& variable : berdyDynVariables) {
        BerdyOutputMap *outputMap = nullptr;
        switch(variable.type)
        {
            case iDynTree::LINK_BODY_PROPER_ACCELERATION:
            case iDynTree::NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            case iDynTree::JOINT_WRENCH:
            case iDynTree::NET_EXT_WRENCH:
                outputMap = &m_inputOutputMapping.outputLinks;
                break;
            case iDynTree::DOF_TORQUE:
            case iDynTree::DOF_ACCELERATION:
                outputMap = &m_inputOutputMapping.outputJoints;
                break;
        }
        if (!outputMap) {
            yError("Could not recognize type for dynamic variable %d ", static_cast<int>(variable.type));
            continue;
        }

        // Get a reference to the inner map. If it does not exist
        // it gets created
        BerdyOutputRangeMap &innerMap = (*outputMap)[variable.id];

        if (innerMap.find(variable.type) != innerMap.end()) {
            yWarning("Duplicate variable of type %s and id %s. Skipping insertion. This may lead to unexpected results", VAR_TO_STR(variable.type), variable.id.c_str());
            continue;
        }
        innerMap.insert(BerdyOutputRangeMap::value_type(variable.type, variable.range));
    }

    // Allocate size for output map
    human::HumanDynamics& humanDynamics = m_outputPort.prepare();
    for (auto& linkElement : m_inputOutputMapping.outputLinks) {
        humanDynamics.linkVariables[linkElement.first];
    }
    for (auto& jointElement : m_inputOutputMapping.outputJoints) {
        humanDynamics.jointVariables[jointElement.first];
    }
    m_outputPort.unprepare();
    
    return true;
}

//---------------------------------------------------------------------
bool HumanDynamicsEstimator::updateModule()
{
    using iDynTree::toEigen;
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif

    // Read inputs
    // - Human state
    // - External forces
    human::HumanState *stateReading = m_humanJointConfigurationPort.read(false);
    if (stateReading && !iDynTree::toiDynTree(stateReading->positions, m_jointsConfiguration)) {
        yError("Error while reading human configuration");
        return false;
    }
    if (stateReading && !iDynTree::toiDynTree(stateReading->velocities, m_jointsVelocity)) {
        yError("Error while reading human velocity");
        return false;
    }

    // Read human state
    // Read forces

    human::HumanForces *forcesReading = m_humanForcesPort.read(false);
    if (forcesReading) {
        for (auto &force6D : forcesReading->forces) {
            SensorKey key = {iDynTree::NET_EXT_WRENCH_SENSOR, force6D.appliedLink};
            BerdySensorsInputMap::const_iterator found = m_inputOutputMapping.inputMeasurements.find(key);
            if (found == m_inputOutputMapping.inputMeasurements.end()
                || found->second.size != 6) {
                yError("Applied link %s of force not found or force does not contain 6 elements", force6D.appliedLink.c_str());
                continue;
            }

            m_measurements(found->second.offset) = force6D.fx;
            m_measurements(found->second.offset + 1) = force6D.fy;
            m_measurements(found->second.offset + 2) = force6D.fz;
            m_measurements(found->second.offset + 3) = force6D.ux;
            m_measurements(found->second.offset + 4) = force6D.uy;
            m_measurements(found->second.offset + 5) = force6D.uz;
        }
    }


    //TODO: fill y with measurements from ACCELEROMETER, GYROSCOPE and DOF_ACCELERATION.
    // At this stage they are 0!

    // Now update the measurements vector


    // and the state vector

    /*
     * Set the kinematic information necessary for the dynamics estimation
     */
    m_berdy.updateKinematicsFromTraversalFixedBase(m_jointsConfiguration, m_jointsVelocity, m_gravity);
    
    computeMaximumAPosteriori();

    /*
     * Output
     */
    human::HumanDynamics &output = m_outputPort.prepare();

    //TODO: check if this does not impact performances
    const int linkVariablesSize = 4;
    iDynTree::BerdyDynamicVariablesTypes linkDynamicsVariableTypes[linkVariablesSize] =
    {
        iDynTree::LINK_BODY_PROPER_ACCELERATION,
        iDynTree::NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV,
        iDynTree::JOINT_WRENCH,
        iDynTree::NET_EXT_WRENCH
    };



    // Iterate on thrift data which are ordered map and thus slower for random access
    for (auto &link : output.linkVariables) {
        BerdyOutputMap::const_iterator linkDynamics = m_inputOutputMapping.outputLinks.find(link.first);
        if (linkDynamics == m_inputOutputMapping.outputLinks.end()) {
            yWarning("Link %s not found in the BERDY bodies", link.first.c_str());
            continue;
        }

        yarp::sig::Vector *dynamicsVariableOutput[linkVariablesSize] =
        {
            &link.second.spatialAcceleration,
            &link.second.netWrench,
            &link.second.transmittedWrench,
            &link.second.externalWrench
        };

        for (size_t variableIndex = 0; variableIndex < linkVariablesSize; ++variableIndex) {
            BerdyOutputRangeMap::const_iterator found = linkDynamics->second.find(linkDynamicsVariableTypes[variableIndex]);

            if (found == linkDynamics->second.end()) {
                yWarning("Link %s not found in the BERDY body list", link.first.c_str());
                continue;
            }
            // Safety check: resize vector (hopefully noop)
            dynamicsVariableOutput[variableIndex]->resize(found->second.size);

            Eigen::Map<Eigen::VectorXd> yarpVector(dynamicsVariableOutput[variableIndex]->data(), dynamicsVariableOutput[variableIndex]->size());
            yarpVector = toEigen(m_expectedDynamicsAPosteriori).segment(found->second.offset, found->second.size);
        }

    }

    const int jointVariablesSize = 2;
    iDynTree::BerdyDynamicVariablesTypes jointDynamicsVariableTypes[jointVariablesSize] =
    {
        iDynTree::DOF_TORQUE,
        iDynTree::DOF_ACCELERATION
    };

    for (auto &joint : output.jointVariables) {
        BerdyOutputMap::const_iterator jointDynamics = m_inputOutputMapping.outputJoints.find(joint.first);
        if (jointDynamics == m_inputOutputMapping.outputJoints.end()) {
            yWarning("Joint %s not found in the BERDY bodies", joint.first.c_str());
            continue;
        }

        double *dynamicsVariableOutput[jointVariablesSize] =
        {
            &joint.second.torque,
            &joint.second.acceleration
        };

        for (size_t variableIndex = 0; variableIndex < jointVariablesSize; ++variableIndex) {
            BerdyOutputRangeMap::const_iterator found = jointDynamics->second.find(jointDynamicsVariableTypes[variableIndex]);

            if (found == jointDynamics->second.end() || found->second.size != 1) {
                yWarning("Joint sensor %s-%s not found in the BERDY body list", joint.first.c_str(), VAR_TO_STR(jointDynamicsVariableTypes[variableIndex]));
                continue;
            }

            *dynamicsVariableOutput[variableIndex] = m_expectedDynamicsAPosteriori(found->second.offset);
        }
    }

    m_outputPort.write();

#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
    return true;
}

//---------------------------------------------------------------------
bool HumanDynamicsEstimator::close()
{
    m_outputPort.close();
    m_humanForcesPort.close();
    m_humanJointConfigurationPort.close();
    
    return true;
}

void HumanDynamicsEstimator::computeMaximumAPosteriori(bool computePermutation)
{
    /*
     * Get berdy matrices
     */
    m_berdy.getBerdyMatrices(m_dynamicsConstraintsMatrix,
                             m_dynamicsConstraintsBias,
                             m_measurementsMatrix,
                             m_measurementsBias);


    // Compute the maximum a posteriori probability
    // See Latella et al., "Whole-Body Human Inverse Dynamics with
    // Distributed Micro-Accelerometers, Gyros and Force Sensing" in Sensors, 2016

    // Intermediate quantities

    // Covariance matrix of the prior of the dynamics: var[p(d)], Eq. 10a
    //TODO: find a way to map to iDynTree::SparseMatrix
    m_intermediateQuantities.covarianceDynamicsPriorInverse
    //    toEigen(m_intermediateQuantities.covarianceDynamicsPriorInverse)
    //Better to assign the "sum" before, and adding the product part later
    = toEigen(m_priorDynamicsRegularizationCovarianceInverse);
    m_intermediateQuantities.covarianceDynamicsPriorInverse += toEigen(m_dynamicsConstraintsMatrix).transpose() * toEigen(m_priorDynamicsConstraintsCovarianceInverse) * toEigen(m_dynamicsConstraintsMatrix);

    // decompose m_covarianceDynamicsPriorInverse
    if (computePermutation) {
//        m_intermediateQuantities.covarianceDynamicsPriorInverseDecomposition.analyzePattern(toEigen(m_intermediateQuantities.covarianceDynamicsPriorInverse));
        m_intermediateQuantities.covarianceDynamicsPriorInverseDecomposition.analyzePattern(m_intermediateQuantities.covarianceDynamicsPriorInverse);
    }
//    m_intermediateQuantities.covarianceDynamicsPriorInverseDecomposition.factorize(toEigen(m_intermediateQuantities.covarianceDynamicsPriorInverse));
    m_intermediateQuantities.covarianceDynamicsPriorInverseDecomposition.factorize(m_intermediateQuantities.covarianceDynamicsPriorInverse);

    // Expected value of the prior of the dynamics: E[p(d)], Eq. 10b
    toEigen(m_intermediateQuantities.expectedDynamicsPriorRHS) = toEigen( m_priorDynamicsRegularizationCovarianceInverse) * toEigen(m_priorDynamicsRegularizationExpectedValue) - toEigen(m_dynamicsConstraintsMatrix).transpose() * toEigen(m_priorDynamicsConstraintsCovarianceInverse) * toEigen(m_dynamicsConstraintsBias);

    toEigen(m_intermediateQuantities.expectedDynamicsPrior) =
    m_intermediateQuantities.covarianceDynamicsPriorInverseDecomposition.solve(toEigen(m_intermediateQuantities.expectedDynamicsPriorRHS));

    // Final result: covariance matrix of the whole-body dynamics, Eq. 11a
    //TODO: find a way to map to iDynTree::SparseMatrix
    m_covarianceDynamicsAPosterioriInverse = m_intermediateQuantities.covarianceDynamicsPriorInverse;
    m_covarianceDynamicsAPosterioriInverse += toEigen(m_measurementsMatrix).transpose() * toEigen(m_priorMeasurementsCovarianceInverse) * toEigen(m_measurementsMatrix);

    // decompose m_covarianceDynamicsAPosterioriInverse
    if (computePermutation) {
//        m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.analyzePattern(toEigen(m_covarianceDynamicsAPosterioriInverse));
        m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.analyzePattern(m_covarianceDynamicsAPosterioriInverse);
    }
//    m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.factorize(toEigen(m_covarianceDynamicsAPosterioriInverse));
    m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.factorize(m_covarianceDynamicsAPosterioriInverse);

    // Final result: expected value of the whole-body dynamics, Eq. 11b
    toEigen(m_intermediateQuantities.expectedDynamicsAPosterioriRHS) = (toEigen(m_measurementsMatrix).transpose() * toEigen(m_priorMeasurementsCovarianceInverse) * (toEigen(m_measurements) - toEigen(m_measurementsBias)) + m_intermediateQuantities.covarianceDynamicsPriorInverse * toEigen(m_intermediateQuantities.expectedDynamicsPrior));
    toEigen(m_expectedDynamicsAPosteriori) =
    m_intermediateQuantities.covarianceDynamicsAPosterioriInverseDecomposition.solve(toEigen(m_intermediateQuantities.expectedDynamicsAPosterioriRHS));
}

//---------------------------------------------------------------------------
/*
 * Implementation of the utility function.
 */
static bool parseFrameListOption(const yarp::os::Value &option, std::vector<std::string> &parsedJoints)
{
    if (option.isNull() || !option.isList() || !option.asList()) return false;
    yarp::os::Bottle *frames = option.asList();
    parsedJoints.reserve(static_cast<size_t>(frames->size()));

    for (int i = 0; i < frames->size(); ++i) {
        if (frames->get(i).isString()) {
            parsedJoints.push_back(frames->get(i).asString());
        }
    }
    return true;
}

static bool parseCovarianceMatrixOption(const yarp::os::Value &option,
                                        size_t expectedMatrixSize,
                                        iDynTree::Triplets &parsedMatrix)
{
    parsedMatrix.clear();
    parsedMatrix.reserve(expectedMatrixSize);
    //Two cases: single value (to be replicate on the diagonal, or full diagonal)
    //TODO implement third case: full matrix or, better, a list of Triplets

    if (option.isDouble() && option.asDouble() > 0) {
        parsedMatrix.setDiagonalMatrix(0, 0, 1.0 / option.asDouble(), expectedMatrixSize);
    } else if (option.isList() && option.asList()->size() == expectedMatrixSize) {
        // This is a list of same size of expectedMatrixSize
        yarp::os::Bottle *list = option.asList();
        for (int index = 0; index < list->size(); ++index) {
            if (!list->get(index).isDouble() || list->get(index).asDouble() <= 0) {
                yWarning("Covariance element %d is not a positive double value", index);
                return false;
            }
            parsedMatrix.pushTriplet(iDynTree::Triplet(index, index, 1.0 / list->get(index).asDouble()));
        }

    } else {
        return false;
    }
    return true;
}
