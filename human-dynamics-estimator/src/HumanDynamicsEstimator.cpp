#include "HumanDynamicsEstimator.h"

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <yarp/os/LogStream.h> 

#include <chrono>
#include <iostream>
#include <map>


#define VAR_TO_STR(x) #x

//---------------------------------------------------------------------------
// Utility function for parsing INI file
static bool parseSensorsRemovalOptionAndRemoveSensors(const yarp::os::Bottle &option, iDynTree::SensorsList& sensorList);
static bool parseFrameListOption(const yarp::os::Value &option, std::vector<std::string> &parsedJoints);
static bool parseMeasurementsPriorsOption(const yarp::os::Bottle& priorsGroup,
                                          const iDynTree::BerdyHelper& berdy,
                                          const std::string& optionPrefix,
                                          iDynTree::SparseMatrix& parseMatrix);
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
        yError("Unable to open port %s", ("/" + getName() + "/humanState:i").c_str());
        close();
        return false;
    }

    if (!m_humanForcesPort.open("/" + getName() + "/humanForces:i")) {
        yError("Unable to open port %s", ("/" + getName() + "/humanForces:i").c_str());
        close();
        return false;
    }

    /*
     * ------Open and prepare a port:o for the output
     */
    if (!m_outputPort.open("/" + getName() + "/dynamicsEstimation:o")) {
        yError("Unable to open port %s", ("/" + getName() + "/dynamicsEstimation:o").c_str());
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
        std::string forcesRemote = rf.find("humanforcesprovider_portname").asString();
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

    //Parse which sensor should be removed from the sensors list
    if (!parseSensorsRemovalOptionAndRemoveSensors(rf.findGroup("SENSORS_REMOVAL"), humanSensors)) {
        yWarning("Failed while parsing sensors removal option. All sensors will be considered");
    }


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
            yWarning("Duplicate sensor of id[type]: %s[%d]. Skipping insertion. This may lead to unexpected results", sensor.id.c_str(), sensor.type);
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
        if (!parseMeasurementsPriorsOption(priorsGroup,
                                           m_berdy,
                                           "cov_measurements",
                                           m_priorMeasurementsCovarianceInverse)) {
            yWarning("Problem parsing priors on measurements constraints. Default to 1");
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

    const std::vector<iDynTree::BerdyDynamicVariable>& berdyDynVariables = m_berdy.getDynamicVariablesOrdering();
    for (const iDynTree::BerdyDynamicVariable& variable : berdyDynVariables) {
        BerdyOutputMap *outputMap = nullptr;
        switch(variable.type)
        {
            case iDynTree::LINK_BODY_PROPER_ACCELERATION:
            case iDynTree::NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            case iDynTree::NET_EXT_WRENCH:
                outputMap = &m_inputOutputMapping.outputLinks;
                break;
            case iDynTree::JOINT_WRENCH:
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
            yWarning("Duplicate variable of id[type]: %s[%d]. Skipping insertion. This may lead to unexpected results", variable.id.c_str(), variable.type);
            continue;
        }
        innerMap.insert(BerdyOutputRangeMap::value_type(variable.type, variable.range));
    }

    // Allocate size for output vectors
    human::HumanDynamics& humanDynamics = m_outputPort.prepare();
    humanDynamics.linkVariables.reserve(m_inputOutputMapping.outputLinks.size());
    humanDynamics.jointVariables.reserve(m_inputOutputMapping.outputJoints.size());

    for (size_t i = 0; i < m_inputOutputMapping.outputLinks.size(); ++i) {
        human::LinkDynamicsEstimation link;
        link.spatialAcceleration.resize(6);
        link.spatialAcceleration.zero();
        link.externalWrench.resize(6);
        link.externalWrench.zero();
        link.netWrench.resize(6);
        link.netWrench.zero();
        humanDynamics.linkVariables.push_back(link);
    }

    for (size_t i = 0; i < m_inputOutputMapping.outputJoints.size(); ++i) {
        human::JointDynamicsEstimation joint;
        joint.acceleration.resize(1);
        joint.acceleration.zero();
        joint.torque.resize(1);
        joint.torque.zero();
        joint.transmittedWrench.resize(6);
        joint.transmittedWrench.zero();
        humanDynamics.jointVariables.push_back(joint);
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

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
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
    m_measurements.zero();
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
    output.linkVariables.resize(m_inputOutputMapping.outputLinks.size());

    const int linkVariablesSize = 3;
    iDynTree::BerdyDynamicVariablesTypes linkDynamicsVariableTypes[linkVariablesSize] =
    {
        iDynTree::LINK_BODY_PROPER_ACCELERATION,
        iDynTree::NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV,
        iDynTree::NET_EXT_WRENCH
    };

    size_t index = 0;
    for (auto &link : m_inputOutputMapping.outputLinks) {
        human::LinkDynamicsEstimation &linkEstimation = output.linkVariables[index];
        linkEstimation.linkName = link.first;
        yarp::sig::Vector *dynamicsVariableOutput[linkVariablesSize] =
        {
            &linkEstimation.spatialAcceleration,
            &linkEstimation.netWrench,
            &linkEstimation.externalWrench
        };

        for (size_t variableIndex = 0; variableIndex < linkVariablesSize; ++variableIndex) {
            BerdyOutputRangeMap::const_iterator found = link.second.find(linkDynamicsVariableTypes[variableIndex]);

            if (found == link.second.end()) {
                // Not needed to emit a warning here. Some variable can be absent
                continue;
            }
            // Safety check: resize vector (hopefully noop)
            dynamicsVariableOutput[variableIndex]->resize(found->second.size);

            Eigen::Map<Eigen::VectorXd> yarpVector(dynamicsVariableOutput[variableIndex]->data(), dynamicsVariableOutput[variableIndex]->size());
            yarpVector = toEigen(m_expectedDynamicsAPosteriori).segment(found->second.offset, found->second.size);
        }

        index++;
    }

    // Joints
    output.jointVariables.resize(m_inputOutputMapping.outputJoints.size());

    const int jointVariablesSize = 3;
    iDynTree::BerdyDynamicVariablesTypes jointDynamicsVariableTypes[jointVariablesSize] =
    {
        iDynTree::JOINT_WRENCH,
        iDynTree::DOF_TORQUE,
        iDynTree::DOF_ACCELERATION
    };

    index = 0;
    for (auto &joint : m_inputOutputMapping.outputJoints) {
        human::JointDynamicsEstimation &jointEstimation = output.jointVariables[index];
        jointEstimation.jointName = joint.first;
        yarp::sig::Vector *dynamicsVariableOutput[jointVariablesSize] =
        {
            &jointEstimation.transmittedWrench,
            &jointEstimation.torque,
            &jointEstimation.acceleration
        };

        for (size_t variableIndex = 0; variableIndex < jointVariablesSize; ++variableIndex) {
            BerdyOutputRangeMap::const_iterator found = joint.second.find(jointDynamicsVariableTypes[variableIndex]);

            if (found == joint.second.end()) {
                // Not needed to emit a warning here. Some variable can be absent
                continue;
            }
            // Safety check: resize vector (hopefully noop)
            dynamicsVariableOutput[variableIndex]->resize(found->second.size);

            Eigen::Map<Eigen::VectorXd> yarpVector(dynamicsVariableOutput[variableIndex]->data(), dynamicsVariableOutput[variableIndex]->size());
            yarpVector = toEigen(m_expectedDynamicsAPosteriori).segment(found->second.offset, found->second.size);
        }

        index++;
    }

    m_outputPort.write();
    std::cerr << "Map took " <<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t1).count() << "ms" << std::endl;

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
static bool parseSensorsRemovalOptionAndRemoveSensors(const yarp::os::Bottle &option, iDynTree::SensorsList& sensorList)
{
    // no actual parsing
    if (option.isNull()) return true;
    const size_t berdySensorNumber = 7;

    using std::string;
    using std::pair;

    pair<iDynTree::BerdySensorTypes, string> sensorsInfo[berdySensorNumber] = {
        pair<iDynTree::BerdySensorTypes, string>(iDynTree::SIX_AXIS_FORCE_TORQUE_SENSOR, "SIX_AXIS_FORCE_TORQUE_SENSOR"),
        pair<iDynTree::BerdySensorTypes, string>(iDynTree::ACCELEROMETER_SENSOR, "ACCELEROMETER_SENSOR"),
        pair<iDynTree::BerdySensorTypes, string>(iDynTree::GYROSCOPE_SENSOR, "GYROSCOPE_SENSOR"),
        pair<iDynTree::BerdySensorTypes, string>(iDynTree::DOF_ACCELERATION_SENSOR, "DOF_ACCELERATION_SENSOR"),
        pair<iDynTree::BerdySensorTypes, string>(iDynTree::DOF_TORQUE_SENSOR, "DOF_TORQUE_SENSOR"),
        pair<iDynTree::BerdySensorTypes, string>(iDynTree::NET_EXT_WRENCH_SENSOR, "NET_EXT_WRENCH_SENSOR"),
        pair<iDynTree::BerdySensorTypes, string>(iDynTree::JOINT_WRENCH_SENSOR, "JOINT_WRENCH_SENSOR"),
    };

    for (auto &sensor : sensorsInfo) {
        yarp::os::Value &sensorValue = option.find(sensor.second);
        if (sensorValue.isNull()) continue;

        if (!sensorValue.isString() && !sensorValue.isList()) {
            continue;
        }
        if (sensorValue.isString()) {
            std::string sensorName = sensorValue.asString();
            if (sensorName == "*") {
                if (!sensorList.removeAllSensorsOfType(static_cast<iDynTree::SensorType>(sensor.first))) {
                    yWarning("Error while removing all sensors of type %d", sensor.first);
                } else {
                    yInfo("Removed all sensors of type %d", sensor.first);
                }
            } else {
                if (!sensorList.removeSensor(static_cast<iDynTree::SensorType>(sensor.first), sensorName)) {
                    yWarning("Error while removing sensor %s of type %d", sensorName.c_str(), sensor.first);
                } else {
                    yInfo("Removed sensor %s of type %d", sensorName.c_str(), sensor.first);
                }
            }
        } else {
            yarp::os::Bottle *list = sensorValue.asList();
            for (int index = 0; index < list->size(); ++index) {
                if (!sensorList.removeSensor(static_cast<iDynTree::SensorType>(sensor.first), list->get(index).asString())) {
                    yWarning("Error while removing sensor %s of type %d", list->get(index).asString().c_str(), sensor.first);
                } else {
                    yInfo("Removed sensor %s of type %d", list->get(index).asString().c_str(), sensor.first);
                }
            }
        }

    }

    return true;
}

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

static bool parseMeasurementsPriorsOption(const yarp::os::Bottle& priorsGroup,
                                          const iDynTree::BerdyHelper& berdy,
                                          const std::string& optionPrefix,
                                          iDynTree::SparseMatrix& parsedMatrix)
{
    using std::pair;
    using std::string;
    //While this method may be applied for all the priors, let's stay focused on the measurements part.
    /* cases are:
     * 1) constant (for all elements)
     * 2) different depending on the type of variable (see measurements)
     * 3) in case of forces, contact/not in contact
     *
     * as ini file we can do the following:
     * 1) easy: key (optionPrefix) => value. This can be a single value or the whole vector
     * 2) use optionPrefix as prefix. Append the string representation of the variable type
     *    as before, this can be single value or whole (limited) vector
     * 3) more complex. I would use the optionPrefix as before.
     *
     * example with measurements
     * 1) cov_meas 1e-2 
     * 2) cov_meas_ACCELEROMETER (1e+1, 2e+1, 5)
     * 3) cov_meas_NET_EXT_WRENCH ((specific_element LeftFoot)(1e+1, 2e+1, 5, 0.1, 0.1, 0.3))
     * 
     * Option 1 needs to fill a simple Triplets with constant elements (of size - inputSize)
     * Option 2 needs to know the type of considered variable and the corresponding offset-size in berdy
     * Option 3 same as 2.
     */

    assert(optionPrefix == "cov_measurements");
    iDynTree::Triplets triplets;

    //Option 1
    if (priorsGroup.check(optionPrefix, "Checking priors on " + optionPrefix)) {
        yarp::os::Value& option = priorsGroup.find(optionPrefix);
        if (!parseCovarianceMatrixOption(option, parsedMatrix.rows(), triplets)) {
            yWarning("Malformed priors information for \"%s\"", optionPrefix.c_str());
        } else {
        }
    }

    //Option 2: for each measurments type, fill specific stuff.
    //This is where we lose genericity and we work only with measurements
    /* This is the "tentative grammar" for the option
     *
     * option2: key value
     * value: plain | constrained |  '(' plain? constrained* ')'
     * constrained: '((' "specific_element" string) plain ')'
     * plain: positive_double | '(' positive_double+ ')'
     */
    const size_t berdySensorNumber = 7;
    iDynTree::BerdySensorTypes sensorTypes[berdySensorNumber] = {
        iDynTree::SIX_AXIS_FORCE_TORQUE_SENSOR,
        iDynTree::ACCELEROMETER_SENSOR,
        iDynTree::GYROSCOPE_SENSOR,
        iDynTree::DOF_ACCELERATION_SENSOR,
        iDynTree::DOF_TORQUE_SENSOR,
        iDynTree::NET_EXT_WRENCH_SENSOR,
        iDynTree::JOINT_WRENCH_SENSOR
    };

    pair<size_t, string> sensorsInfo[berdySensorNumber] = {
        pair<size_t, string>(6, "SIX_AXIS_FORCE_TORQUE_SENSOR"),
        pair<size_t, string>(3, "ACCELEROMETER_SENSOR"),
        pair<size_t, string>(3, "GYROSCOPE_SENSOR"),
        pair<size_t, string>(1, "DOF_ACCELERATION_SENSOR"),
        pair<size_t, string>(1, "DOF_TORQUE_SENSOR"),
        pair<size_t, string>(6, "NET_EXT_WRENCH_SENSOR"),
        pair<size_t, string>(6, "JOINT_WRENCH_SENSOR"),
    };



    for (size_t i = 0; i < berdySensorNumber; ++i) {
        const pair<size_t, string>& sensorInfo = sensorsInfo[i];

        if (priorsGroup.check(optionPrefix + "_" + sensorInfo.second, "Checking priors on " + optionPrefix)) {
            yarp::os::Bottle& option = priorsGroup.findGroup(optionPrefix + "_" + sensorInfo.second);

            //Get the value
            std::vector<pair<string, iDynTree::Triplets> > partialElements;
            iDynTree::Triplets currentTriplets;

            //Try to parse it as if it were a plain type
            if (!parseCovarianceMatrixOption(option.get(1), sensorInfo.first, currentTriplets)) {
                // If not let's try more difficult parsing
                yarp::os::Bottle *values = option.get(1).asList();
                if (!values) {
                    //Empty value
                    continue;
                }
                for (int valueIndex = 0; valueIndex < values->size(); ++valueIndex) {
                    yarp::os::Value& currentValue = values->get(valueIndex);
                    if (valueIndex == 0) {
                        // This should parse the "plain" value which if present is always (and only) the first
                        currentTriplets.clear();
                        if (parseCovarianceMatrixOption(currentValue, sensorInfo.first, currentTriplets)) {
                            // plain value found. Go to next
                            partialElements.push_back(pair<string, iDynTree::Triplets>("", currentTriplets));
                            continue;
                        }
                    }
                    //If here we expect two elements
                    if (!currentValue.isList() || currentValue.asList()->size() != 2) {
                        continue;
                    }
                    yarp::os::Bottle *specificList = currentValue.asList();
                    //First element MUST be the name of the constrained link
                    if (!specificList->get(0).isList() || specificList->get(0).asList()->size() != 2
                        || specificList->get(0).asList()->get(0).asString() != "specific_element") {
                        continue;
                    }
                    std::string consideredFrame = specificList->get(0).asList()->get(1).asString();
                    //Second element can be a list or value. So parse it using the usual method
                    currentTriplets.clear();
                    if (!parseCovarianceMatrixOption(specificList->get(1), sensorInfo.first, currentTriplets)) {
                        continue;
                    }
                    partialElements.push_back(pair<string, iDynTree::Triplets>(consideredFrame, currentTriplets));
                }

            } else {
                partialElements.push_back(pair<string, iDynTree::Triplets>("", currentTriplets));
            }

            if (partialElements.empty()) {
                yWarning("Malformed priors information for \"%s\"", (optionPrefix + "_" + sensorInfo.second).c_str());
                continue;
            }

            //Now I have to set this triplets in the global one
            for (auto &sensor : berdy.getSensorsOrdering()) {
                for (pair<string, iDynTree::Triplets> element : partialElements) {
                    if (sensor.type != sensorTypes[i]) continue;
                    if (!element.first.empty() && sensor.id != element.first) continue;

                    for (const auto &triplet : element.second) {
                        iDynTree::Triplet modifiedTriplet = triplet;
                        modifiedTriplet.row += sensor.range.offset;
                        modifiedTriplet.column += sensor.range.offset;
                        triplets.setTriplet(modifiedTriplet);
                    }
                }
            }
        }
    }

    if (!triplets.isEmpty()) {
        parsedMatrix.setFromTriplets(triplets);
    }
    return true;
}

static bool parseCovarianceMatrixOption(const yarp::os::Value &option,
                                        size_t expectedMatrixSize,
                                        iDynTree::Triplets &parsedMatrix)
{
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
            parsedMatrix.setTriplet(iDynTree::Triplet(index, index, 1.0 / list->get(index).asDouble()));
        }

    } else {
        return false;
    }
    return true;
}
