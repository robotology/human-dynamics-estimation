/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanDynamicsEstimator.h"

#include "IHumanState.h"
#include "IHumanWrench.h"

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Estimation/BerdyHelper.h>
#include <iDynTree/Estimation/BerdySparseMAPSolver.h>
#include <iDynTree/Model/ContactWrench.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/ThreeAxisForceTorqueContactSensor.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/ThreeAxisAngularAccelerometerSensor.h>
#include <iDynTree/KinDynComputations.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/RpcServer.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <atomic>

#include <iostream>
#include <fstream>

const std::string DeviceName = "HumanDynamicsEstimator";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

struct LinkNetExternalWrenchAnalogSensorData
{
    size_t numberOfChannels = 0;
    std::vector<double> measurements;
};

void printMatrix(auto& matrix) {
    yInfo() << LogPrefix << "============= Matrix Size : " << matrix.rows() << " , " << matrix.columns() <<  " =============";
    for (size_t i = 0; i < matrix.rows(); i++) {
        for (size_t j = 0; j < matrix.columns(); j++) {
            std::cout << matrix.getValue(i,j);
        }
        std::cout << std::endl;
    }
}

static bool parseYarpValueToStdVector(const yarp::os::Value& option, std::vector<double>& output)
{
    bool isList = option.isList();
    bool isDouble = option.isFloat64();

    if (!(isList && (option.asList()->size() > 0)) && !isDouble) {
        yError() << LogPrefix << "The options must be either a double or a list of double";
        return false;
    }

    output.clear();

    if (isList) {
        size_t numOfElements = option.asList()->size();
        output.reserve(numOfElements);

        for (unsigned i = 0; i < numOfElements; ++i) {
            if (!option.asList()->get(i).isFloat64()) {
                yError() << LogPrefix << "The" << i << "th element of the list is not a double";
                return false;
            }

            output.push_back(option.asList()->get(i).asDouble());
        }
    }
    else if (isDouble) {
        output.push_back(option.asFloat64());
    }

    return true;
}

struct SensorKey
{
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
        // Using the hash as suggested in
        // http://stackoverflow.com/questions/1646807/quick-and-simple-hash-code-combinations/1646913#1646913
        size_t result = 17;
        result = result * 31 + std::hash<int>()(static_cast<int>(k.type));
        result = result * 31 + std::hash<std::string>()(k.id);
        return result;
    }
};

typedef std::unordered_map<SensorKey, iDynTree::IndexRange, SensorKeyHash> SensorMapIndex;

// This function processes the covariance option in the following way:
// - double: if a single value is passed, it resizes the vector argument to match the
//           number of values expected from the sensor type
// - list: validates that the number of elements passed in the options match the
//         number of values expected from the sensor type
static bool getVectorWithFullCovarianceValues(const std::string& optionName,
                                              std::vector<double>& values)
{
    if (values.size() == 0) {
        yError() << LogPrefix << "Failed to process vector with covariance data";
        return false;
    }

    struct BerdySensorInfo
    {
        iDynTree::BerdySensorTypes type;
        size_t size;
    };

    std::unordered_map<std::string, BerdySensorInfo> mapBerdySensorInfo = {
        {"SIX_AXIS_FORCE_TORQUE_SENSOR",
         {iDynTree::BerdySensorTypes::SIX_AXIS_FORCE_TORQUE_SENSOR, 6}},
        {"ACCELEROMETER_SENSOR", {iDynTree::BerdySensorTypes::ACCELEROMETER_SENSOR, 3}},
        {"GYROSCOPE_SENSOR", {iDynTree::BerdySensorTypes::GYROSCOPE_SENSOR, 3}},
        {"THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR",
         {iDynTree::BerdySensorTypes::THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR, 3}},
        {"THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR",
         {iDynTree::BerdySensorTypes::THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR, 3}},
        {"DOF_ACCELERATION_SENSOR", {iDynTree::BerdySensorTypes::DOF_ACCELERATION_SENSOR, 1}},
        {"DOF_TORQUE_SENSOR", {iDynTree::BerdySensorTypes::DOF_TORQUE_SENSOR, 1}},
        {"NET_EXT_WRENCH_SENSOR", {iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR, 6}},
        {"COM_ACCELEROMETER_SENSOR", {iDynTree::BerdySensorTypes::COM_ACCELEROMETER_SENSOR, 3}},
    };

    if (mapBerdySensorInfo.find(optionName) == mapBerdySensorInfo.end()) {
        yError() << LogPrefix << "Sensor name" << optionName << "not supported by Berdy";
        return false;
    }

    // Validate list size
    if (values.size() > 1) {
        if (values.size() != mapBerdySensorInfo[optionName].size) {
            yError() << LogPrefix << "The list size from the option (" << values.size()
                     << ") do not match the expected size of the" << optionName << "sensor ("
                     << mapBerdySensorInfo[optionName].size << ")";
            return false;
        }

        // If it is ok, do not edit values since it is already ok
        return true;
    }

    // Resize the vector accordingly
    values = std::vector<double>(mapBerdySensorInfo[optionName].size, values.front());
    return true;
}

struct BerdyData
{
    std::unique_ptr<iDynTree::BerdySparseMAPSolver> solver = nullptr;
    iDynTree::BerdyHelper helper;

    struct Priors
    {
        // Regularization priors
        iDynTree::VectorDynSize dynamicsRegularizationExpectedValueVector; // mu_d
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsRegularizationCovarianceMatrix; // sigma_d

        // Dynamic constraint prior
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsConstraintsCovarianceMatrix; // sigma_D

        // Measurements prior
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> measurementsCovarianceMatrix; // sigma_y

        // Task 1 Priors
        // Regularization priors
        iDynTree::VectorDynSize task1_dynamicsRegularizationExpectedValueVector; // mu_d
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> task1_dynamicsRegularizationCovarianceMatrix; // sigma_d

        // Dynamic constraint prior
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> task1_dynamicsConstraintsCovarianceMatrix; // sigma_D

        // Measurements prior
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> task1_measurementsCovarianceMatrix; // sigma_y

        static void
        initializeSparseMatrixSize(size_t size,
                                   iDynTree::SparseMatrix<iDynTree::ColumnMajor>& matrix)
        {
            iDynTree::Triplets identityTriplets;
            identityTriplets.reserve(size);

            // Set triplets to Identity
            identityTriplets.setDiagonalMatrix(0, 0, 1.0, size);

            matrix.resize(size, size);
            matrix.setFromTriplets(identityTriplets);
        }
    } priors;

    struct Buffers
    {
        iDynTree::VectorDynSize measurements;
        iDynTree::VectorDynSize task1_measurements;

    } buffers;

    struct KinematicState
    {
        iDynTree::FrameIndex floatingBaseFrameIndex;

        iDynTree::Vector3 baseAngularVelocity;
        iDynTree::JointPosDoubleArray jointsPosition;
        iDynTree::JointDOFsDoubleArray jointsVelocity;
        iDynTree::JointDOFsDoubleArray jointsAcceleration;
    } state;

    struct DynamicEstimates
    {
        iDynTree::JointDOFsDoubleArray jointTorqueEstimates;
        iDynTree::LinkNetExternalWrenches task1_linkNetExternalWrenchEstimates;
        iDynTree::LinkNetExternalWrenches linkNetExternalWrenchEstimates;
        iDynTree::LinkAccArray linkClassicalProperAccelerationEstimates; // This is also called sensor proper acceleration in Traversaro's PhD Thesis
    } estimates;    
};

// Creates an iDynTree sparse matrix (set of triplets) from a vector
static bool getSparseCovarianceMatrix(const std::vector<double>& values,
                                      iDynTree::Triplets& covarianceMatrix)
{
    if (values.size() == 0) {
        yError() << LogPrefix << "Trying to parse a covariance matrix with 0 elements";
        return false;
    }

    covarianceMatrix.clear();
    covarianceMatrix.reserve(values.size());

    for (unsigned i = 0; i < values.size(); ++i) {
        // Check that it is not zero since we need to take its inverse
        if (values[i] == 0) {
            yError() << LogPrefix
                     << "The covariance value specified in the options is 0 and it not allowed";
            return false;
        }

        // Fill the diagonal with the value stored in the option
        covarianceMatrix.setTriplet({i, i, values[i]});
    }

    return true;
}

static bool getTripletsFromPriorGroupCase1Case2(const yarp::os::Value& covMeasurementOption,
                                                const std::string& sensorType,
                                                iDynTree::Triplets& triplets)
{
    if (covMeasurementOption.isDouble()) {
        std::vector<double> covValues;
        covValues.push_back(covMeasurementOption.asFloat64());

        if (!getVectorWithFullCovarianceValues(sensorType, covValues)) {
            yError() << LogPrefix << "Failed to process" << sensorType << "sensor type";
            return false;
        }

        if (!getSparseCovarianceMatrix(covValues, triplets)) {
            yError() << LogPrefix << "Failed to process covariance matrix for" << sensorType
                     << "sensor type";
            return false;
        }

        // Triplet returned as function argument
        return true;
    }

    // Case 2
    if (covMeasurementOption.isList()) {
        if (covMeasurementOption.asList()->size() < 1) {
            yError() << "List must contain at least one element";
            return false;
        }

        std::vector<double> covValues;

        if (!parseYarpValueToStdVector(covMeasurementOption, covValues)) {
            yError() << LogPrefix << "Failed to convert yarp vector to std vector";
            return false;
        }

        if (!getVectorWithFullCovarianceValues(sensorType, covValues)) {
            yError() << LogPrefix << "Failed to process" << sensorType << "sensor type";
            return false;
        }

        if (!getSparseCovarianceMatrix(covValues, triplets)) {
            yError() << LogPrefix << "Failed to process covariance matrix for" << sensorType
                     << "sensor type";
            return false;
        }

        // Triplet returned as function argument
        return true;
    }

    // Return true because it can still be Case 3 (and its processing calls this function)
    return true;
}

static bool getTripletsFromPriorGroup(const yarp::os::Bottle priorGroup,
                                      const std::string& optionPrefix,
                                      const std::string& sensorType,
                                      iDynTree::Triplets& triplets,
                                      const iDynTree::BerdySensor& sensor)
{
    // Three cases:
    //
    // 1. Single value
    // 2. List of values
    // 3. Group with a 'value' parameter and exceptions

    // We are in case 3 if there is a group option with a value param inside
    bool isCase3 = !priorGroup.findGroup(optionPrefix + sensorType).isNull()
                   && priorGroup.findGroup(optionPrefix + sensorType).check("value");
    double value = 0;

    // -----------------
    // Case 1 and Case 2
    // -----------------

    if (!isCase3) {
        // Get the option
        yarp::os::Value& covMeasurementOption = priorGroup.find("cov_measurements_" + sensorType);

        // Case 1 and Case 2
        if (!getTripletsFromPriorGroupCase1Case2(covMeasurementOption, sensorType, triplets)) {
            yError() << LogPrefix << "Failed to parse covariance data for sensor" << sensorType;
            return false;
        }

        return true;
    }
    // ------
    // Case 3
    // ------
    else {
        // Get the value option
        yarp::os::Value& valueOption = priorGroup.findGroup(optionPrefix + sensorType).find("value");

        // Initialize the triplet with the default value
        if (!getTripletsFromPriorGroupCase1Case2(valueOption, sensorType, triplets)) {
            yError() << LogPrefix << "Failed to parse the 'value' option for sensor" << sensorType;
            return false;
        }
    }

    // Parse the exceptions for sensors that have a different covariance than the default one
    yarp::os::Bottle covMeasurementGroup = priorGroup.findGroup(optionPrefix + sensorType);

    // Check if specific_element list exists and is valid
    if (!(covMeasurementGroup.check("specific_elements")
          && covMeasurementGroup.find("specific_elements").isList())) {
        yError() << LogPrefix
                 << "Failed to find 'specific_elements' list inside cov measurement group";
        return false;
    }

    // Check if all subgroups associated with the specific_elements exist and are valid
    yarp::os::Bottle* list = covMeasurementGroup.find("specific_elements").asList();
    for (unsigned i = 0; i < list->size(); ++i) {
        // Get the specific element name
        if (!list->get(i).isString()) {
            yError() << LogPrefix << "The 'specific_elements' should be a list of strings";
            return false;
        }

        // Check if the options associated to the specific element exists
        std::string frameName = list->get(i).asString();
        if (!covMeasurementGroup.check(frameName)) {
            yError() << LogPrefix << "Failed to find specific option associated to sensor"
                     << frameName;
            return false;
        }

        if (frameName == sensor.id) {
            // Now it is as Case 1 and 2:
            yarp::os::Value covMeasurementOfSpecificElement = covMeasurementGroup.find(frameName);

            // This check allows sharing the same getTripletsFromPriorGroupCase1Case2 function
            // for parsing Case 3
            if (!covMeasurementOfSpecificElement.isDouble()
                && !(covMeasurementOfSpecificElement.isList()
                     && (covMeasurementOfSpecificElement.asList()->size() > 0))) {
                yError() << LogPrefix << "The specific elements for"
                         << "frameName should be either a double or a list of doubles";
                return false;
            }

            // Parse the specific element reusing the Case1Case2 function
            if (!getTripletsFromPriorGroupCase1Case2(
                    covMeasurementOfSpecificElement, sensorType, triplets)) {
                yError() << LogPrefix << "Failed to parse covariance data for specific element"
                         << frameName;
                return false;
            }
        }
    }

    return true;
}

static bool parsePriorsGroup(const yarp::os::Bottle& priorsGroup,
                             BerdyData& berdyData,
                             const std::unordered_map<iDynTree::BerdySensorTypes, std::string>& mapBerdySensorType,
                             std::vector<std::string>& ficticiousWrenchLinks)
{
    // =================
    // CHECK THE OPTIONS
    // =================

    if (priorsGroup.isNull()) {
        yError() << LogPrefix << "Failed to find the PRIORS options group";
        return false;
    }

    // mu_d
    bool setMuDynVariables = priorsGroup.check("mu_dyn_variables");
    if (!setMuDynVariables) {
        yWarning() << LogPrefix << "Using default values for 'mu_dyn_variables' option";
    }

    // sigma_d
    bool setCovDynVariables = priorsGroup.check("cov_dyn_variables");
    if (!setCovDynVariables) {
        yWarning() << LogPrefix << "Using default values for 'cov_dyn_variables' option";
    }

    // sigma_D
    bool setCovDynConstraints = priorsGroup.check("cov_dyn_constraints");
    if (!setCovDynConstraints) {
        yWarning() << LogPrefix << "Using default values for 'cov_dyn_constraints' option";
    }

    // =================
    // PARSE THE OPTIONS
    // =================

    std::vector<double> muDynVariables; // mu_d
    std::vector<double> covDynVariables; //sigma_d
    std::vector<double> covDynConstraints; // sigma_D

    std::vector<double> task1_muDynVariables; // mu_d
    std::vector<double> task1_covDynVariables; //sigma_d
    std::vector<double> task1_covDynConstraints; // sigma_D

    if (setMuDynVariables
        && !parseYarpValueToStdVector(priorsGroup.find("mu_dyn_variables"), muDynVariables)) {
        yError() << LogPrefix << "Failed to parse 'mu_dyn_variables' option";
        return false;
    }

    if (setCovDynVariables
        && !parseYarpValueToStdVector(priorsGroup.find("cov_dyn_variables"), covDynVariables)) {
        yError() << LogPrefix << "Failed to parse 'cov_dyn_variables' option";
        return false;
    }

    if (setCovDynConstraints
        && !parseYarpValueToStdVector(priorsGroup.find("cov_dyn_constraints"), covDynConstraints)) {
        yError() << LogPrefix << "Failed to parse 'cov_dyn_constraints' option";
        return false;
    }

    // ==========================
    // PROCESS THE PARSED OPTIONS
    // ==========================

    // ----------------------------------------------------------------
    // Priors on dynamics variables regularization expected value: mu_d
    // ----------------------------------------------------------------

    // Resize and zero the buffer
    size_t nrOfDynamicVariables = berdyData.helper.getNrOfDynamicVariables();
    size_t task1_nrOfDynamicVariables = berdyData.helper.getNrOfDynamicVariables(true);

    // Set the values stored in the configuration if any
    if (setMuDynVariables) {
        // If only one value is provided, resize it to the expected size
        if (muDynVariables.size() == 1) {
            muDynVariables = std::vector<double>(static_cast<size_t>(nrOfDynamicVariables),
                                                 muDynVariables.front());

            task1_muDynVariables = std::vector<double>(static_cast<size_t>(task1_nrOfDynamicVariables),
                                                 muDynVariables.front());
        }

        // Store the value into the berdyData
        for (size_t i = 0; i < muDynVariables.size(); ++i) {
            berdyData.priors.dynamicsRegularizationExpectedValueVector(i) = muDynVariables[i];
        }

        for (size_t i = 0; i < task1_muDynVariables.size(); ++i) {
            berdyData.priors.task1_dynamicsRegularizationExpectedValueVector(i) = task1_muDynVariables[i];
        }

        yInfo() << LogPrefix << "Dynamic variables regularization expected value vector set successfully";
    }


    // ---------------------------------------------------------------
    // Priors on dynamics variables regularization covariance: Sigma_d
    // ---------------------------------------------------------------

    // Set the values stored in the configuration if any
    if (setCovDynVariables) {
        size_t nrOfDynamicVariables = berdyData.helper.getNrOfDynamicVariables();
        size_t task1_nrOfDynamicVariables = berdyData.helper.getNrOfDynamicVariables(true);

        // If only one value is provided, resize it to the expected size
        if (covDynVariables.size() == 1) {
            covDynVariables = std::vector<double>(static_cast<size_t>(nrOfDynamicVariables),
                                                  covDynVariables.front());
            task1_covDynVariables = std::vector<double>(static_cast<size_t>(task1_nrOfDynamicVariables),
                                                  covDynVariables.front());
        }
        // Otherwise, check that the size is what is expected
        else if (covDynVariables.size() != nrOfDynamicVariables) {
            yError() << LogPrefix << "The solver expects" << nrOfDynamicVariables
                     << "elements for 'cov_dyn_variables' but only" << covDynVariables.size()
                     << "have been provided";
            return false;
        }

        iDynTree::Triplets covDynVariablesTriplets;
        if (!getSparseCovarianceMatrix(covDynVariables, covDynVariablesTriplets)) {
            yError() << LogPrefix << "Failed to process values of 'covDynVariables' option";
            return false;
        }

        // Check the size of the triplets
        if (covDynVariablesTriplets.size() != 0) {
            // Resize the sparse matrix before storing values from triplets
            berdyData.priors.dynamicsRegularizationCovarianceMatrix.resize(covDynVariablesTriplets.size(),
                                                                            covDynVariablesTriplets.size());
            // Store the value into the berdyData
            berdyData.priors.dynamicsRegularizationCovarianceMatrix.setFromTriplets(covDynVariablesTriplets);
        }
        else {
            yError() << LogPrefix << "covDynVariablesTriplets size invalid";
            return false;
        }

        // Fill task1 regularization covariance matrix
        iDynTree::Triplets task1_covDynVariablesTriplets;
        if (!getSparseCovarianceMatrix(task1_covDynVariables, task1_covDynVariablesTriplets)) {
            yError() << LogPrefix << "Failed to process values of 'covDynVariables' option for task1";
            return false;
        }

        if (task1_covDynVariablesTriplets.size() != 0) {
            // Resize the sparse matrix before storing values from triplets
            berdyData.priors.task1_dynamicsRegularizationCovarianceMatrix.resize(task1_covDynVariablesTriplets.size(),
                                                                            task1_covDynVariablesTriplets.size());
            // Store the value into the berdyData
            berdyData.priors.task1_dynamicsRegularizationCovarianceMatrix.setFromTriplets(task1_covDynVariablesTriplets);
        }
        else {
            yError() << LogPrefix << "task1 covDynVariablesTriplets size invalid";
            return false;
        }

        yInfo() << LogPrefix << "Dynamic variables regularization covariance matrix set successfully";
    }


    // --------------------------------------------------
    // Priors on dynamics constraints covariance: Sigma_D
    // --------------------------------------------------

    // Set the values stored in the configuration if any
    if (setCovDynConstraints) {
        size_t nrOfDynamicEquations = berdyData.helper.getNrOfDynamicEquations();
        size_t task1_nrOfDynamicEquations = berdyData.helper.getNrOfDynamicEquations(true);

        // If only one value is provided, resize it to the expected size
        if (covDynConstraints.size() == 1) {
            covDynConstraints = std::vector<double>(static_cast<size_t>(nrOfDynamicEquations),
                                                    covDynConstraints.front());

            task1_covDynConstraints = std::vector<double>(static_cast<size_t>(task1_nrOfDynamicEquations),
                                                    covDynConstraints.front());
        }
        // Otherwise, check that the size is what is expected
        else if (covDynConstraints.size() != nrOfDynamicEquations) {
            yError() << LogPrefix << "The solver expects" << nrOfDynamicEquations
                     << "elements for 'cov_dyn_variables' but only" << covDynConstraints.size()
                     << "have been provided";
            return false;
        }

        iDynTree::Triplets covDynConstraintsTriplets;
        if (!getSparseCovarianceMatrix(covDynConstraints, covDynConstraintsTriplets)) {
            yError() << LogPrefix << "Failed to process values of 'covDynConstraints' option";
            return false;
        }

        // Check the size of the triplets
        if (covDynConstraintsTriplets.size() != 0) {
            // Resize the sparse matrix before storing values from triplets
            berdyData.priors.dynamicsConstraintsCovarianceMatrix.resize(covDynConstraintsTriplets.size(),
                                                                         covDynConstraintsTriplets.size());
            // Store the value into the berdyData
            berdyData.priors.dynamicsConstraintsCovarianceMatrix.setFromTriplets(covDynConstraintsTriplets);
        }
        else {
            yError() << LogPrefix << "covDynConstraintsTriplets size invalid";
            return false;
        }

        // Set task1 dynamic constraints covariance matrix
        iDynTree::Triplets task1_covDynConstraintsTriplets;
        if (!getSparseCovarianceMatrix(task1_covDynConstraints, task1_covDynConstraintsTriplets)) {
            yError() << LogPrefix << "Failed to process values of 'covDynConstraints' option";
            return false;
        }

        // Check the size of the triplets
        if (task1_covDynConstraintsTriplets.size() != 0) {
            // Resize the sparse matrix before storing values from triplets
            berdyData.priors.task1_dynamicsConstraintsCovarianceMatrix.resize(task1_covDynConstraintsTriplets.size(),
                                                                         task1_covDynConstraintsTriplets.size());
            // Store the value into the berdyData
            berdyData.priors.task1_dynamicsConstraintsCovarianceMatrix.setFromTriplets(task1_covDynConstraintsTriplets);
        }
        else {
            yError() << LogPrefix << "task1 covDynConstraintsTriplets size invalid";
            return false;
        }

        yInfo() << LogPrefix << "Dynamic constraints covariance covariance matrix set successfully";
    }


    // -------------------------------------------
    // Priors on measurements constraints: Sigma_y
    // -------------------------------------------

    if (!priorsGroup.check("cov_measurements_NET_EXT_WRENCH_SENSOR")) {
        yError() << LogPrefix << "No 'cov_measurements_NET_EXT_WRENCH_SENSOR' option is found";
        return false;
    }
    else {

        yarp::os::Bottle covNetExtWrenchMeasurementGroup = priorsGroup.findGroup("cov_measurements_NET_EXT_WRENCH_SENSOR");

        // Check if specific_element list exists and is valid
        if (!(covNetExtWrenchMeasurementGroup.check("FicticiousWrenchLinks")
              && covNetExtWrenchMeasurementGroup.find("FicticiousWrenchLinks").isList())) {
            yWarning() << LogPrefix
                     << "No 'FicticiousWrenchLinks' list inside cov_measurements_NET_EXT_WRENCH_SENSOR group";
        }
        else {
            // Get dynamic wrench links list from priorsGroup
            yarp::os::Bottle* ficticiousWrenchLinksList = covNetExtWrenchMeasurementGroup.find("FicticiousWrenchLinks").asList();

            // Dynamic wrench links vector on which covariances invert from high for task1 to low for task2
            ficticiousWrenchLinks.resize(ficticiousWrenchLinksList->size());
            for (unsigned l = 0; l < ficticiousWrenchLinksList->size(); l++) {

                if (!ficticiousWrenchLinksList->get(l).isString()) {
                    yError() << LogPrefix << "The 'FicticiousWrenchLinks' should be a list of strings of link names";
                    return false;
                }

                if (!ficticiousWrenchLinksList->get(l).isString()) {
                    yError() << LogPrefix << "The 'FicticiousWrenchLinks' should be a list of strings";
                    return false;
                }

                std::string frameName = ficticiousWrenchLinksList->get(l).asString();
                ficticiousWrenchLinks.at(l) = frameName;
            }
        }
    }

    iDynTree::Triplets allSensorsTriplets;
    iDynTree::Triplets task1SensorsTriplets;
    std::string covMeasurementOptionPrefix = "cov_measurements_";

    // Construct the task1 measurement covariance matrix
    for (const iDynTree::BerdySensor& task1BerdySensor : berdyData.helper.getSensorsOrdering(true)) {

        // Chec that the sensor is a valid berdy sensor of task1
        // Check that the sensor is a valid berdy sensor
        if (mapBerdySensorType.find(task1BerdySensor.type) == mapBerdySensorType.end()) {
            yError() << LogPrefix << "Failed to find berdy sensor type. Maybe is a new sensor?";
            return false;
        }

        // Get the string from the enum
        std::string berdySensorTypeString = mapBerdySensorType.at(task1BerdySensor.type);

        if (!priorsGroup.find(covMeasurementOptionPrefix + berdySensorTypeString).isNull()) {

            iDynTree::Triplets task1_triplets{};

            if (!getTripletsFromPriorGroup(
                    priorsGroup, covMeasurementOptionPrefix, berdySensorTypeString, task1_triplets, task1BerdySensor)) {
                yError() << LogPrefix << "Failed to get task1 triplets for sensor"
                         << berdySensorTypeString;
                return false;
            }

            // Modify the triplets before adding them to the global sparse matrix.
            // This is necessary because we stack all the sensors in a single matrix.
            for (const iDynTree::Triplet& task1_triplet : task1_triplets) {
                iDynTree::Triplet task1_modifiedTriplet = task1_triplet;
                task1_modifiedTriplet.row += task1BerdySensor.range.offset;
                task1_modifiedTriplet.column += task1BerdySensor.range.offset;

                // Combine the triplet of the sensor with the global one
                task1SensorsTriplets.setTriplet(task1_modifiedTriplet);
            }

        }
        else {
            yError() << LogPrefix << "Failed to find the parameter " << covMeasurementOptionPrefix + berdySensorTypeString;
            return false;
        }

    }

    // Check the size of task1 triplets
    if (task1SensorsTriplets.size() != 0) {
        // Resize the sparse matrix before storing values from task1 triplets
        berdyData.priors.task1_measurementsCovarianceMatrix.resize(task1SensorsTriplets.size(),
                                                              task1SensorsTriplets.size());
        // Store the priors of the sensors
        berdyData.priors.task1_measurementsCovarianceMatrix.setFromTriplets(task1SensorsTriplets);
    }
    else {
        yError() << LogPrefix << "task1SensorsTriplets size invalid";
        return false;
    }

    for (const iDynTree::BerdySensor& berdySensor : berdyData.helper.getSensorsOrdering()) {

        // Check that the sensor is a valid berdy sensor
        if (mapBerdySensorType.find(berdySensor.type) == mapBerdySensorType.end()) {
            yError() << LogPrefix << "Failed to find berdy sensor type. Maybe is a new sensor?";
            return false;
        }

        // Get the string from the enum
        std::string berdySensorTypeString = mapBerdySensorType.at(berdySensor.type);

        if (!priorsGroup.find(covMeasurementOptionPrefix + berdySensorTypeString).isNull()) {

            iDynTree::Triplets triplets{};

            if (!getTripletsFromPriorGroup(
                    priorsGroup, covMeasurementOptionPrefix, berdySensorTypeString, triplets, berdySensor)) {
                yError() << LogPrefix << "Failed to get triplets for sensor"
                         << berdySensorTypeString;
                return false;
            }

            // Modify the triplets before adding them to the global sparse matrix.
            // This is necessary because we stack all the sensors in a single matrix.
            for (const iDynTree::Triplet& triplet : triplets) {
                iDynTree::Triplet modifiedTriplet = triplet;
                modifiedTriplet.row += berdySensor.range.offset;
                modifiedTriplet.column += berdySensor.range.offset;

                std::vector<std::string>::iterator found = std::find(ficticiousWrenchLinks.begin(), ficticiousWrenchLinks.end(), berdySensor.id);
                if ((found != ficticiousWrenchLinks.end()) && berdySensor.type == iDynTree::NET_EXT_WRENCH_SENSOR ) {

                    // Invert specific links wrench covariance
                    modifiedTriplet.value = 1/triplet.value;

                }

                // Combine the triplet of the sensor with the global one
                allSensorsTriplets.setTriplet(modifiedTriplet);
            }

        }
        else {
            yError() << LogPrefix << "Failed to find the parameter " << covMeasurementOptionPrefix + berdySensorTypeString;
            return false;
        }
    }

    // Check the size of the triplets
    if (allSensorsTriplets.size() != 0) {
        // Resize the sparse matrix before storing values from triplets
        berdyData.priors.measurementsCovarianceMatrix.resize(allSensorsTriplets.size(),
                                                              allSensorsTriplets.size());
        // Store the priors of the sensors
        berdyData.priors.measurementsCovarianceMatrix.setFromTriplets(allSensorsTriplets);
    }
    else {
        yError() << LogPrefix << "allSensorsTriplets size invalid";
        return false;
    }

    yInfo() << LogPrefix << "Measurements covariance inverse matrix set successfully";

    return true;
}

static bool parseSensorRemovalGroup(const yarp::os::Bottle& sensorRemovalGroup,
                                    iDynTree::SensorsList& sensorList,
                                    const std::unordered_map<iDynTree::BerdySensorTypes, std::string>& mapBerdySensorType)
{
    // =================
    // CHECK THE OPTIONS
    // =================

    if (sensorRemovalGroup.isNull()) {
        yError() << LogPrefix << "Failed to find the SENSOR_REMOVAL options group";
        return false;
    }

    for (const auto& sensor : mapBerdySensorType) {
        iDynTree::BerdySensorTypes berdySensorType = sensor.first;
        std::string sensorTypeString = sensor.second;

        // If there is no entry for this sensor type, continue
        if (!sensorRemovalGroup.check(sensorTypeString)) {
            yInfo() << LogPrefix << "Keeping all sensors of type" << sensorTypeString << "if any";
            continue;
        }

        // Check if the entry is valid and its type
        bool isList = false;
        bool isString = false;

        if (sensorRemovalGroup.find(sensorTypeString).isString()) {
            isString = true;
        }
        else if (sensorRemovalGroup.find(sensorTypeString).isList()) {
            isList = true;
        }
        else {
            yError() << LogPrefix << "The sensor removal option for sensor type" << sensorTypeString
                     << "must be either a string or a list";
            return false;
        }

        // String option
        if (isString) {
            // Get the value
            std::string sensorName = sensorRemovalGroup.find(sensorTypeString).asString();

            if (sensorName == "*") {
                // Remove all the sensors of this type
                if (!sensorList.removeAllSensorsOfType(
                        static_cast<iDynTree::SensorType>(berdySensorType))) {
                    yError() << LogPrefix << "Failed to remove all the sensors of type"
                             << sensorTypeString;
                    return false;
                }
                yInfo() << LogPrefix << "Removed all the sensors or type" << sensorTypeString;
            }
            else {
                // Remove the single sensor
                if (!sensorList.removeSensor(static_cast<iDynTree::SensorType>(berdySensorType),
                                             sensorName)) {
                    yError() << LogPrefix << "Failed to remove sensor" << sensorName << "of type"
                             << sensorTypeString;
                    return false;
                }
                yInfo() << LogPrefix << "Removed sensor" << sensorName << "of type"
                        << sensorTypeString;
            }
        }
        // List option
        else if (isList) {
            yarp::os::Bottle* list = sensorRemovalGroup.find(sensorTypeString).asList();

            if (list->size() == 0) {
                yError() << LogPrefix << "The list for removing sensor type" << sensorTypeString
                         << "does not contain any element";
                return false;
            }

            for (int index = 0; index < list->size(); ++index) {
                // Check the sensor name
                if (!list->get(index).isString()) {
                    yError() << LogPrefix << "Trying to remove the" << index << "sensor of type"
                             << sensorTypeString << "but the list element is not a string";
                    return false;
                }

                // Get the sensor name
                std::string sensorName = list->get(index).asString();

                // Remove the ith sensor
                if (!sensorList.removeSensor(static_cast<iDynTree::SensorType>(berdySensorType),
                                             sensorName)) {
                    yError() << LogPrefix << "Failed to remove sensor" << sensorName << "of type"
                             << sensorTypeString;
                    return false;
                }
                yInfo() << LogPrefix << "Removed sensor" << sensorName << "of type"
                        << sensorTypeString;
            }
        }
    }

    // Debug code to show the type and number of sensors finally contained in sensor list
    yInfo() << LogPrefix << "Number of SIX_AXIS_FORCE_TORQUE_SENSOR sensors : " << sensorList.getNrOfSensors(static_cast<iDynTree::SensorType>(iDynTree::BerdySensorTypes::SIX_AXIS_FORCE_TORQUE_SENSOR));
    yInfo() << LogPrefix << "Number of ACCELEROMETER_SENSOR sensors : " << sensorList.getNrOfSensors(static_cast<iDynTree::SensorType>(iDynTree::BerdySensorTypes::ACCELEROMETER_SENSOR));
    yInfo() << LogPrefix << "Number of GYROSCOPE_SENSOR sensors : " << sensorList.getNrOfSensors(static_cast<iDynTree::SensorType>(iDynTree::BerdySensorTypes::GYROSCOPE_SENSOR));
    yInfo() << LogPrefix << "Number of THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR sensors : " << sensorList.getNrOfSensors(static_cast<iDynTree::SensorType>(iDynTree::BerdySensorTypes::THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR));
    yInfo() << LogPrefix << "Number of THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR sensors : " << sensorList.getNrOfSensors(static_cast<iDynTree::SensorType>(iDynTree::BerdySensorTypes::THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR));

    //TODO: Double check the casting for the following berdy sensors
    //yInfo() << LogPrefix << "Number of DOF_ACCELERATION_SENSOR sensors : " << sensorList.getNrOfSensors(static_cast<iDynTree::SensorType>(iDynTree::BerdySensorTypes::DOF_ACCELERATION_SENSOR));
    //yInfo() << LogPrefix << "Number of DOF_TORQUE_SENSOR sensors : " << sensorList.getNrOfSensors(static_cast<iDynTree::SensorType>(iDynTree::BerdySensorTypes::DOF_TORQUE_SENSOR));
    //yInfo() << LogPrefix << "Number of NET_EXT_WRENCH_SENSOR sensors : " << sensorList.getNrOfSensors(static_cast<iDynTree::SensorType>(iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR));

    return true;
}

class CmdParser : public yarp::os::PortReader
{

public:
    std::atomic<bool> cmdStatus{false};
    std::atomic<bool> task1{false};
    std::atomic<bool> ficticiousLinks{false};

    bool read(yarp::os::ConnectionReader& connection) override
    {
        yarp::os::Bottle command, response;
        if (command.read(connection)) {

            if (command.get(0).asString() == "help") {
                response.addString("Enter <removeWrenchOffset> to remove the wrench estimates offset on ficticious wrench sources");
            }
            else if (command.get(0).asString() == "removeWrenchOffset" && this->ficticiousLinks) {
                response.addString("Entered command <removeWrenchOffset> is correct");
                this->cmdStatus = true;
            }
            else if (command.get(0).asString() == "removeWrenchOffset" && !this->ficticiousLinks) {
                response.addString("Entered command <removeWrenchOffset> is correct but no ficticious links present");
            }
            else {
                response.addString(
                    "Entered command is incorrect, Enter help to know available commands");
            }
        }
        else {
            this->cmdStatus = false;
            return false;
        }

        yarp::os::ConnectionWriter* reply = connection.getWriter();

        if (reply != NULL) {
            response.write(*reply);
        }
        else
            return false;

        return true;
    }
};

class HumanDynamicsEstimator::Impl
{
public:
    Impl()
    {
        gravity.zero();
        gravity(2) = -9.81;
    }

    // Rpc
    CmdParser commandPro;
    yarp::os::RpcServer rpcPort;

    // Stack of tasks berdy flags
    bool task1;

    // Debug files
    std::ofstream task1MeasurementFile;
    std::ofstream task1DynamicVariablesFile;
    std::ofstream measurementsFile;
    std::ofstream dynamicVariablesFile;

    // Attached interfaces
    hde::interfaces::IHumanState* iHumanState = nullptr;
    hde::interfaces::IHumanWrench* iHumanWrench = nullptr;
    yarp::dev::IAnalogSensor* iAnalogSensor = nullptr;

    mutable std::mutex mutex;
    iDynTree::Vector3 gravity;

    const std::unordered_map<iDynTree::BerdySensorTypes, std::string> mapBerdySensorType = {
        {iDynTree::BerdySensorTypes::SIX_AXIS_FORCE_TORQUE_SENSOR, "SIX_AXIS_FORCE_TORQUE_SENSOR"},
        {iDynTree::BerdySensorTypes::ACCELEROMETER_SENSOR, "ACCELEROMETER_SENSOR"},
        {iDynTree::BerdySensorTypes::GYROSCOPE_SENSOR, "GYROSCOPE_SENSOR"},
        {iDynTree::BerdySensorTypes::THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR,
         "THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR"},
        {iDynTree::BerdySensorTypes::THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR,
         "THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR"},
        {iDynTree::BerdySensorTypes::DOF_ACCELERATION_SENSOR, "DOF_ACCELERATION_SENSOR"},
        {iDynTree::BerdySensorTypes::DOF_TORQUE_SENSOR, "DOF_TORQUE_SENSOR"},
        {iDynTree::BerdySensorTypes::NET_EXT_WRENCH_SENSOR, "NET_EXT_WRENCH_SENSOR"},
        {iDynTree::BerdySensorTypes::JOINT_WRENCH_SENSOR, "JOINT_WRENCH_SENSOR"},
        {iDynTree::BerdySensorTypes::COM_ACCELEROMETER_SENSOR, "COM_ACCELEROMETER_SENSOR"}};

    // Berdy sensors map
    SensorMapIndex sensorMapIndex;
    SensorMapIndex task1SensorMapIndex;

    // Berdy variable
    BerdyData berdyData;

    // Model variables
    iDynTree::Model humanModel;

    // Wrench sensor link names variable
    std::vector<std::string> wrenchSensorsLinkNames;

    // Vector of links with ficticious wrench sources
    std::vector<std::string> ficticiousWrenchLinks;

    // Ficticious wrench offsets vector
    std::unordered_map<std::string, iDynTree::Wrench> ficticiousWrenchOffsetMap;

    // Link net external wrench analog sensor variable
    LinkNetExternalWrenchAnalogSensorData linkNetExternalWrenchAnalogSensorData;
};

HumanDynamicsEstimator::HumanDynamicsEstimator()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new Impl()}
{}

// Without this destructor here, the linker complains for
// undefined reference to vtable
HumanDynamicsEstimator::~HumanDynamicsEstimator() = default;

bool HumanDynamicsEstimator::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("urdf") && config.find("urdf").isString())) {
        yError() << LogPrefix << "Parameter 'urdf' missing or invalid";
        return false;
    }

    if (!(config.check("baseLink") && config.find("baseLink").isString())) {
        yError() << LogPrefix << "Parameter 'baseLink' missing or invalid";
        return false;
    }

    if (!(config.check("number_of_wrench_sensors") && config.find("number_of_wrench_sensors").isInt())) {
        yError() << LogPrefix << "Parameter 'number_of_wrench_sensors' missing or invalid";
        return false;
    }

    if (!(config.check("wrench_sensors_link_name") && config.find("wrench_sensors_link_name").isList())) {
        yError() << LogPrefix << "Parameter 'number_of_wrench_sensors' missing or invalid";
        return false;
    }

    // ===================
    // INITIALIZE RPC PORT
    // ===================

    std::string rpcPortName = "/" + DeviceName + "/rpc:i";
    if (!pImpl->rpcPort.open(rpcPortName)) {
        yError() << LogPrefix << "Unable to open rpc port " << rpcPortName;
        return false;
    }

    // Set rpc port reader
    pImpl->rpcPort.setReader(pImpl->commandPro);

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.find("period").asFloat64();
    std::string urdfFileName = config.find("urdf").asString();
    std::string baseLink = config.find("baseLink").asString();
    int number_of_wrench_sensors = config.find("number_of_wrench_sensors").asInt();
    yarp::os::Bottle* linkNames = config.find("wrench_sensors_link_name").asList();

    if (number_of_wrench_sensors != linkNames->size()) {
        yError() << LogPrefix << "mismatch between the number of wrench sensors and corresponding sensor link names list";
        return false;
    }

    pImpl->wrenchSensorsLinkNames.resize(linkNames->size());
    for (int i = 0; i < linkNames->size(); i++) {
        pImpl->wrenchSensorsLinkNames.at(i) = linkNames->get(i).asString();
    }

    // Initialize the number of channels of the equivalent IAnalogSensor
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        pImpl->linkNetExternalWrenchAnalogSensorData.measurements.resize(6 * pImpl->wrenchSensorsLinkNames.size(), 0);
        pImpl->linkNetExternalWrenchAnalogSensorData.numberOfChannels = 6 * pImpl->wrenchSensorsLinkNames.size();
    }

    yInfo() << LogPrefix << "*** ===========================";
    yInfo() << LogPrefix << "*** Period                    :" << period;
    yInfo() << LogPrefix << "*** Urdf file name            :" << urdfFileName;
    yInfo() << LogPrefix << "*** Base link name            :" << baseLink;
    yInfo() << LogPrefix << "*** Number of wrench sensors  :" << number_of_wrench_sensors;
    yInfo() << LogPrefix << "*** Wrench sensors link names :" << linkNames->toString();
    yInfo() << LogPrefix << "*** ===========================";

    // ===========
    // BERDY SETUP
    // ===========

    // Find the URDF file
    auto& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string urdfFilePath = rf.findFile(urdfFileName);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("urdf").asString();
        return false;
    }

    // Load the model
    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << urdfFilePath;
        return false;
    }

    // Get the model from the loader
    pImpl->humanModel = modelLoader.model();

    // Set fixed frame index
    pImpl->berdyData.state.floatingBaseFrameIndex = pImpl->humanModel.getFrameIndex(baseLink);

    if (pImpl->berdyData.state.floatingBaseFrameIndex == iDynTree::FRAME_INVALID_INDEX) {
        yError() << LogPrefix << "Passed frame" << baseLink << "not found in the model";
        return false;
    }

    // Initialize the sensors
    iDynTree::SensorsList humanSensors = modelLoader.sensors();

    // Add THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR sensors to the sensor list
    for (size_t a = 0; a < humanSensors.getNrOfSensors(iDynTree::ACCELEROMETER); a++) {

        // Initialize THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR
        iDynTree::ThreeAxisAngularAccelerometerSensor angAccSensor;

        // Get the accelerometer sensor pointer
        iDynTree::AccelerometerSensor *linAccSensor = static_cast<iDynTree::AccelerometerSensor*>(humanSensors.getSensor(iDynTree::ACCELEROMETER, a));

        if (!linAccSensor->isValid() || !linAccSensor->isConsistent(pImpl->humanModel)) {
            yError() << LogPrefix << "Error in reading human sensor";
            return false;
        }

        // Set the angular acceleration sensor name e.g. LeftFoot_accelerometer with accelerometer parent link name and add _angAccelerometer suffix
        std::string angAccSensorName = linAccSensor->getParentLink() + "_angAccelerometer";

        // Set angular accelerometer sensor name
        angAccSensor.setName(angAccSensorName);

        // Set angular accelerometer sensor properties same as the linear accelerometer
        angAccSensor.setParentLink(linAccSensor->getParentLink());
        angAccSensor.setParentLinkIndex(linAccSensor->getParentLinkIndex());
        angAccSensor.setLinkSensorTransform(linAccSensor->getLinkSensorTransform());

        // Add angular accelerometer to sensors list
        int sensorIndex = humanSensors.addSensor(angAccSensor);
        if ( sensorIndex == -1){
            yError() << LogPrefix << "Error in adding angular accelerometer sensor " << angAccSensor.getName()
                                  << " to the sensors list";
            return false;

        }

    }

    // If any, remove the sensors from the SENSORS_REMOVAL option
    if (!parseSensorRemovalGroup(config.findGroup("SENSORS_REMOVAL"), humanSensors, pImpl->mapBerdySensorType)) {
        yError() << LogPrefix << "Failed to parse SENSORS_REMOVAL group";
        return false;
    }

    // Initialize the options
    iDynTree::BerdyOptions berdyOptions;
    berdyOptions.baseLink = baseLink;
    berdyOptions.berdyVariant = iDynTree::BerdyVariants::BERDY_FLOATING_BASE;
    berdyOptions.includeAllNetExternalWrenchesAsSensors = true;
    berdyOptions.includeAllNetExternalWrenchesAsDynamicVariables = true;
    berdyOptions.includeAllJointAccelerationsAsSensors = true;
    berdyOptions.includeAllJointTorquesAsSensors = false;
    berdyOptions.includeFixedBaseExternalWrench = false;
    berdyOptions.includeCoMAccelerometerAsSensorInTask1 = true;
    berdyOptions.includeCoMAccelerometerAsSensorInTask2 = false;
    berdyOptions.stackOfTasksMAP = true;

    // Set rpc task1 variable
    if (berdyOptions.stackOfTasksMAP) {
        pImpl->commandPro.task1 = true;
    }

    // Berdy task1 flag
    // TODO: Probably better to have it as an option
    pImpl->task1 = true;

    // Get the comConstraintIncludeAllLinks option. The default value is true
    bool comConstraintIncludeAllLinks = config.check("comConstraintIncludeAllLinks",yarp::os::Value(true)).asBool();

    // Set the links to be considered for com acceleration constraint
    // TODO: Check if initializing the default case in berdy init is a better approach. At the moment there is no back compatibility problem
    // This is an berdy option that has to be initialized before calling berdy initialization
    // If only the links that are having know measurements are to be considered, use the wrench sensor link names
    if ((berdyOptions.includeCoMAccelerometerAsSensorInTask1 || berdyOptions.includeCoMAccelerometerAsSensorInTask2) && !comConstraintIncludeAllLinks) {
        berdyOptions.comConstraintLinkIndexVector.resize(pImpl->wrenchSensorsLinkNames.size());

        for (size_t idx = 0; idx < pImpl->wrenchSensorsLinkNames.size(); idx++)
        {
            berdyOptions.comConstraintLinkIndexVector.at(idx) = modelLoader.model().getLinkIndex(pImpl->wrenchSensorsLinkNames.at(idx));
        }
    }
    else if ((berdyOptions.includeCoMAccelerometerAsSensorInTask1 || berdyOptions.includeCoMAccelerometerAsSensorInTask2) && comConstraintIncludeAllLinks) { // Include all the model links
        berdyOptions.comConstraintLinkIndexVector.resize(modelLoader.model().getNrOfLinks());

        for (size_t idx = 0; idx < modelLoader.model().getNrOfLinks(); idx++) {
            berdyOptions.comConstraintLinkIndexVector.at(idx) = idx;
        }
    }


    // Check berdy options
    if (!berdyOptions.checkConsistency()) {
        yError() << LogPrefix << "BERDY options are not consistent";
        return false;
    }

    // Initialize the BerdyHelper
    if (!pImpl->berdyData.helper.init(modelLoader.model(), humanSensors, berdyOptions)) {
        yError() << LogPrefix << "Failed to initialize BERDY";
        return false;
    }

    if (berdyOptions.includeCoMAccelerometerAsSensorInTask2) {
        yInfo() << LogPrefix << "Number of COM_ACCELEROMETER_SENSOR sensors : " << humanSensors.getNrOfSensors(static_cast<iDynTree::SensorType>(iDynTree::BerdySensorTypes::COM_ACCELEROMETER_SENSOR));
    }

    // Initialize the BerdySolver
    pImpl->berdyData.solver = std::make_unique<iDynTree::BerdySparseMAPSolver>(pImpl->berdyData.helper);
    pImpl->berdyData.solver->initialize();

    if (!pImpl->berdyData.solver->isValid()) {
        yError() << LogPrefix << "Failed to initialize the Berdy MAP solver";
        return false;
    }

    // Get dynamic and measurement variable size
    size_t numberOfDynVariables = pImpl->berdyData.helper.getNrOfDynamicVariables();
    size_t numberOfDynEquations = pImpl->berdyData.helper.getNrOfDynamicEquations();
    size_t numberOfMeasurements = pImpl->berdyData.helper.getNrOfSensorsMeasurements();

    size_t task1_numberOfDynVariables = pImpl->berdyData.helper.getNrOfDynamicVariables(true);
    size_t task1_numberOfDynEquations = pImpl->berdyData.helper.getNrOfDynamicEquations(true);
    size_t task1_numberOfMeasurements = pImpl->berdyData.helper.getNrOfSensorsMeasurements(true);

    // Debug info
    yInfo() << LogPrefix << "Task1 number of dynamic variables : " << task1_numberOfDynVariables;
    yInfo() << LogPrefix << "Task1 number of dynamic equations : " << task1_numberOfDynEquations;
    yInfo() << LogPrefix << "Task1 number of measurements : " << task1_numberOfMeasurements;

    // Set measurements size and initialize to zero
    pImpl->berdyData.buffers.measurements.resize(numberOfMeasurements);
    pImpl->berdyData.buffers.measurements.zero();

    pImpl->berdyData.buffers.task1_measurements.resize(task1_numberOfMeasurements);
    pImpl->berdyData.buffers.task1_measurements.zero();

    // Set state variables size and initialize to zero
    pImpl->berdyData.state.jointsPosition = iDynTree::JointPosDoubleArray(pImpl->berdyData.helper.model());
    pImpl->berdyData.state.jointsPosition.zero();

    pImpl->berdyData.state.jointsVelocity = iDynTree::JointDOFsDoubleArray(pImpl->berdyData.helper.model());
    pImpl->berdyData.state.jointsVelocity.zero();

    pImpl->berdyData.state.jointsAcceleration = iDynTree::JointDOFsDoubleArray(pImpl->berdyData.helper.model());
    pImpl->berdyData.state.jointsAcceleration.zero();

    pImpl->berdyData.state.baseAngularVelocity.zero();

    // Set joint torque estimates size and initialize to zero
    pImpl->berdyData.estimates.jointTorqueEstimates = iDynTree::JointDOFsDoubleArray(pImpl->berdyData.helper.model());
    pImpl->berdyData.estimates.jointTorqueEstimates.zero();

    // Set the links net external wrench estimates size and initialize to zero
    pImpl->berdyData.estimates.task1_linkNetExternalWrenchEstimates = iDynTree::LinkWrenches(pImpl->berdyData.helper.model().getNrOfLinks());
    pImpl->berdyData.estimates.task1_linkNetExternalWrenchEstimates.zero();

    pImpl->berdyData.estimates.linkNetExternalWrenchEstimates = iDynTree::LinkWrenches(pImpl->berdyData.helper.model().getNrOfLinks());
    pImpl->berdyData.estimates.linkNetExternalWrenchEstimates.zero();

    // Set the links classical proper acceleration estimates size
    pImpl->berdyData.estimates.linkClassicalProperAccelerationEstimates = iDynTree::LinkAccArray(pImpl->berdyData.helper.model().getNrOfLinks());

    // Get the task1 berdy sensors following its internal order
    std::vector<iDynTree::BerdySensor> task1BerdySensors = pImpl->berdyData.helper.getSensorsOrdering(pImpl->task1);

    // Create a map that describes where are the sensors measurements in the task1 y vector
    // in terms of index offset and range
    for (const iDynTree::BerdySensor& task1Sensor : task1BerdySensors) {
        // Create the key
        SensorKey key = {task1Sensor.type, task1Sensor.id};

        // Check that it is unique
        if (pImpl->task1SensorMapIndex.find(key) != pImpl->task1SensorMapIndex.end()) {
            yWarning() << "The sensor" << task1Sensor.id
                       << "has been already inserted in task1SensorMapIndex. Check the urdf model for duplicates. "
                          "Skipping it.";
            continue;
        }

        // Insert the sensor index range
        // TODO: This line seems to cause corrupted size vs. prev_size Aborted (core dumped) error sometimes
        // Check if this is related to the changes in berdy for including y1 measurements
        pImpl->task1SensorMapIndex.insert({key, task1Sensor.range});
    }

    // Get the berdy sensors following its internal order
    std::vector<iDynTree::BerdySensor> berdySensors = pImpl->berdyData.helper.getSensorsOrdering();

    /* The total number of sensors are :
     * 17 Linear Accelerometers, 17 Angular Acceleromters 66 DOF Acceleration sensors and 67 NET EXT WRENCH sensors
     * The total number of sensor measurements = (17x3) + (17x3) + (66x1) + (67x6) = 519
     */

    // Create a map that describes where are the sensors measurements in the y vector
    // in terms of index offset and range
    for (const iDynTree::BerdySensor& sensor : berdySensors) {
        // Create the key
        SensorKey key = {sensor.type, sensor.id};

        // Check that it is unique
        if (pImpl->sensorMapIndex.find(key) != pImpl->sensorMapIndex.end()) {
            yWarning() << "The sensor" << sensor.id
                       << "has been already inserted. Check the urdf model for duplicates. "
                          "Skipping it.";
            continue;
        }

        // Insert the sensor index range
        pImpl->sensorMapIndex.insert({key, sensor.range});
    }

    yInfo() << LogPrefix << "The sensors are parsed successfully";


    // Set mu_d prior size and initialize to zero
    pImpl->berdyData.priors.dynamicsRegularizationExpectedValueVector.resize(numberOfDynVariables);
    pImpl->berdyData.priors.dynamicsRegularizationExpectedValueVector.zero();

    pImpl->berdyData.priors.task1_dynamicsRegularizationExpectedValueVector.resize(task1_numberOfDynVariables);
    pImpl->berdyData.priors.task1_dynamicsRegularizationExpectedValueVector.zero();

    // Set sigma_d, sigma_D and sigma_y priors size and initialize identity triplets
    pImpl->berdyData.priors.initializeSparseMatrixSize(numberOfDynVariables, pImpl->berdyData.priors.dynamicsRegularizationCovarianceMatrix);
    pImpl->berdyData.priors.initializeSparseMatrixSize(numberOfDynEquations, pImpl->berdyData.priors.dynamicsConstraintsCovarianceMatrix);
    pImpl->berdyData.priors.initializeSparseMatrixSize(numberOfMeasurements, pImpl->berdyData.priors.measurementsCovarianceMatrix);

    pImpl->berdyData.priors.initializeSparseMatrixSize(task1_numberOfDynVariables, pImpl->berdyData.priors.task1_dynamicsRegularizationCovarianceMatrix);
    pImpl->berdyData.priors.initializeSparseMatrixSize(task1_numberOfDynEquations, pImpl->berdyData.priors.task1_dynamicsConstraintsCovarianceMatrix);
    pImpl->berdyData.priors.initializeSparseMatrixSize(task1_numberOfMeasurements, pImpl->berdyData.priors.task1_measurementsCovarianceMatrix);

    // Parse the priors
    if (!parsePriorsGroup(config.findGroup("PRIORS"), pImpl->berdyData, pImpl->mapBerdySensorType, pImpl->ficticiousWrenchLinks)) {
        yError() << LogPrefix << "Failed to parse PRIORS group";
        return false;
    }
    else {
        yInfo() << LogPrefix << "PRIORS group parsed correctly";
    }

    // Set the size of ficticious wrench offset map
    if (pImpl->ficticiousWrenchLinks.size() != 0) {

        // Set ficticios links flag of the rpc command processor
        pImpl->commandPro.ficticiousLinks = true;

        // Initialize to identity wrench
        for (size_t w = 0; w < pImpl->ficticiousWrenchLinks.size(); w++) {

            iDynTree::Wrench dummyWrench = iDynTree::Wrench::Zero();
            pImpl->ficticiousWrenchOffsetMap.insert(make_pair(pImpl->ficticiousWrenchLinks.at(w), dummyWrench));

        }
    }

    // ----------------------------
    // Run a first dummy estimation
    // ----------------------------

    // Set the priors to berdy solver for task1
    pImpl->berdyData.solver->setDynamicsRegularizationPriorExpectedValue(pImpl->berdyData.priors.task1_dynamicsRegularizationExpectedValueVector, pImpl->task1);
    yInfo() << LogPrefix << "Task1 Berdy solver DynamicsRegularizationPriorExpectedValue set successfully";

    pImpl->berdyData.solver->setDynamicsRegularizationPriorCovariance(pImpl->berdyData.priors.task1_dynamicsRegularizationCovarianceMatrix, pImpl->task1);
    yInfo() << LogPrefix << "Task1 Berdy solver DynamicsRegularizationPriorCovariance set successfully";

    pImpl->berdyData.solver->setDynamicsConstraintsPriorCovariance(pImpl->berdyData.priors.task1_dynamicsConstraintsCovarianceMatrix, pImpl->task1);
    yInfo() << LogPrefix << "Task1 Berdy solver DynamicsConstraintsPriorCovariance set successfully";

    pImpl->berdyData.solver->setMeasurementsPriorCovariance(pImpl->berdyData.priors.task1_measurementsCovarianceMatrix, pImpl->task1);
    yInfo() << LogPrefix << "Task1 Berdy solver MeasurementsPriorCovariance set successfully";

    // Update estimator with task1 information
    pImpl->berdyData.solver->updateEstimateInformationFloatingBase(pImpl->berdyData.state.jointsPosition,
                                                                   pImpl->berdyData.state.jointsVelocity,
                                                                   pImpl->berdyData.state.floatingBaseFrameIndex,
                                                                   pImpl->berdyData.state.baseAngularVelocity,
                                                                   pImpl->berdyData.buffers.task1_measurements,
                                                                   pImpl->task1);

    // Do task1 berdy estimation
    if (!pImpl->berdyData.solver->doEstimate(pImpl->task1)) {
        yError() << LogPrefix << "Failed to launch a first dummy estimation for task1";
        return false;
    }

    // Extract the estimated dynamic variables of task1
    iDynTree::VectorDynSize task1_estimatedDynamicVariables(pImpl->berdyData.helper.getNrOfDynamicVariables(pImpl->task1));
    pImpl->berdyData.solver->getLastEstimate(task1_estimatedDynamicVariables, pImpl->task1);

    // Extract links net external wrench from estimated dynamic variables of task1
    pImpl->berdyData.helper.extractLinkNetExternalWrenchesFromDynamicVariables(task1_estimatedDynamicVariables,
                                                                               pImpl->berdyData.estimates.task1_linkNetExternalWrenchEstimates,
                                                                               pImpl->task1);
    // Set the priors to berdy solver
    pImpl->berdyData.solver->setDynamicsRegularizationPriorExpectedValue(pImpl->berdyData.priors.dynamicsRegularizationExpectedValueVector);
    yInfo() << LogPrefix << "Berdy solver DynamicsRegularizationPriorExpectedValue set successfully";

    pImpl->berdyData.solver->setDynamicsRegularizationPriorCovariance(pImpl->berdyData.priors.dynamicsRegularizationCovarianceMatrix);
    yInfo() << LogPrefix << "Berdy solver DynamicsRegularizationPriorCovariance set successfully";

    pImpl->berdyData.solver->setDynamicsConstraintsPriorCovariance(pImpl->berdyData.priors.dynamicsConstraintsCovarianceMatrix);
    yInfo() << LogPrefix << "Berdy solver DynamicsConstraintsPriorCovariance set successfully";

    pImpl->berdyData.solver->setMeasurementsPriorCovariance(pImpl->berdyData.priors.measurementsCovarianceMatrix);
    yInfo() << LogPrefix << "Berdy solver MeasurementsPriorCovariance set successfully";

    // Update estimator information
    pImpl->berdyData.solver->updateEstimateInformationFloatingBase(pImpl->berdyData.state.jointsPosition,
                                                                   pImpl->berdyData.state.jointsVelocity,
                                                                   pImpl->berdyData.state.floatingBaseFrameIndex,
                                                                   pImpl->berdyData.state.baseAngularVelocity,
                                                                   pImpl->berdyData.buffers.measurements);

    // Do berdy estimation
    if (!pImpl->berdyData.solver->doEstimate()) {
        yError() << LogPrefix << "Failed to launch a first dummy estimation";
        return false;
    }

    // Extract the estimated dynamic variables
    iDynTree::VectorDynSize estimatedDynamicVariables(pImpl->berdyData.helper.getNrOfDynamicVariables());
    pImpl->berdyData.solver->getLastEstimate(estimatedDynamicVariables);

    // Extract joint torques from estimated dynamic variables
    pImpl->berdyData.helper.extractJointTorquesFromDynamicVariables(estimatedDynamicVariables,
                                                                    pImpl->berdyData.state.jointsPosition,
                                                                    pImpl->berdyData.estimates.jointTorqueEstimates);

    // Extract links net external wrench from estimated dynamic variables
    pImpl->berdyData.helper.extractLinkNetExternalWrenchesFromDynamicVariables(estimatedDynamicVariables,
                                                                               pImpl->berdyData.estimates.linkNetExternalWrenchEstimates);

    // Open debug files
    pImpl->task1MeasurementFile.open("task1MeasurementsVector.txt", std::ios::trunc);
    pImpl->task1DynamicVariablesFile.open("task1DynamicVariablesVector.txt", std::ios::trunc);
    pImpl->measurementsFile.open("measurementsVector.txt", std::ios::trunc);
    pImpl->dynamicVariablesFile.open("dynamicVariablesVector.txt", std::ios::trunc);

    return true;
}

bool HumanDynamicsEstimator::close()
{
    pImpl->task1MeasurementFile.close();
    pImpl->task1DynamicVariablesFile.close();
    pImpl->measurementsFile.close();
    pImpl->dynamicVariablesFile.close();

    return true;
}

void HumanDynamicsEstimator::run()
{
    // Get state data from the attached IHumanState interface
    std::vector<double> jointsPosition    = pImpl->iHumanState->getJointPositions();
    std::vector<double> jointsVelocity    = pImpl->iHumanState->getJointVelocities();

    std::array<double, 3> basePosition    = pImpl->iHumanState->getBasePosition();
    std::array<double, 4> baseOrientation = pImpl->iHumanState->getBaseOrientation();
    std::array<double, 6> baseVelocity    = pImpl->iHumanState->getBaseVelocity();

    std::vector<std::string> accelerometerSensorNames = pImpl->iHumanState->getAccelerometerNames();
    std::vector<std::array<double, 6>> properAccelerations = pImpl->iHumanState->getProperAccelerations();

    std::array<double, 3> comProperAccelerationInBaseFrame = pImpl->iHumanState->getCoMProperAccelerationExpressedInBaseFrame();

    // Set base angular velocity
    pImpl->berdyData.state.baseAngularVelocity.setVal(0, baseVelocity.at(3));
    pImpl->berdyData.state.baseAngularVelocity.setVal(1, baseVelocity.at(4));
    pImpl->berdyData.state.baseAngularVelocity.setVal(2, baseVelocity.at(5));

    // Set the received state data to berdy state variables
    pImpl->berdyData.state.jointsPosition.resize(jointsPosition.size());
    for (size_t i = 0; i < jointsPosition.size(); i++) {
        pImpl->berdyData.state.jointsPosition.setVal(i, jointsPosition.at(i));
    }

    pImpl->berdyData.state.jointsVelocity.resize(jointsVelocity.size());
    for (size_t i = 0; i < jointsVelocity.size(); ++i) {
        pImpl->berdyData.state.jointsVelocity.setVal(i, jointsVelocity.at(i));
    }

    // Fill in the y vector with sensor measurements for the FT sensors
    std::vector<double> wrenchValues = pImpl->iHumanWrench->getWrenches();

    // Get the task1 berdy sensors following its internal order
    std::vector<iDynTree::BerdySensor> task1BerdySensors = pImpl->berdyData.helper.getSensorsOrdering(pImpl->task1);

    // Clear task1 measurements vector y1
    pImpl->berdyData.buffers.task1_measurements.zero();

    // Iterate over the task1 sensors and add corresponding measurements
    for (const iDynTree::BerdySensor& task1Sensor : task1BerdySensors) {

        // Create the key
        SensorKey key = {task1Sensor.type, task1Sensor.id};

        // Check that it exists in the sensorMapIndex
        if (pImpl->task1SensorMapIndex.find(key) != pImpl->task1SensorMapIndex.end()) {
            SensorMapIndex::const_iterator found = pImpl->task1SensorMapIndex.find(key);

            // Update task1 sensor measurements vector y1
            switch (task1Sensor.type)
            {
                case iDynTree::COM_ACCELEROMETER_SENSOR:
                {
                    // Set com proper acceleration measurements
                    pImpl->berdyData.buffers.task1_measurements(found->second.offset + 0) = comProperAccelerationInBaseFrame[0];
                    pImpl->berdyData.buffers.task1_measurements(found->second.offset + 1) = comProperAccelerationInBaseFrame[1];
                    pImpl->berdyData.buffers.task1_measurements(found->second.offset + 2) = comProperAccelerationInBaseFrame[2];
                }
                break;
                case iDynTree::NET_EXT_WRENCH_SENSOR:
                {
                    // Filling y vector with zero values for NET_EXT_WRENCH sensors
                    for (int idx = 0; idx < pImpl->wrenchSensorsLinkNames.size(); idx++) {
                        std::string wrenchSensorLinkName = pImpl->wrenchSensorsLinkNames.at(idx);
                        if (wrenchSensorLinkName.compare(task1Sensor.id) == 0) {
                            for (int i = 0; i < 6; i++)
                            {
                                pImpl->berdyData.buffers.task1_measurements(found->second.offset + i) = wrenchValues.at(idx*6 + i);
                            }
                            break;
                        }
                        else {
                            for (int i = 0; i < 6; i++)
                            {
                                pImpl->berdyData.buffers.task1_measurements(found->second.offset + i) = 0;
                            }
                        }
                    }
                }
                break;
            default:
                yWarning() << LogPrefix << task1Sensor.type << " sensor unimplemented";
                break;
            }
        }
    }

    // Dump task1 measurement vector
    pImpl->task1MeasurementFile << pImpl->berdyData.buffers.task1_measurements.toString().c_str() << std::endl;

    // Update estimator information
    pImpl->berdyData.solver->updateEstimateInformationFloatingBase(pImpl->berdyData.state.jointsPosition,
                                                                   pImpl->berdyData.state.jointsVelocity,
                                                                   pImpl->berdyData.state.floatingBaseFrameIndex,
                                                                   pImpl->berdyData.state.baseAngularVelocity,
                                                                   pImpl->berdyData.buffers.task1_measurements,
                                                                   pImpl->task1);
    // Do task1 berdy estimation
    if (!pImpl->berdyData.solver->doEstimate(pImpl->task1)) {
        yError() << LogPrefix << "Failed to do task1 berdy estimation";
    }

    // Extract the estimated dynamic variables
    iDynTree::VectorDynSize task1_estimatedDynamicVariables(pImpl->berdyData.helper.getNrOfDynamicVariables(pImpl->task1));
    pImpl->berdyData.solver->getLastEstimate(task1_estimatedDynamicVariables, pImpl->task1);

    // Dump task1 dynamic variables vector
    pImpl->task1DynamicVariablesFile << task1_estimatedDynamicVariables.toString().c_str() << std::endl;

    // Extract links net external wrench from  task1 estimated dynamic variables
    pImpl->berdyData.helper.extractLinkNetExternalWrenchesFromDynamicVariables(task1_estimatedDynamicVariables,
                                                                               pImpl->berdyData.estimates.task1_linkNetExternalWrenchEstimates,
                                                                               pImpl->task1);


    // Update wrench values with the estimates from task1
    for (int idx = 0; idx < pImpl->wrenchSensorsLinkNames.size(); idx++) {

        std::string wrenchSensorLinkName = pImpl->wrenchSensorsLinkNames.at(idx);

        iDynTree::LinkIndex linkIndex = pImpl->berdyData.helper.model().getLinkIndex(wrenchSensorLinkName);

        iDynTree::Wrench linkWrench = pImpl->berdyData.estimates.task1_linkNetExternalWrenchEstimates(linkIndex);

        iDynTree::Wrench ficticiousWrench = iDynTree::Wrench::Zero();
        if (pImpl->ficticiousWrenchOffsetMap.find(wrenchSensorLinkName) != pImpl->ficticiousWrenchOffsetMap.end()) {
            ficticiousWrench = pImpl->ficticiousWrenchOffsetMap.at(wrenchSensorLinkName);
        }

        // TODO: Double check if updating only the hands wrench measurements is better
        wrenchValues.at(idx*6 + 0) = linkWrench.getVal(0) - ficticiousWrench.getVal(0);
        wrenchValues.at(idx*6 + 1) = linkWrench.getVal(1) - ficticiousWrench.getVal(1);
        wrenchValues.at(idx*6 + 2) = linkWrench.getVal(2) - ficticiousWrench.getVal(2);
        wrenchValues.at(idx*6 + 3) = linkWrench.getVal(3) - ficticiousWrench.getVal(3);
        wrenchValues.at(idx*6 + 4) = linkWrench.getVal(4) - ficticiousWrench.getVal(4);
        wrenchValues.at(idx*6 + 5) = linkWrench.getVal(5) - ficticiousWrench.getVal(5);

        // Check for rpc command status
        if (pImpl->commandPro.cmdStatus && pImpl->commandPro.task1) {
            if (pImpl->ficticiousWrenchOffsetMap.find(wrenchSensorLinkName) != pImpl->ficticiousWrenchOffsetMap.end()) {

                // Set ficticious wrench offset
                pImpl->ficticiousWrenchOffsetMap.at(wrenchSensorLinkName) = linkWrench;
            }
        }

    }

    // Set rpc command status to false
    pImpl->commandPro.cmdStatus = false;

    // Get the berdy sensors following its internal order
    std::vector<iDynTree::BerdySensor> berdySensors = pImpl->berdyData.helper.getSensorsOrdering();

    // Clear measurements
    pImpl->berdyData.buffers.measurements.zero();

    /* The total number of sensors are :
     * 17 Accelerometers, 66 DOF Acceleration sensors and 67 NET EXT WRENCH sensors
     * The total number of sensor measurements = (17x3) + (66x1) + (67x6) = 519
     */

    // Iterate over the sensors and add corresponding measurements
    for (const iDynTree::BerdySensor& sensor : berdySensors) {

        // Create the key
        SensorKey key = {sensor.type, sensor.id};

        // Check that it exists in the sensorMapIndex
        if (pImpl->sensorMapIndex.find(key) != pImpl->sensorMapIndex.end()) {
            SensorMapIndex::const_iterator found = pImpl->sensorMapIndex.find(key);
            // Update sensor measurements vector y
            switch (sensor.type)
            {
                case iDynTree::COM_ACCELEROMETER_SENSOR:
                {
                    // Set com proper acceleration measurements
                    pImpl->berdyData.buffers.measurements(found->second.offset + 0) = comProperAccelerationInBaseFrame[0];
                    pImpl->berdyData.buffers.measurements(found->second.offset + 1) = comProperAccelerationInBaseFrame[1];
                    pImpl->berdyData.buffers.measurements(found->second.offset + 2) = comProperAccelerationInBaseFrame[2];
                }
                break;
                case iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER_SENSOR:
                {
                    // Check for the ParentLinkName_accelerometer sensor name, as this is the name of the sensors in IHumanState interface
                    std::size_t separatorLocation = sensor.id.find_last_of("_");

                    // Double check the sensor names from the model and the IHumanState interface
                    std::vector<std::string>::iterator itr = std::find(accelerometerSensorNames.begin(),
                                                                       accelerometerSensorNames.end(),
                                                                       sensor.id.substr(0, separatorLocation) + "_accelerometer");

                    // Find if the parent link name is present in the names from IHumanState interface
                    if (itr != accelerometerSensorNames.end()) {

                        // Get the proper acceleration from IHumanState interface
                        std::array<double, 6> properAcceleration = properAccelerations.at(std::distance(accelerometerSensorNames.begin(), itr));

                        // Set linear proper acceleration measurements
                        pImpl->berdyData.buffers.measurements(found->second.offset + 0) = properAcceleration[3];
                        pImpl->berdyData.buffers.measurements(found->second.offset + 1) = properAcceleration[4];
                        pImpl->berdyData.buffers.measurements(found->second.offset + 2) = properAcceleration[5];

                    }
                    else {

                        yWarning() << LogPrefix << "Could not find accelerometer" << sensor.id
                                   << "in the accelerometerSensorNames obtained from IHumanState interface. Ignoring it "
                                      "Double check if the same models are used in HumanStateProvider and HumanDynamicsEstimator";

                        // Set zero default values for ignored sensors
                        for (int i = 0; i < 3; i++)
                        {
                            pImpl->berdyData.buffers.measurements(found->second.offset + i) = 0;
                        }

                    }

                }
                break;
                case iDynTree::ACCELEROMETER_SENSOR:
                {
                    // Double check the sensor names from the model and the IHumanState interface
                    std::vector<std::string>::iterator itr = std::find(accelerometerSensorNames.begin(),
                                                                       accelerometerSensorNames.end(),
                                                                       sensor.id);

                    // Find if the parent link name is present in the names from IHumanState interface
                    if (itr != accelerometerSensorNames.end()) {

                        // Get the proper acceleration from IHumanState interface
                        std::array<double, 6> properAcceleration = properAccelerations.at(std::distance(accelerometerSensorNames.begin(), itr));

                        // Set linear proper acceleration measurements
                        pImpl->berdyData.buffers.measurements(found->second.offset + 0) = properAcceleration[0];
                        pImpl->berdyData.buffers.measurements(found->second.offset + 1) = properAcceleration[1];
                        pImpl->berdyData.buffers.measurements(found->second.offset + 2) = properAcceleration[2];

                    }
                    else {

                        yWarning() << LogPrefix << "Could not find accelerometer" << sensor.id
                                   << "in the accelerometerSensorNames obtained from IHumanState interface. Ignoring it "
                                      "Double check if the same models are used in HumanStateProvider and HumanDynamicsEstimator";

                        // Set zero default values for ignored sensors
                        for (int i = 0; i < 3; i++)
                        {
                            pImpl->berdyData.buffers.measurements(found->second.offset + i) = 0;
                        }

                    }
                }
                break;
                case iDynTree::DOF_ACCELERATION_SENSOR:
                {
                    // Filling y vector with zero values for DOF_ACCELEROMETER sensors
                    pImpl->berdyData.buffers.measurements(found->second.offset) = 0;
                }
                break;
                case iDynTree::NET_EXT_WRENCH_SENSOR:
                {
                    // Filling y vector with zero values for NET_EXT_WRENCH sensors
                    for (int idx = 0; idx < pImpl->wrenchSensorsLinkNames.size(); idx++) {
                        std::string wrenchSensorLinkName = pImpl->wrenchSensorsLinkNames.at(idx);
                        if (wrenchSensorLinkName.compare(sensor.id) == 0) {
                            for (int i = 0; i < 6; i++)
                            {
                                pImpl->berdyData.buffers.measurements(found->second.offset + i) = wrenchValues.at(idx*6 + i);
                            }
                            break;
                        }
                        else {
                            for (int i = 0; i < 6; i++)
                            {
                                pImpl->berdyData.buffers.measurements(found->second.offset + i) = 0;
                            }
                        }
                    }
                }
                break;
            default:
                yWarning() << LogPrefix << sensor.type << " sensor unimplemented";
                break;
            }
        }
    }

    // Dump measurement vector to a text file
    pImpl->measurementsFile << pImpl->berdyData.buffers.measurements.toString().c_str() << std::endl;


    // TODO: Check if this call to updateKinematicsFromFloatingBase is needed
    // updateEstimateInformationFloatingBase calls updateKinematicsFromFloatingBase in the backend of berdy helper
    // Set the kinematic information necessary for the dynamics estimation
    pImpl->berdyData.helper.updateKinematicsFromFloatingBase(pImpl->berdyData.state.jointsPosition,
                                                             pImpl->berdyData.state.jointsVelocity,
                                                             pImpl->berdyData.state.floatingBaseFrameIndex,
                                                             pImpl->berdyData.state.baseAngularVelocity);

    // Update estimator information
    pImpl->berdyData.solver->updateEstimateInformationFloatingBase(pImpl->berdyData.state.jointsPosition,
                                                                   pImpl->berdyData.state.jointsVelocity,
                                                                   pImpl->berdyData.state.floatingBaseFrameIndex,
                                                                   pImpl->berdyData.state.baseAngularVelocity,
                                                                   pImpl->berdyData.buffers.measurements);
    // Do berdy estimation
    if (!pImpl->berdyData.solver->doEstimate()) {
        yError() << LogPrefix << "Failed to do berdy estimation";
    }

    // Extract the estimated dynamic variables
    iDynTree::VectorDynSize estimatedDynamicVariables(pImpl->berdyData.helper.getNrOfDynamicVariables());
    pImpl->berdyData.solver->getLastEstimate(estimatedDynamicVariables);

    // Dump dynamic variables vector to a text file
    pImpl->dynamicVariablesFile << estimatedDynamicVariables.toString().c_str() << std::endl;

    iDynTree::LinkProperAccArray properAccs;
    iDynTree::LinkNetTotalWrenchesWithoutGravity netTotalWrenchesWithoutGrav;
    iDynTree::LinkNetExternalWrenches netExtWrenches;
    iDynTree::LinkInternalWrenches linkJointWrenches;
    iDynTree::JointDOFsDoubleArray jointTorques;
    iDynTree::JointDOFsDoubleArray jointAccs;

    //Extract LINK_BODY_PROPER_CLASSICAL_ACCELERATION from dynamic variables
    for (iDynTree::LinkIndex lnkIdx=0; lnkIdx < static_cast<iDynTree::LinkIndex>(pImpl->humanModel.getNrOfLinks()); lnkIdx++) {

        iDynTree::IndexRange range = pImpl->berdyData.helper.getRangeLinkVariable(iDynTree::LINK_BODY_PROPER_CLASSICAL_ACCELERATION, lnkIdx);
        iDynTree::LinAcceleration linAcc(estimatedDynamicVariables.data() + range.offset, 3);
        iDynTree::AngAcceleration angAcc(estimatedDynamicVariables.data() + range.offset + 3, 3);
        pImpl->berdyData.estimates.linkClassicalProperAccelerationEstimates(lnkIdx) = iDynTree::SpatialAcc(linAcc, angAcc);


    }

    //yInfo() << LogPrefix << "================ Berdy classical acceleration estimates size is " << pImpl->berdyData.estimates.linkClassicalProperAccelerationEstimates.getNrOfLinks() << " ==============";
    //yInfo() << pImpl->berdyData.estimates.linkClassicalProperAccelerationEstimates.toString(pImpl->humanModel).c_str();


    // ===========================
    // EXPOSE DATA FOR IHUMANSTATE
    // ===========================

    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);

        // Extract joint torques from estimated dynamic variables
        pImpl->berdyData.helper.extractJointTorquesFromDynamicVariables(estimatedDynamicVariables,
                                                                        pImpl->berdyData.state.jointsPosition,
                                                                        pImpl->berdyData.estimates.jointTorqueEstimates);

        // Extract links net external wrench from estimated dynamic variables
        pImpl->berdyData.helper.extractLinkNetExternalWrenchesFromDynamicVariables(estimatedDynamicVariables,
                                                                                   pImpl->berdyData.estimates.linkNetExternalWrenchEstimates);

//        pImpl->berdyData.estimates.linkNetExternalWrenchEstimates = pImpl->berdyData.estimates.task1_linkNetExternalWrenchEstimates;

        // Using kindyn to compute link
        iDynTree::KinDynComputations kinDynComputations;
        kinDynComputations.loadRobotModel(pImpl->humanModel);
        kinDynComputations.setFloatingBase(pImpl->humanModel.getLinkName(pImpl->berdyData.state.floatingBaseFrameIndex));
        iDynTree::Vector4 quat;
        quat.setVal(0, baseOrientation[0]);
        quat.setVal(1, baseOrientation[1]);
        quat.setVal(2, baseOrientation[2]);
        quat.setVal(3, baseOrientation[3]);

        iDynTree::Rotation rot;
        rot.fromQuaternion(quat);

        iDynTree::Position pos;
        pos.setVal(0, basePosition[0]);
        pos.setVal(1, basePosition[1]);
        pos.setVal(2, basePosition[2]);

        iDynTree::Transform baseTransform;
        baseTransform.setPosition(pos);
        baseTransform.setRotation(rot);

        iDynTree::VectorDynSize s, sdot;
        s.resize(jointsPosition.size());
        sdot.resize(jointsPosition.size());
        for (size_t idx = 0; idx < jointsPosition.size(); idx++)
        {
            s.setVal(idx, jointsPosition[idx]);
            sdot.setVal(idx, jointsVelocity[idx]);
        }

        // TODO clean this workaround
        iDynTree::Vector3 gravity;
        gravity.zero();
        gravity(0) = pImpl->gravity(0);
        gravity(1) = pImpl->gravity(1);
        gravity(2) = pImpl->gravity(2);
        kinDynComputations.setRobotState(baseTransform,
                                         s,
                                         iDynTree::Twist::Zero(),
                                         sdot,
                                         gravity);


        // Check to ensure all the links net external wrenches are extracted correctly
        if (!pImpl->berdyData.estimates.linkNetExternalWrenchEstimates.isConsistent(pImpl->humanModel))
        {
            yError() << LogPrefix << "Links net external wrench estimates extracted from dynamic variables are not consistent with the model provided";
        }

        // Expose the data as IAnalogSensor
        // ================================
        // Exposing only wrench on only selected links
        // TODO: Instead of using wrenchSensorsLinkNames may be we can give a separate parameter
        // containing links we are interested to see the extracted wrench estimates
        for (int i = 0; i < pImpl->wrenchSensorsLinkNames.size(); i++) {

            std::string linkName = pImpl->wrenchSensorsLinkNames.at(i);

            // Get world_H_link transform
            iDynTree::Transform world_H_link = kinDynComputations.getWorldTransform(linkName);   
            world_H_link.setPosition(iDynTree::Position::Zero());
            //yInfo() << "----> link name " << linkName << " " << pImpl->berdyData.estimates.linkNetExternalWrenchEstimates(pImpl->humanModel.getLinkIndex(linkName)).toString();

//            iDynTree::Wrench linkNetExternalWrench = (world_H_link.inverse()) * pImpl->berdyData.estimates.linkNetExternalWrenchEstimates(pImpl->humanModel.getLinkIndex(linkName));
            iDynTree::Wrench linkNetExternalWrench = (iDynTree::Transform::Identity()) * pImpl->berdyData.estimates.linkNetExternalWrenchEstimates(pImpl->humanModel.getLinkIndex(linkName));

            //yInfo() << "----> link name " << linkName << " " << linkNetExternalWrench.toString();
            pImpl->linkNetExternalWrenchAnalogSensorData.measurements[6 * i + 0] = linkNetExternalWrench.getLinearVec3()(0);
            pImpl->linkNetExternalWrenchAnalogSensorData.measurements[6 * i + 1] = linkNetExternalWrench.getLinearVec3()(1);
            pImpl->linkNetExternalWrenchAnalogSensorData.measurements[6 * i + 2] = linkNetExternalWrench.getLinearVec3()(2);
            pImpl->linkNetExternalWrenchAnalogSensorData.measurements[6 * i + 3] = linkNetExternalWrench.getAngularVec3()(0);
            pImpl->linkNetExternalWrenchAnalogSensorData.measurements[6 * i + 4] = linkNetExternalWrench.getAngularVec3()(1);
            pImpl->linkNetExternalWrenchAnalogSensorData.measurements[6 * i + 5] = linkNetExternalWrench.getAngularVec3()(2);

        }

    }

}

bool HumanDynamicsEstimator::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    // Get the device name from the driver
    const std::string deviceName = poly->getValue("device").asString();

    if (deviceName == "human_state_provider" || deviceName == "xsens_human_state_provider") {

        // Attach IHumanState interface from HumanStateProvider
        if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
            yError() << LogPrefix << "Failed to view IHumanState interface from the polydriver";
            return false;
        }

        // Check the interface
        if (pImpl->iHumanState->getNumberOfJoints() == 0
                || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
            yError() << "The IHumanState interface might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
    }

    if (deviceName == "human_wrench_provider") {
        // Attach IAnalogServer interfaces coming from HumanWrenchProvider
        if (pImpl->iAnalogSensor || !poly->view(pImpl->iAnalogSensor) || !pImpl->iAnalogSensor) {
            yError() << LogPrefix << "Failed to view IAnalogSensor interface from the polydriver";
            return false;
        }

        // Check the interface
        auto numberOfSensors = stoi(poly->getValue("number_of_sources").asString());
        if (pImpl->iAnalogSensor->getChannels() != 6 * numberOfSensors) {
            yError() << LogPrefix << "The IAnalogSensor interface might not be ready";
            return false;
        }

        // Attach IHumanWrench interfaces coming from HumanWrenchProvider
        if (pImpl->iHumanWrench || !poly->view(pImpl->iHumanWrench) || !pImpl->iHumanWrench) {
            yError() << LogPrefix << "Failed to view iHumanWrench interface from the polydriver";
            return false;
        }

        // Check the interface
        if (pImpl->iHumanWrench->getNumberOfWrenchSources() == 0
                || pImpl->iHumanWrench->getNumberOfWrenchSources() != pImpl->iHumanWrench->getWrenchSourceNames().size()) {
            yError() << "The IHumanWrench interface might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
    }

    return true;
}

void HumanDynamicsEstimator::threadRelease()
{}

bool HumanDynamicsEstimator::detach()
{
    while(isRunning()) {
        stop();
    }

    pImpl->iHumanState = nullptr;
    pImpl->iHumanWrench = nullptr;
    pImpl->iAnalogSensor = nullptr;
    stop();
    return true;
}

bool HumanDynamicsEstimator::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    bool attachStatus = true;
    if (driverList.size() > 2) {
        yError() << LogPrefix << "This wrapper accepts only two attached PolyDriver";
        return false;
    }

    for (size_t i = 0; i < driverList.size(); i++) {
        const yarp::dev::PolyDriverDescriptor* driver = driverList[i];

        if (!driver) {
            yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
            return false;
        }

        attachStatus = attachStatus && attach(driver->poly);
    }

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (attachStatus && !start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    return attachStatus;
}

bool HumanDynamicsEstimator::detachAll()
{
    return detach();
}

std::vector<std::string> HumanDynamicsEstimator::getJointNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    std::vector<std::string> jointNames;

    for (size_t jointIndex = 0; jointIndex < pImpl->humanModel.getNrOfJoints(); ++jointIndex) {
        jointNames.emplace_back(pImpl->humanModel.getJointName(jointIndex));
    }

    return jointNames;
}

size_t HumanDynamicsEstimator::getNumberOfJoints() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->humanModel.getNrOfJoints();
}

std::vector<double> HumanDynamicsEstimator::getJointTorques() const
{
    std::vector<double> jointTorques;
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    size_t vecSize = pImpl->berdyData.estimates.jointTorqueEstimates.size();
    jointTorques.resize(vecSize);
    for (size_t index = 0; index < vecSize; index++) {
        jointTorques.at(index) = pImpl->berdyData.estimates.jointTorqueEstimates.getVal(index);
    }
    return jointTorques;
}

// =============
// IAnalogSensor
// =============

int HumanDynamicsEstimator::read(yarp::sig::Vector& out)
{
    out.resize(pImpl->linkNetExternalWrenchAnalogSensorData.measurements.size());

    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        std::copy(pImpl->linkNetExternalWrenchAnalogSensorData.measurements.begin(),
                  pImpl->linkNetExternalWrenchAnalogSensorData.measurements.end(),
                  out.data());
    }

    return IAnalogSensor::AS_OK;
}

int HumanDynamicsEstimator::getState(int ch)
{
    // The return status is always ok as the data is not from any sensor but estimation values
    return IAnalogSensor::AS_OK;
}

int HumanDynamicsEstimator::getChannels()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->linkNetExternalWrenchAnalogSensorData.numberOfChannels;
}

int HumanDynamicsEstimator::calibrateSensor()
{
    return IAnalogSensor::AS_ERROR;
}

int HumanDynamicsEstimator::calibrateSensor(const yarp::sig::Vector& /*value*/)
{
    return IAnalogSensor::AS_ERROR;
}

int HumanDynamicsEstimator::calibrateChannel(int /*ch*/)
{
    return IAnalogSensor::AS_ERROR;
}

int HumanDynamicsEstimator::calibrateChannel(int /*ch*/, double /*value*/)
{
    return IAnalogSensor::AS_ERROR;
}
