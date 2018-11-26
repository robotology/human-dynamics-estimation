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

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IAnalogSensor.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

const std::string DeviceName = "HumanDynamicsEstimator";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

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

// TODO: @Yeshi some of these structures were needed before I started using the BerdySparseMAPSolver
struct BerdyData
{
    std::unique_ptr<iDynTree::BerdySparseMAPSolver> solver = nullptr;
    iDynTree::BerdyHelper helper;

    struct Priors
    {
        // Dynamics
        iDynTree::VectorDynSize dynamicsRegularizationExpectedValue;
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsConstraintsCovarianceInverse;
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsRegularizationCovarianceInverse;

        // Measurements
        iDynTree::SparseMatrix<iDynTree::ColumnMajor> measurementsCovarianceInverse;

        static void
        initializeSparseMatrixSize(size_t size,
                                   iDynTree::SparseMatrix<iDynTree::ColumnMajor>& matrix)
        {
            iDynTree::Triplets triplets;
            triplets.reserve(size);
            triplets.setDiagonalMatrix(0, 0, 1.0, size);

            matrix.resize(size, size);
            matrix.setFromTriplets(triplets);
        }
    } priors;

    struct Posteriors
    {
        iDynTree::VectorDynSize expectedDynamics;
        Eigen::SparseMatrix<double, Eigen::ColMajor> dynamicsCovarianceInverse;
    } posteriors;

    //    struct LinearSystemData
    //    {
    //        // Dynamics
    //        iDynTree::VectorDynSize dynamicsConstraintsBias;
    //        iDynTree::SparseMatrix<iDynTree::ColumnMajor> dynamicsConstraintsMatrix;
    //        // Measurements
    //        iDynTree::VectorDynSize measurementsBias;
    //        iDynTree::SparseMatrix<iDynTree::ColumnMajor> measurementsMatrix;
    //    } data;

    struct Buffers
    {
        // TODO naming?
        Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>>
            covarianceDynamicsPriorInverseDecomposition;
        Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>>
            covarianceDynamicsAPosterioriInverseDecomposition;

        Eigen::SparseMatrix<double, Eigen::ColMajor> covarianceDynamicsPriorInverse;

        iDynTree::VectorDynSize expectedDynamicsPrior;

        iDynTree::VectorDynSize expectedDynamicsPriorRHS;
        iDynTree::VectorDynSize expectedDynamicsAPosterioriRHS;

        iDynTree::VectorDynSize measurements;

    } buffers;

    struct KinematicState
    {
        iDynTree::FrameIndex floatingFrameIndex;

        iDynTree::Vector3 baseAngularVelocity;
        iDynTree::JointPosDoubleArray jointsPosition;
        iDynTree::JointDOFsDoubleArray jointsVelocity;
        iDynTree::JointDOFsDoubleArray jointsAcceleration;
    } state;

    struct DynamicEstimates
    {
        iDynTree::JointDOFsDoubleArray jointTorqueEstimates;
    } estimates;

    //    void computeMaximumAPosteriori(bool computePermutation);

    // TODO where put these
    // ====================

    SensorMapIndex sensorMapIndex;
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

        // Fill the diagonal with the inverse of the value stored in the option
        covarianceMatrix.setTriplet({i, i, 1.0 / values[i]});
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
                                      iDynTree::Triplets& triplets)
{
    // Three cases:
    //
    // 1. Single value
    // 2. List of values
    // 3. Group with a 'value' parameter and exceptions

    // We are in case 3 if there is a group option with a value param inside
    bool isCase3 = !priorGroup.findGroup(optionPrefix + sensorType).isNull()
                   && priorGroup.findGroup(optionPrefix + sensorType).check("value");

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

    // Check if value exists and is valid
    yarp::os::Bottle covMeasurementGroup = priorGroup.findGroup(optionPrefix + sensorType);
    if (!(covMeasurementGroup.check("value") && covMeasurementGroup.find("value").asFloat64())) {
        yError() << LogPrefix << "Failed to find 'value' option inside cov measurement group";
        return false;
    }

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

    return true;
}

static bool parsePriorsGroup(const yarp::os::Bottle& priorsGroup,
                             BerdyData& berdyData,
                             const std::unordered_map<iDynTree::BerdySensorTypes, std::string>& mapBerdySensorType)
{
    // =================
    // CHECK THE OPTIONS
    // =================

    if (priorsGroup.isNull()) {
        yError() << LogPrefix << "Failed to find the PRIORS options group";
        return false;
    }

    bool setCovDynVariables = priorsGroup.check("cov_dyn_variables");
    if (!setCovDynVariables) {
        yWarning() << LogPrefix << "Using default values for 'cov_dyn_variables' option";
    }

    bool setCovDynConstraints = priorsGroup.check("cov_dyn_constraints");
    if (!setCovDynConstraints) {
        yWarning() << LogPrefix << "Using default values for 'cov_dyn_constraints' option";
    }

    bool setMuDynVariables = priorsGroup.check("mu_dyn_variables");
    if (!setMuDynVariables) {
        yWarning() << LogPrefix << "Using default values for 'mu_dyn_variables' option";
    }

    // =================
    // PARSE THE OPTIONS
    // =================

    std::vector<double> covDynVariables;
    std::vector<double> covDynConstraints;
    std::vector<double> muDynVariables;

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

    if (setMuDynVariables
        && !parseYarpValueToStdVector(priorsGroup.find("mu_dyn_variables"), muDynVariables)) {
        yError() << LogPrefix << "Failed to parse 'mu_dyn_variables' option";
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
    berdyData.priors.dynamicsRegularizationExpectedValue.resize(nrOfDynamicVariables);
    berdyData.priors.dynamicsRegularizationExpectedValue.zero();

    // Set the values stored in the configuration if any
    if (setMuDynVariables) {
        // If only one value is provided, resize it to the expected size
        if (muDynVariables.size() == 1) {
            muDynVariables = std::vector<double>(static_cast<size_t>(nrOfDynamicVariables),
                                                 muDynVariables.front());
        }

        // Store the value into the berdyData
        for (size_t i = 0; i < muDynVariables.size(); ++i) {
            berdyData.priors.dynamicsRegularizationExpectedValue(i) = muDynVariables[i];
        }

        yInfo() << LogPrefix << "Dynamic variables regularization expected value vector set successfully";
    }

    // --------------------------------------------------
    // Priors on dynamics constraints covariance: Sigma_D
    // --------------------------------------------------

    // Set the values stored in the configuration if any
    if (setCovDynConstraints) {
        size_t nrOfDynamicEquations = berdyData.helper.getNrOfDynamicEquations();

        // If only one value is provided, resize it to the expected size
        if (covDynConstraints.size() == 1) {
            covDynConstraints = std::vector<double>(static_cast<size_t>(nrOfDynamicEquations),
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
            berdyData.priors.dynamicsConstraintsCovarianceInverse.resize(covDynConstraintsTriplets.size(),
                                                                         covDynConstraintsTriplets.size());
            // Store the value into the berdyData
            berdyData.priors.dynamicsConstraintsCovarianceInverse.setFromTriplets(covDynConstraintsTriplets);

        }
        else {
            yError() << LogPrefix << "covDynConstraintsTriplets size invalid";
            return false;
        }

        yInfo() << LogPrefix << "Dynamic constraints covariance covariance matrix set successfully";
    }

    // ---------------------------------------------------------------
    // Priors on dynamics variables regularization covariance: Sigma_d
    // ---------------------------------------------------------------

    // Set the values stored in the configuration if any
    if (setCovDynVariables) {
        size_t nrOfDynamicVariables = berdyData.helper.getNrOfDynamicVariables();

        // If only one value is provided, resize it to the expected size
        if (covDynVariables.size() == 1) {
            covDynVariables = std::vector<double>(static_cast<size_t>(nrOfDynamicVariables),
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
            berdyData.priors.dynamicsRegularizationCovarianceInverse.resize(covDynVariablesTriplets.size(),
                                                                            covDynVariablesTriplets.size());
            // Store the value into the berdyData
            berdyData.priors.dynamicsRegularizationCovarianceInverse.setFromTriplets(covDynVariablesTriplets);

        }
        else {
            yError() << LogPrefix << "covDynVariablesTriplets size invalid";
            return false;
        }

        yInfo() << LogPrefix << "Dynamic variables regularization covariance matrix set successfully";
    }

    // ----------------------------------
    // Priors on measurements constraints
    // ----------------------------------

    iDynTree::Triplets allSensorsTriplets;
    std::string covMeasurementOptionPrefix = "cov_measurements_";

    for (const iDynTree::BerdySensor& berdySensor : berdyData.helper.getSensorsOrdering()) {

        // Check that the sensor is a valid berdy sensor
        if (mapBerdySensorType.find(berdySensor.type) == mapBerdySensorType.end()) {
            yError() << LogPrefix << "Failed to find berdy sensor type. Maybe is a new sensor?";
            return false;
        }

        // Get the string from the enum
        std::string berdySensorTypeString = mapBerdySensorType.at(berdySensor.type);

        if (!priorsGroup.find(covMeasurementOptionPrefix + berdySensorTypeString).isNull()) {

            iDynTree::Triplets triplets;

            if (!getTripletsFromPriorGroup(
                    priorsGroup, covMeasurementOptionPrefix, berdySensorTypeString, triplets)) {
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
        berdyData.priors.measurementsCovarianceInverse.resize(allSensorsTriplets.size(),
                                                              allSensorsTriplets.size());
        // Store the priors of the sensors
        berdyData.priors.measurementsCovarianceInverse.setFromTriplets(allSensorsTriplets);
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

class HumanDynamicsEstimator::Impl
{
public:
    Impl()
    {
        gravity.zero();
        gravity(2) = -9.81;
    }

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
        {iDynTree::BerdySensorTypes::JOINT_WRENCH_SENSOR, "JOINT_WRENCH_SENSOR"}};

    // Berdy variable
    BerdyData berdyData;

    // Model variables
    iDynTree::Model humanModel;

    // Wrench sensor link names variable
    std::vector<std::string> wrenchSensorsLinkNames;
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

    // Initialize the sensors
    iDynTree::SensorsList humanSensors = modelLoader.sensors();

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

    if (!berdyOptions.checkConsistency()) {
        yError() << LogPrefix << "BERDY options are not consistent";
        return false;
    }

    // Initialize the BerdyHelper
    if (!pImpl->berdyData.helper.init(modelLoader.model(), humanSensors, berdyOptions)) {
        yError() << LogPrefix << "Failed to initialize BERDY";
        return false;
    }

    // Initialize the BerdySolver
    pImpl->berdyData.solver = std::make_unique<iDynTree::BerdySparseMAPSolver>(pImpl->berdyData.helper);
    pImpl->berdyData.solver->initialize();

    if (!pImpl->berdyData.solver->isValid()) {
        yError() << LogPrefix << "Failed to initialize the Berdy MAP solver";
        return false;
    }

    yInfo() << LogPrefix << "Number of measurements : " << pImpl->berdyData.helper.getNrOfSensorsMeasurements();
    yInfo() << LogPrefix << "Number of links : " << pImpl->humanModel.getNrOfLinks();

    // Initialize buffers of the state
    pImpl->berdyData.state.jointsPosition = iDynTree::JointPosDoubleArray(pImpl->berdyData.helper.model());
    pImpl->berdyData.state.jointsVelocity = iDynTree::JointDOFsDoubleArray(pImpl->berdyData.helper.model());
    pImpl->berdyData.state.jointsAcceleration = iDynTree::JointDOFsDoubleArray(pImpl->berdyData.helper.model());

    pImpl->berdyData.estimates.jointTorqueEstimates.resize(modelLoader.model());

    // Get the berdy sensors following its internal order
    std::vector<iDynTree::BerdySensor> berdySensors = pImpl->berdyData.helper.getSensorsOrdering();

    /* The total number of sensors are :
     * 17 Accelerometers, 66 DOF Acceleration sensors and 67 NET EXT WRENCH sensors
     * The total number of sensor measurements = (17x3) + (66x1) + (67x6) = 519
     */

    // Create a map that describes where are the sensors measurements in the y vector
    // in terms of index offset and range
    for (const iDynTree::BerdySensor& sensor : berdySensors) {
        // Create the key
        SensorKey key = {sensor.type, sensor.id};

        // Check that it is unique
        if (pImpl->berdyData.sensorMapIndex.find(key) != pImpl->berdyData.sensorMapIndex.end()) {
            yWarning() << "The sensor" << sensor.id
                       << "has been already inserted. Check the urdf model for duplicates. "
                          "Skipping it.";
            continue;
        }

        // Insert the sensor index range
        pImpl->berdyData.sensorMapIndex.insert({key, sensor.range});
    }

    yInfo() << LogPrefix << "The sensors are parsed successfully";

    // Load the priors
    if (!parsePriorsGroup(config.findGroup("PRIORS"), pImpl->berdyData, pImpl->mapBerdySensorType)) {
        yError() << LogPrefix << "Failed to parse PRIORS group";
        return false;
    }
    else {
        yInfo() << LogPrefix << "PRIORS group parsed correctly";
    }

    // Set the priors into the berdy solver
    // The sizes of the priors are set in the parsePriorsGroup function
    pImpl->berdyData.solver->setDynamicsRegularizationPriorExpectedValue(pImpl->berdyData.priors.dynamicsRegularizationExpectedValue);
    yInfo() << LogPrefix << "Berdy solver DynamicsRegularizationPriorExpectedValue set successfully";

    pImpl->berdyData.solver->setDynamicsConstraintsPriorCovariance(pImpl->berdyData.priors.dynamicsConstraintsCovarianceInverse);
    yInfo() << LogPrefix << "Berdy solver DynamicsConstraintsPriorCovariance set successfully";

    pImpl->berdyData.solver->setDynamicsRegularizationPriorCovariance(pImpl->berdyData.priors.dynamicsRegularizationCovarianceInverse);
    yInfo() << LogPrefix << "Berdy solver DynamicsRegularizationPriorCovariance set successfully";

    pImpl->berdyData.solver->setMeasurementsPriorCovariance(pImpl->berdyData.priors.measurementsCovarianceInverse);
    yInfo() << LogPrefix << "Berdy solver MeasurementsPriorCovariance set successfully";

    pImpl->berdyData.buffers.measurements.resize(pImpl->berdyData.helper.getNrOfSensorsMeasurements());

    // ----------------------------
    // Run a first dummy estimation
    // ----------------------------

    pImpl->berdyData.state.floatingFrameIndex = pImpl->berdyData.helper.model().getFrameIndex(baseLink);

    if (pImpl->berdyData.state.floatingFrameIndex == iDynTree::FRAME_INVALID_INDEX) {
        yError() << LogPrefix << "Passed frame" << baseLink << "not found in the model";
        return false;
    }

    // Fill random data
    iDynTree::getRandomVector(pImpl->berdyData.state.jointsPosition);
    iDynTree::getRandomVector(pImpl->berdyData.state.jointsVelocity);
    iDynTree::getRandomVector(pImpl->berdyData.state.baseAngularVelocity);
    iDynTree::getRandomVector(pImpl->berdyData.buffers.measurements);

    // Solve a dummy problem
    pImpl->berdyData.solver->updateEstimateInformationFloatingBase(pImpl->berdyData.state.jointsPosition,
                                                                   pImpl->berdyData.state.jointsVelocity,
                                                                   pImpl->berdyData.state.floatingFrameIndex,
                                                                   pImpl->berdyData.state.baseAngularVelocity,
                                                                   pImpl->berdyData.buffers.measurements);

    if (!pImpl->berdyData.solver->doEstimate()) {
        yError() << LogPrefix << "Failed to launch a first dummy estimation";
        return false;
    }

    // Extract the solution
    iDynTree::VectorDynSize estimatedDynamicVariables(pImpl->berdyData.helper.getNrOfDynamicVariables());
    pImpl->berdyData.solver->getLastEstimate(estimatedDynamicVariables);

    return true;
}

bool HumanDynamicsEstimator::close()
{
    stop();
    detachAll();
    return true;
}

void HumanDynamicsEstimator::run()
{
    // Kinematic state is read from the attached IHumanState interface
    // Get kinematic state data
    std::vector<double> jointsPosition    = pImpl->iHumanState->getJointPositions();
    std::vector<double> jointsVelocity    = pImpl->iHumanState->getJointVelocities();

    std::array<double, 3> basePosition    = pImpl->iHumanState->getBasePosition();
    std::array<double, 4> baseOrientation = pImpl->iHumanState->getBaseOrientation();
    std::array<double, 6> baseVelocity    = pImpl->iHumanState->getBaseVelocity();

    // Debug code
    //yInfo() << LogPrefix << "Joint positions : " << jointsPosition;
    //yInfo() << LogPrefix << "Joint velocity : " << jointsVelocity;

    //yInfo() << LogPrefix << "Base position : " << basePosition.at(0) << basePosition.at(1) << basePosition.at(2);
    //yInfo() << LogPrefix << "Base orientation : " << baseOrientation.at(0) << baseOrientation.at(1) << baseOrientation.at(2);
    //yInfo() << LogPrefix << "Base velocity : " << baseVelocity.at(0)
     //                                          << baseVelocity.at(1)
      //                                         << baseVelocity.at(2)
      //                                         << baseVelocity.at(3)
       //                                        << baseVelocity.at(4)
         //                                      << baseVelocity.at(5);

    // Pad the received state data to berdy state variables
    pImpl->berdyData.state.jointsPosition.resize(jointsPosition.size());
    for (size_t i = 0; i < jointsPosition.size(); i++) {
        pImpl->berdyData.state.jointsPosition.setVal(i, jointsPosition.at(i));
    }

    pImpl->berdyData.state.jointsVelocity.resize(jointsVelocity.size());
    for (size_t i = 0; i < jointsVelocity.size(); ++i) {
        pImpl->berdyData.state.jointsVelocity.setVal(i, jointsVelocity.size());
    }

    pImpl->berdyData.state.baseAngularVelocity.setVal(0, baseVelocity.at(3));
    pImpl->berdyData.state.baseAngularVelocity.setVal(1, baseVelocity.at(4));
    pImpl->berdyData.state.baseAngularVelocity.setVal(2, baseVelocity.at(5));

    // Wrench data is read from the attached IAnalogSensor interface
    // TODO: Check how to use the wrench data in berdy
    yarp::sig::Vector wrenchData;
    pImpl->iAnalogSensor->read(wrenchData);

    //yInfo() << LogPrefix << "Wrench data size : " << wrenchData.size();
    //yInfo() << LogPrefix << "Wrench data : " << wrenchData.toString();

    //yInfo() << LogPrefix << " sensor measurements size : " << pImpl->berdyData.buffers.sensorMeasurements.getSizeOfAllSensorsMeasurements();
    //yInfo() << LogPrefix << " number of accelerometers : " << pImpl->berdyData.buffers.sensorMeasurements.getNrOfSensors(iDynTree::ACCELEROMETER);

    // Fill in the y vector with sensor measurements for the FT sensors
    std::vector<double> wrenchValues;
    wrenchValues = pImpl->iHumanWrench->getWrenches();

    //yInfo() << LogPrefix << " number of wrench sources : " << pImpl->iHumanWrench->getNumberOfWrenchSources();
    //yInfo() << LogPrefix << " wrench source names : " << pImpl->iHumanWrench->getWrenchSourceNames().size();
    //yInfo() << LogPrefix << " wrench values : " << pImpl->iHumanWrench->getWrenches();

    // Get the berdy sensors following its internal order
    std::vector<iDynTree::BerdySensor> berdySensors = pImpl->berdyData.helper.getSensorsOrdering();

    /* The total number of sensors are :
     * 17 Accelerometers, 66 DOF Acceleration sensors and 67 NET EXT WRENCH sensors
     * The total number of sensor measurements = (17x3) + (66x1) + (67x6) = 519
     */

    // Iterate over the sensors and add corresponding measurements
    int wrenchSensor = 0;
    bool wrenchMeasurementsSet = false;
    for (const iDynTree::BerdySensor& sensor : berdySensors) {
        // Create the key
        SensorKey key = {sensor.type, sensor.id};

        // Check that it exists in the sensorMapIndex
        if (pImpl->berdyData.sensorMapIndex.find(key) != pImpl->berdyData.sensorMapIndex.end()) {
            SensorMapIndex::const_iterator found = pImpl->berdyData.sensorMapIndex.find(key);
            // Update sensor measurements vector y
            switch (sensor.type)
            {
                case iDynTree::ACCELEROMETER_SENSOR:
                {
                    // Filling y vector with zero values for ACCELEROMETER sensors
                    // TODO: Fill the correct data
                    for (int i = 0; i < 3; i++)
                    {
                        pImpl->berdyData.buffers.measurements(found->second.offset + i) = 0;
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
                    // TODO: Fill the correct data
                    if (!wrenchMeasurementsSet) {
                        std::string wrenchSensorLinkName = pImpl->wrenchSensorsLinkNames.at(wrenchSensor);
                        for (int idx = 0; idx < pImpl->humanModel.getNrOfLinks(); idx++) {
                            std::string linkName = pImpl->humanModel.getLinkName(idx);

                            if(linkName.compare(wrenchSensorLinkName) == 0) {
                                for (int i = 0; i < 6; i++)
                                {
                                    pImpl->berdyData.buffers.measurements(found->second.offset + i) = wrenchValues.at(wrenchSensor*6 + i);
                                }
                                if (wrenchSensor < 4) {
                                    wrenchSensor++;
                                    if (wrenchSensor == 4) {
                                        wrenchMeasurementsSet = true;
                                    }
                                }
                            }

                        }
                    }
                    else {
                        for (int i = 0; i < 6; i++)
                        {
                            pImpl->berdyData.buffers.measurements(found->second.offset + i) = 0;
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

    // Solve a berdy problem
    pImpl->berdyData.solver->updateEstimateInformationFloatingBase(pImpl->berdyData.state.jointsPosition,
                                                                   pImpl->berdyData.state.jointsVelocity,
                                                                   pImpl->berdyData.state.floatingFrameIndex,
                                                                   pImpl->berdyData.state.baseAngularVelocity,
                                                                   pImpl->berdyData.buffers.measurements);

    if (!pImpl->berdyData.solver->doEstimate()) {
        yError() << LogPrefix << "Failed to do berdy estimation";
    }

    // Extract the solution
    iDynTree::VectorDynSize estimatedDynamicVariables(pImpl->berdyData.helper.getNrOfDynamicVariables());
    pImpl->berdyData.solver->getLastEstimate(estimatedDynamicVariables);

    // ===========================
    // EXPOSE DATA FOR IHUMANSTATE
    // ===========================

    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);

        // Extract the joint torque estimates and store to the variable
        pImpl->berdyData.helper.extractJointTorquesFromDynamicVariables(estimatedDynamicVariables,
                                                                        pImpl->berdyData.state.jointsPosition,
                                                                        pImpl->berdyData.estimates.jointTorqueEstimates);

        //yInfo() << LogPrefix << "Estimated joint torques size : " << pImpl->berdyData.estimates.jointTorqueEstimates.size();
        //yInfo() << LogPrefix << "Estimated joint torques : " << pImpl->berdyData.estimates.jointTorqueEstimates.toString();

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

    if (deviceName == "human_state_provider") {

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
            yError() << "The IHumanState interface might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
    }

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    return true;
}

bool HumanDynamicsEstimator::detach()
{
    pImpl->iHumanState = nullptr;
    pImpl->iAnalogSensor = nullptr;
    stop();
    return true;
}

bool HumanDynamicsEstimator::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    bool attachStatus = false;
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

        attachStatus = attach(driver->poly);
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
