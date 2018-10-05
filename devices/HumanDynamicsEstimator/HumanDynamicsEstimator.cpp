/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanDynamicsEstimator.h"

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
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

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

    //    void computeMaximumAPosteriori(bool computePermutation);

    // TODO where put these
    // ====================

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

    std::unordered_map<SensorKey, iDynTree::IndexRange, SensorKeyHash> sensorMapIndex;
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

static bool parsePriorsGroup(const yarp::os::Bottle& priorsGroup, BerdyData& berdyData)
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

        // Store the value into the berdyData
        berdyData.priors.dynamicsConstraintsCovarianceInverse.setFromTriplets(
            covDynConstraintsTriplets);
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

        // Store the value into the berdyData
        berdyData.priors.dynamicsRegularizationCovarianceInverse.setFromTriplets(
            covDynVariablesTriplets);
    }

    // ----------------------------------
    // Priors on measurements constraints
    // ----------------------------------

    std::map<iDynTree::BerdySensorTypes, std::string> mapBerdySensorType = {
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

        // TODO: @Yeshi for some reason this find fails but the option is there
        if (!priorsGroup.find(covMeasurementOptionPrefix + berdySensorTypeString).isNull()) {
            yDebug() << "HERE";
            // TODO: log messages
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
    }

    // Store the priors of the sensors
    berdyData.priors.measurementsCovarianceInverse.setFromTriplets(allSensorsTriplets);
    // TODO: store this later into the solver as done with the other sparse matices

    return true;
}

static bool parseSensorRemovalGroup(const yarp::os::Bottle& sensorRemovalGroup,
                                    iDynTree::SensorsList& sensorList)
{
    // =================
    // CHECK THE OPTIONS
    // =================

    if (sensorRemovalGroup.isNull()) {
        yError() << LogPrefix << "Failed to find the SENSOR_REMOVAL options group";
        return false;
    }

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

    mutable std::mutex mutex;
    iDynTree::Vector3 gravity;
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

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.find("period").asFloat64();
    std::string urdfFileName = config.find("urdf").asString();
    std::string baseLink = config.find("baseLink").asString();

    // TODO: print

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

    // Initialize the sensors
    iDynTree::SensorsList humanSensors = modelLoader.sensors();

    // If any, remove the sensors from the SENSORS_REMOVAL option
    if (!parseSensorRemovalGroup(config.findGroup("SENSORS_REMOVAL"), humanSensors)) {
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
    BerdyData berdyData;
    if (!berdyData.helper.init(modelLoader.model(), humanSensors, berdyOptions)) {
        yError() << LogPrefix << "Failed to initialize BERDY";
        return false;
    }

    // Initialize the BerdySolver
    berdyData.solver = std::make_unique<iDynTree::BerdySparseMAPSolver>(berdyData.helper);
    berdyData.solver->initialize();

    if (!berdyData.solver->isValid()) {
        yError() << LogPrefix << "Failed to initialize the Berdy MAP solver";
        return false;
    }

    // Initialize buffers of the state
    berdyData.state.jointsPosition = iDynTree::JointPosDoubleArray(berdyData.helper.model());
    berdyData.state.jointsVelocity = iDynTree::JointDOFsDoubleArray(berdyData.helper.model());
    berdyData.state.jointsAcceleration = iDynTree::JointDOFsDoubleArray(berdyData.helper.model());

    // Get the berdy sensors following its internal order
    std::vector<iDynTree::BerdySensor> berdySensors = berdyData.helper.getSensorsOrdering();

    // Create a map that describes where are the sensors measurements in the y vector
    // in terms of index offset and range
    for (const iDynTree::BerdySensor& sensor : berdySensors) {
        // Create the key
        BerdyData::SensorKey key = {sensor.type, sensor.id};

        // Check that it is unique
        if (berdyData.sensorMapIndex.find(key) != berdyData.sensorMapIndex.end()) {
            yWarning() << "The sensor" << sensor.id
                       << "has been alredy inserted. Check the urdf model for duplicates. "
                          "Skipping it.";
            continue;
        }

        // Insert the sensor index range
        berdyData.sensorMapIndex.insert({key, sensor.range});

        // TODO print yInfo messages for notifying that the sensor was parsed
    }

    // Load the priors
    if (!parsePriorsGroup(config.findGroup("PRIORS"), berdyData)) {
        yError() << LogPrefix << "Failed to parse PRIORS group";
        return false;
    }

    // Set the priors into the berdy solver.
    // The sizes or the priors are set in the parsePriorsGroup function.
    // TODO: what aboyut setMeasurementsPriorCovariance?
    berdyData.solver->setDynamicsRegularizationPriorExpectedValue(
        berdyData.priors.dynamicsRegularizationExpectedValue);
    // TODO: @Yeshi The following for some reason crashes during runtime
    berdyData.solver->setDynamicsRegularizationPriorCovariance(
        berdyData.priors.dynamicsRegularizationCovarianceInverse);
    berdyData.solver->setDynamicsConstraintsPriorCovariance(
        berdyData.priors.dynamicsConstraintsCovarianceInverse);

    berdyData.buffers.measurements.resize(berdyData.helper.getNrOfSensorsMeasurements());

    // ----------------------------
    // Run a first dummy estimation
    // ----------------------------

    iDynTree::FrameIndex baseIx = berdyData.helper.model().getFrameIndex(baseLink);

    if (baseIx == iDynTree::FRAME_INVALID_INDEX) {
        yError() << LogPrefix << "Passed frame" << baseLink << "not found in the model";
        return false;
    }

    // Fill random data
    iDynTree::getRandomVector(berdyData.state.jointsPosition);
    iDynTree::getRandomVector(berdyData.state.jointsVelocity);
    iDynTree::getRandomVector(berdyData.state.baseAngularVelocity);
    iDynTree::getRandomVector(berdyData.buffers.measurements);

    // Solve a dummy problem
    berdyData.solver->updateEstimateInformationFloatingBase(berdyData.state.jointsPosition,
                                                            berdyData.state.jointsVelocity,
                                                            berdyData.state.floatingFrameIndex,
                                                            berdyData.state.baseAngularVelocity,
                                                            berdyData.buffers.measurements);

    if (!berdyData.solver->doEstimate()) {
        yError() << LogPrefix << "Failed to launch a first dummy estimation";
        return false;
    }

    // Extract the solution
    iDynTree::VectorDynSize estimatedDynamicVariables(berdyData.helper.getNrOfDynamicVariables());
    berdyData.solver->getLastEstimate(estimatedDynamicVariables);

    // TODO: @Yeshi this is more or less the behavior of the run() function.
    //       The only difference is that the kinematic state is read from the
    //       attached IHumanState interface, and the human forces from IHumanWrench.

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
    // TODO @Yeshi take inspiration from the dummy estimation performed at the end of the open()
    // method
}

bool HumanDynamicsEstimator::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    // TODO: Attach with IHumanState and IHumanWrench interfaces coming respectively
    //       from HumanStateProvider and HumanWrenchProvider

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    yInfo() << LogPrefix << "attach() successful";
    return true;
}

bool HumanDynamicsEstimator::detach()
{
    stop();
    return true;
}

bool HumanDynamicsEstimator::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool HumanDynamicsEstimator::detachAll()
{
    return detach();
}
