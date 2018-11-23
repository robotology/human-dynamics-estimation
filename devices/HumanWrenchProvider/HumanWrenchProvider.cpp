/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanWrenchProvider.h"
#include "WrenchFrameTransformers.h"

#include "IHumanWrench.h"

#include <Wearable/IWear/IWear.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/KinDynComputations.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <mutex>
#include <string>
#include <vector>

const std::string DeviceName = "HumanWrenchProvider";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;
using namespace hde::devices::impl;

struct AnalogSensorData
{
    size_t numberOfChannels = 0;
    std::vector<double> measurements;
};

enum class WrenchSourceType
{
    Fixed,
    Robot, // TODO
};

struct WrenchSourceData
{
    std::string name;
    WrenchSourceType type;

    std::string outputFrame;
    std::unique_ptr<IWrenchFrameTransformer> frameTransformer;

    wearable::sensor::SensorName sensorName;
    wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor> wearableSensor;

    // Variables for Robot Human Transformation
    std::string humanLinkingFrame;
    std::string robotLinkingFrame;

    iDynTree::Position humanFootPosition;
};

class HumanWrenchProvider::Impl
{
public:
    mutable std::mutex mutex;
    wearable::IWear* iWear = nullptr;

    AnalogSensorData analogSensorData;
    std::vector<WrenchSourceData> wrenchSources;

    // Model variables
    iDynTree::Model humanModel;
    iDynTree::Model robotModel;
};

HumanWrenchProvider::HumanWrenchProvider()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new Impl()}
{}

// Without this destructor here, the linker complains for
// undefined reference to vtable
HumanWrenchProvider::~HumanWrenchProvider() = default;

bool parseRotation(yarp::os::Bottle* list, iDynTree::Rotation& rotation)
{
    if (list->size() != 9) {
        yError() << LogPrefix << "The list with rotation data does not contain 9 elements";
        return false;
    }

    rotation = iDynTree::Rotation(list->get(0).asDouble(),
                                  list->get(1).asDouble(),
                                  list->get(2).asDouble(),
                                  list->get(3).asDouble(),
                                  list->get(4).asDouble(),
                                  list->get(5).asDouble(),
                                  list->get(6).asDouble(),
                                  list->get(7).asDouble(),
                                  list->get(8).asDouble());
    return true;
}

bool parsePosition(yarp::os::Bottle* list, iDynTree::Position& position)
{
    if (list->size() != 3) {
        yError() << LogPrefix << "The list with position data does not contain 9 elements";
        return false;
    }

    position = iDynTree::Position(
        list->get(0).asDouble(), list->get(1).asDouble(), list->get(2).asDouble());
    return true;
}

bool HumanWrenchProvider::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    // ==========================
    // INITIALIZE THE HUMAN MODEL
    // ==========================

    const std::string humanURDFFileName = config.find("human_urdf").asString();
    auto& human_rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string humanURDFFilePath = human_rf.findFile(humanURDFFileName);
    if (humanURDFFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("human_urdf").asString();
        return false;
    }

    iDynTree::ModelLoader humanModelLoader;
    if (!humanModelLoader.loadModelFromFile(humanURDFFilePath) || !humanModelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << humanURDFFilePath;
        return false;
    }

    pImpl->humanModel = humanModelLoader.model();

    // ==========================
    // INITIALIZE THE ROBOT MODEL
    // ==========================

    const std::string robotURDFFileName = config.find("robot_urdf").asString();
    auto& robot_rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string robotURDFFilePath = robot_rf.findFile(robotURDFFileName);
    if (robotURDFFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("robot_urdf").asString();
        return false;
    }

    iDynTree::ModelLoader robotModelLoader;
    if (!robotModelLoader.loadModelFromFile(robotURDFFilePath) || !robotModelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << robotURDFFilePath;
        return false;
    }

    pImpl->robotModel = robotModelLoader.model();

    // =============================
    // INITIALIZE THE WRENCH SOURCES
    // =============================

    if (!(config.check("number_of_sources") && config.find("number_of_sources").asInt())) {
        yError() << LogPrefix << "Option 'number_of_sources' not found or not a valid Integer";
        return false;
    }

    if (!(config.check("sources") && config.find("sources").isList())) {
        yError() << LogPrefix << "Option 'sources' not found or not a valid list";
        return false;
    }

    yarp::os::Bottle* listOfSources = config.find("sources").asList();
    std::vector<std::string> sourcesNames;

    // Check the number of sources are correct
    if ( config.find("number_of_sources").asInt() != listOfSources->size()) {
        yError() << LogPrefix << " 'number_of_sources' and 'sources' list provided are not matching";
        return false;
    }
    else {
        for (unsigned i = 0; i < listOfSources->size(); ++i) {
            sourcesNames.push_back(listOfSources->get(i).asString());
            yDebug() << LogPrefix << "Found source" << sourcesNames.back();
        }
    }

    // Parse the groups using the sources list names
    for (const auto& sourceName : sourcesNames) {
        yarp::os::Bottle& sourceGroup = config.findGroup(sourceName);

        if (sourceGroup.isNull()) {
            yError() << LogPrefix << "Failed to find group" << sourceName;
            return false;
        }

        // Temporary object
        WrenchSourceData WrenchSourceData;
        WrenchSourceData.name = sourceName;

        if (!(sourceGroup.check("sensorName") && sourceGroup.find("sensorName").isString())) {
            yError() << LogPrefix << "Option" << sourceName
                     << ":: sensorName not found or not a valid string";
            return false;
        }

        WrenchSourceData.sensorName = sourceGroup.find("sensorName").asString();

        if (!(sourceGroup.check("outputFrame") && sourceGroup.find("outputFrame").isString())) {
            yError() << LogPrefix << "Option" << sourceName
                     << ":: outputFrame not found or not a valid string";
            return false;
        }

        WrenchSourceData.outputFrame = sourceGroup.find("outputFrame").asString();

        if (!(sourceGroup.check("type") && sourceGroup.find("type").isString())) {
            yError() << LogPrefix << "Option" << sourceName
                     << ":: type not found or not a valid string";
            return false;
        }

        std::string sourceType = sourceGroup.find("type").asString();
        if (sourceType == "fixed") {
            WrenchSourceData.type = WrenchSourceType::Fixed;
        }
        else if (sourceType == "robot") {
            WrenchSourceData.type = WrenchSourceType::Robot;
        }
        else {
            yError() << LogPrefix << "Option" << sourceName
                     << ":: type must be either 'fixed' or 'robot'";
            return false;
        }

        switch (WrenchSourceData.type) {
                // Process Fixed source type
                // =========================

            case WrenchSourceType::Fixed: {
                auto transformer = std::make_unique<FixedFrameWrenchTransformer>();

                if (!(sourceGroup.check("rotation") && sourceGroup.find("rotation").isList())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: rotation not found or not a valid list";
                    return false;
                }

                if (!(sourceGroup.check("position") && sourceGroup.find("position").isList())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: position not found or not a valid list";
                    return false;
                }

                iDynTree::Rotation rotation;
                iDynTree::Position position;
                if (!parseRotation(sourceGroup.find("rotation").asList(), rotation)
                    || !parsePosition(sourceGroup.find("position").asList(), position)) {
                    yError() << LogPrefix << "Failed to parse" << sourceName
                             << ":: position or rotation";
                    return false;
                }

                // Store the transform in the temporary object
                transformer->transform = {rotation, position};

                // Downcast it and move the ownership into the object containing the source data
                auto ptr = static_cast<IWrenchFrameTransformer*>(transformer.release());
                WrenchSourceData.frameTransformer.reset(ptr);

                yDebug() << LogPrefix << "=============:";
                yDebug() << LogPrefix << "New source   :" << WrenchSourceData.name;
                yDebug() << LogPrefix << "Sensor name  :" << WrenchSourceData.sensorName;
                yDebug() << LogPrefix << "Type         :" << sourceType;
                yDebug() << LogPrefix << "Output frame :" << WrenchSourceData.outputFrame;
                yDebug() << LogPrefix
                         << "Rotation     :" << sourceGroup.find("rotation").asList()->toString();
                yDebug() << LogPrefix
                         << "Position     :" << sourceGroup.find("position").asList()->toString();
                yDebug() << LogPrefix << "=============:";

                break;
            }

            // Process Robot source type
            // =========================
            case WrenchSourceType::Robot: {
                auto transformer = std::make_unique<RobotFrameWrenchTransformer>();

                if (!(sourceGroup.check("rotation") && sourceGroup.find("rotation").isList())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: rotation not found or not a valid list";
                    return false;
                }

                if (!(sourceGroup.check("position") && sourceGroup.find("position").isList())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: position not found or not a valid list";
                    return false;
                }

                iDynTree::Rotation rotation;
                iDynTree::Position position;

                if (!parseRotation(sourceGroup.find("rotation").asList(),rotation)
                        || !parsePosition(sourceGroup.find("position").asList(),position)) {
                    yError() << LogPrefix << "Failed to parse" << sourceName
                             << ":: rotation or position";
                    return false;
                }

                if (!(sourceGroup.check("humanLinkingFrame") && sourceGroup.find("humanLinkingFrame").isString())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: humanLinkingFrame not found or not a valid string";
                    return false;
                }

                WrenchSourceData.humanLinkingFrame = sourceGroup.find("humanLinkingFrame").asString();

                if (!(sourceGroup.check("robotLinkingFrame") && sourceGroup.find("robotLinkingFrame").isString())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: robotLinkingFrame not found or not a valid string";
                    return false;
                }

                WrenchSourceData.robotLinkingFrame = sourceGroup.find("robotLinkingFrame").asString();

                if (!(sourceGroup.check("humanFootPosition") && sourceGroup.find("humanFootPosition").isList())) {
                    yError() << LogPrefix << "Option" << sourceName
                             << ":: humanFootPosition not found or not a valid list";
                    return false;
                }

                if (!parsePosition(sourceGroup.find("humanFootPosition").asList(),WrenchSourceData.humanFootPosition)) {
                    yError() << LogPrefix << "Failed to parse" << sourceName
                             << ":: humanFootPosition";
                    return false;
                }

                // Store the fixed transform in the temporary object
                transformer->fixedTransform = {rotation, position};

                // Initialize the wrench transform to be Identity
                transformer->transform = iDynTree::Transform::Identity();

                // Downcast it and move the ownership into the object containing the source data
                auto ptr = static_cast<IWrenchFrameTransformer*>(transformer.release());
                WrenchSourceData.frameTransformer.reset(ptr);

                yDebug() << LogPrefix << "=============:";
                yDebug() << LogPrefix << "New source   :" << WrenchSourceData.name;
                yDebug() << LogPrefix << "Sensor name  :" << WrenchSourceData.sensorName;
                yDebug() << LogPrefix << "Type         :" << sourceType;
                yDebug() << LogPrefix << "Output frame :" << WrenchSourceData.outputFrame;
                yDebug() << LogPrefix
                         << "Rotation     :" << sourceGroup.find("rotation").asList()->toString();
                yDebug() << LogPrefix
                         << "Position     :" << sourceGroup.find("position").asList()->toString();
                yDebug() << LogPrefix <<
                            "Human foot position :" << sourceGroup.find("humanFootPosition").asList()->toString();
                yDebug() << LogPrefix << "Human linking frame :" << WrenchSourceData.humanLinkingFrame;
                yDebug() << LogPrefix << "Robot linking frame :" << WrenchSourceData.robotLinkingFrame;
                yDebug() << LogPrefix << "=============:";

                break;
            }

        }

        // Store the wrench source data
        pImpl->wrenchSources.emplace_back(std::move(WrenchSourceData));
    }

    return true;
}

bool HumanWrenchProvider::close()
{
    stop();
    detachAll();
    return true;
}

void HumanWrenchProvider::run()
{
    for (unsigned i = 0; i < pImpl->wrenchSources.size(); ++i) {
        auto& forceSource = pImpl->wrenchSources[i];

        // Get the measurement
        // ===================

        wearable::Vector3 forces;
        wearable::Vector3 torques;

        if (!forceSource.wearableSensor) {
            yError() << LogPrefix << "Failed to get wearable sensor for source" << forceSource.name;
            askToStop();
            return;
        }

        if (!forceSource.wearableSensor->getForceTorque6D(forces, torques)) {
            yError() << LogPrefix << "Failed to get measurement from sensor"
                     << forceSource.wearableSensor->getSensorName();
            askToStop();
            return;
        }

        // Tranform it to the correct frame
        // ================================

        iDynTree::Wrench inputWrench({forces[0], forces[1], forces[2]},
                                     {torques[0], torques[1], torques[2]});

        iDynTree::Wrench transformedWrench;

        if (forceSource.type == WrenchSourceType::Robot) {

            iDynTree::Transform robotFeetToHandsTransform = iDynTree::Transform::Identity();
            iDynTree::KinDynComputations robotKinDynComp;

            robotKinDynComp.loadRobotModel(pImpl->robotModel);
            robotFeetToHandsTransform = robotKinDynComp.getRelativeTransform(forceSource.robotLinkingFrame,forceSource.outputFrame);

            // TODO: Move this logic to WrenchFrameTransformers.cpp file

            // Access the tranforms through pointers
            std::lock_guard<std::mutex> lock(pImpl->mutex);

            // Downcast it and move the pointer ownership into the object of derived class
            auto transformerPtr = dynamic_cast<RobotFrameWrenchTransformer*>(forceSource.frameTransformer.release());
            std::unique_ptr<RobotFrameWrenchTransformer> newTransformer;
            newTransformer.reset(transformerPtr);

            // Access the fixed transformation stored from configuration
            auto robotHumanFeetFixedTransform = newTransformer->fixedTransform;

            // Compute the final robot to human transform
            iDynTree::Transform robotToHumanTransform;

            robotToHumanTransform = iDynTree::Transform::Identity() *
                                    iDynTree::Transform(iDynTree::Rotation::Identity(), forceSource.humanFootPosition) *
                                    robotHumanFeetFixedTransform *
                                    robotFeetToHandsTransform;

            // Update the stored transform
            newTransformer->transform = robotToHumanTransform;

            // Downcast it and move the pointer ownership into the object containing the source data
            auto ptr = static_cast<IWrenchFrameTransformer*>(newTransformer.release());
            forceSource.frameTransformer.reset(ptr);
        }

        if (!forceSource.frameTransformer->transformWrenchFrame(inputWrench, transformedWrench)) {
            askToStop();
            return;
        }

        // Expose the data as IAnalogSensor
        // ================================
        {
            std::lock_guard<std::mutex> lock(pImpl->mutex);

            pImpl->analogSensorData.measurements[6 * i + 0] = transformedWrench.getLinearVec3()(0);
            pImpl->analogSensorData.measurements[6 * i + 1] = transformedWrench.getLinearVec3()(1);
            pImpl->analogSensorData.measurements[6 * i + 2] = transformedWrench.getLinearVec3()(2);
            pImpl->analogSensorData.measurements[6 * i + 3] = transformedWrench.getAngularVec3()(0);
            pImpl->analogSensorData.measurements[6 * i + 4] = transformedWrench.getAngularVec3()(1);
            pImpl->analogSensorData.measurements[6 * i + 5] = transformedWrench.getAngularVec3()(2);
        }
    }
}

bool HumanWrenchProvider::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->iWear || !poly->view(pImpl->iWear) || !pImpl->iWear) {
        yError() << LogPrefix << "Failed to view the IWear interface from the PolyDriver";
        return false;
    }

    while (pImpl->iWear->getStatus() == wearable::WearStatus::WaitingForFirstRead) {
        yInfo() << LogPrefix << "IWear interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
    }

    if (pImpl->iWear->getStatus() != wearable::WearStatus::Ok) {
        yError() << LogPrefix << "The status of the attached IWear interface is not ok ("
                 << static_cast<int>(pImpl->iWear->getStatus()) << ")";
        return false;
    }

    // TODO: switch to FourceSourceData

    // Get the wearable sensors containing the input measurements
    for (auto& sensorSourceData : pImpl->wrenchSources) {
        auto sensor = pImpl->iWear->getForceTorque6DSensor(sensorSourceData.sensorName);

        if (!sensor) {
            yError() << LogPrefix << "Failed to get sensor" << sensorSourceData.sensorName
                     << "from the attached IWear interface";
            return false;
        }

        sensorSourceData.wearableSensor = sensor;
    }

    // Initialize the number of channels of the equivalent IAnalogSensor
    const size_t numberOfFTSensors = pImpl->wrenchSources.size();
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        pImpl->analogSensorData.measurements.resize(6 * numberOfFTSensors, 0);
        pImpl->analogSensorData.numberOfChannels = 6 * numberOfFTSensors;
    }

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

bool HumanWrenchProvider::detach()
{
    askToStop();

    pImpl->iWear = nullptr;
    return true;
}

bool HumanWrenchProvider::attachAll(const yarp::dev::PolyDriverList& driverList)
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

bool HumanWrenchProvider::detachAll()
{
    return detach();
}

// =============
// IAnalogSensor
// =============

int HumanWrenchProvider::read(yarp::sig::Vector& out)
{
    out.resize(pImpl->analogSensorData.measurements.size());

    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        std::copy(pImpl->analogSensorData.measurements.begin(),
                  pImpl->analogSensorData.measurements.end(),
                  out.data());
    }

    return IAnalogSensor::AS_OK; // TODO
}

int HumanWrenchProvider::getState(int ch)
{
    // Check if channel is in the right range
    if (ch < 0 || ch > pImpl->wrenchSources.size() - 1) {
        yError() << LogPrefix << "Failed to get status for channel" << ch;
        yError() << LogPrefix << "Channels must be in the range 0 -"
                 << pImpl->wrenchSources.size() - 1;
        return IAnalogSensor::AS_ERROR;
    }

    // Get the sensor associated with the channel
    const auto& sensorData = pImpl->wrenchSources[ch];

    if (!sensorData.wearableSensor) {
        yError() << "The wearable sensor for this channel was not allocated";
        return IAnalogSensor::AS_ERROR;
    }

    // Map the wearable sensor status to IAnalogSensor status
    switch (sensorData.wearableSensor->getSensorStatus()) {
        case wearable::WearStatus::Error:
            return IAnalogSensor::AS_ERROR;
        case wearable::WearStatus::Ok:
            return IAnalogSensor::AS_OK;
        case wearable::WearStatus::Calibrating:
            return IAnalogSensor::AS_TIMEOUT;
            ;
        case wearable::WearStatus::Overflow:
            return IAnalogSensor::AS_OVF;
        case wearable::WearStatus::Timeout:
            return IAnalogSensor::AS_TIMEOUT;

        case wearable::WearStatus::Unknown:
            return IAnalogSensor::AS_ERROR;

        case wearable::WearStatus::WaitingForFirstRead:
            return IAnalogSensor::AS_TIMEOUT;
    };
}

int HumanWrenchProvider::getChannels()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->analogSensorData.numberOfChannels;
}

int HumanWrenchProvider::calibrateSensor()
{
    return IAnalogSensor::AS_ERROR;
}

int HumanWrenchProvider::calibrateSensor(const yarp::sig::Vector& /*value*/)
{
    return IAnalogSensor::AS_ERROR;
}

int HumanWrenchProvider::calibrateChannel(int /*ch*/)
{
    return IAnalogSensor::AS_ERROR;
}

int HumanWrenchProvider::calibrateChannel(int /*ch*/, double /*value*/)
{
    return IAnalogSensor::AS_ERROR;
}

// ============
// IHumanWrench
// ============

std::vector<std::string> HumanWrenchProvider::getWrenchSourceNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    std::vector<std::string> sourcesNames;
    for (size_t idx = 0; idx < pImpl->wrenchSources.size(); idx++) {
        sourcesNames.emplace_back(pImpl->wrenchSources.at(idx).name);
    }

    return sourcesNames;
}

size_t HumanWrenchProvider::getNumberOfWrenchSources() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->wrenchSources.size();
}

std::vector<double> HumanWrenchProvider::getWrenches() const
{
    std::vector<double> wrenchValues;

    std::lock_guard<std::mutex> lock(pImpl->mutex);
    size_t vecSize = pImpl->analogSensorData.measurements.size();
    wrenchValues.resize(vecSize);
    for (size_t idx = 0; idx < vecSize; idx++) {
        wrenchValues.at(idx) = pImpl->analogSensorData.measurements.at(idx);
    }

    return wrenchValues;
}


