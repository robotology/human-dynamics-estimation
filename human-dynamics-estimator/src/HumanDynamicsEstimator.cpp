
//
//  HumanDynamicsEstimator.cpp
//  HumanDynamicsEstimator
//
//  Created by Claudia Latella on 03/02/17.
//  Copyright © 2017 Claudia Latella. All rights reserved.

#include <iostream>
#include <yarp/os/LogStream.h>  //for using yError()
#include <iDynTree/Estimation/BerdyHelper.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Sensors/Sensors.h>
#include "HumanDynamicsEstimator.h"


//---------------------------------------------------------------------
double HumanDynamicsEstimator::getPeriod()
{
    return 1.0; //module periodicity (in seconds)
}

//---------------------------------------------------------------------
bool HumanDynamicsEstimator::configure(yarp::os::ResourceFinder &rf)
{
    count = 0;
    /*
     * ------Human model loading and initialization
     */
    modelFilename = rf.findFile("humanModel");
    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(modelFilename))
    {
        yError() << " Something wrong with the model loading!";
        return false;
    }
    
    iDynTree::Model humanModel = modelLoader.model();
//    iDynTree::KinDynComputations human_kinDynComp;
//    if (!human_kinDynComp.loadRobotModel(humanModel))
//    {
//        yError() << " Something wrong with the model loading!";
//    }
//
    /*
     * ------Model sensors initialization
     */
    iDynTree::SensorsList humanSensors = modelLoader.sensors();
   
    std::cerr << humanModel.toString() << std::endl;
    
    
    /*
     * ------Setting options for Berdy obj
     */
    std::string base = rf.find("baseLink").asString();      // base for the model

    iDynTree::BerdyOptions berdyOpts;
    berdyOpts.checkConsistency();
    berdyOpts.baseLink = base;
    berdyOpts.includeAllNetExternalWrenchesAsSensors          = true;
    berdyOpts.includeAllNetExternalWrenchesAsDynamicVariables = true;
    berdyOpts.includeAllJointAccelerationsAsSensors           = true;
    berdyOpts.includeAllJointTorquesAsSensors                 = false;
    berdyOpts.includeFixedBaseExternalWrench                  = true;
    
    // Remove sensors that are placed on the base (not supported by Berdy)
    humanSensors.removeSensor(iDynTree::SensorType::ACCELEROMETER, base + "_accelerometer");
    humanSensors.removeSensor(iDynTree::SensorType::GYROSCOPE, base + "_gyro");

    
    /*
     * ------Berdy obj inizialization
     */
    iDynTree::BerdyHelper berdy;
    berdy.init(humanModel, humanSensors, berdyOpts);
    
    
    /*
     * ------Open port for sensor data
     */
    // TODO: port or device?
    
    
    /*
     * ------Wrapper data for a Berdy compatible form --> order of y vector
     */
     // TODO: wrapper method
    
    return true;
}

//---------------------------------------------------------------------
bool HumanDynamicsEstimator::updateModule()
{
    count++;
    std::cout<<"["<<count<<"]"<< " updateModule..."<<std::endl;
    return true;
    
    // TODO: MAP computation --> Maria's code to be integrated
}

//---------------------------------------------------------------------
bool HumanDynamicsEstimator::interruptModule()
{
    return true;
}

//---------------------------------------------------------------------
/*
 * Close function, to perform cleanup.
 */
bool HumanDynamicsEstimator::close()
{
    return true;
  
  // TODO: here it has to be implemented the code for closing
  // all the ports (input and output).
}

