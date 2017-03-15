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

//---------------------------------------------------------------------------
// Utility function for parsing INI file
static bool parseFrameListOption(const yarp::os::Value &option, std::vector<std::string> &parsedJoints);

//---------------------------------------------------------------------
HumanDynamicsEstimator::HumanDynamicsEstimator()
:m_period(0.1){};

//---------------------------------------------------------------------

HumanDynamicsEstimator::~HumanDynamicsEstimator(){};

//---------------------------------------------------------------------
double HumanDynamicsEstimator::getPeriod()
{
    return 1.0; //module periodicity (in seconds)
}

//---------------------------------------------------------------------
bool HumanDynamicsEstimator::configure(yarp::os::ResourceFinder &rf)
{
    std::string moduleName = rf.findFile("name");
    
    /*
     * ------Open a port:i for the human joint configuration
     */
    if (!m_humanJointConfiguration_port.open("/humanJointConfiguration:i"))
    {
        yError() << "Unable to open port /humanJointConfiguration:i";
        return false;
    }
    
    if (!yarp::os::Network::connect("/human-state-provider/state:o","/humanJointConfiguration:i"))
    {
                yError() << "Error! Could not connect to server ";
                return false;
    }
        
//     /* 
//      * ------Open a port:i for the human forces
//      */    
//     if (!m_humanForces_port.open("/humanForces:i"))
//     {
//         yError() << "Unable to open port /humanForces:i";
//         return false;
//     }   
// 
//     if (!yarp::os::Network::connect("    ","/humanForces:i"))       // name port 
//     {
//                 yError() << "Error! Could not connect to server ";
//                 return false;
//     }
    
    /*
     * ------Human model loading 
     */  
    yarp::os::Value defaultOffline; defaultOffline.fromString("true");
    bool offline = rf.check("offline", defaultOffline, "Checking offline mode").asBool();
    
    std::vector<std::string> joints;
    // TODO: substitution with thrift
    if (!offline) 
    {
        //online reading
    } 
    else 
    {
        if (!parseFrameListOption(rf.find("jointsList"), joints)) 
        {
            yError() << "Error while parsing joints list";
            return false;  
        }      
    }     
    
    std::string modelFilename = rf.findFile("humanModel");
    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadReducedModelFromFile(modelFilename, joints))
    {
        yError() << " Something wrong with the model loading!";
        return false;
    }
        
    iDynTree::Model humanModel = modelLoader.model();
    
    
    for (int i=0; i<humanModel.getNrOfLinks(); ++i) 
    {
        m_linkName.push_back(humanModel.getLinkName(i));
    }
    for (int i=0; i<humanModel.getNrOfJoints(); ++i) 
    {
        m_jointName.push_back(humanModel.getJointName(i));
    }
    
    /*
     * ------Human model initialization
     */
    if (!m_humanComputations.loadRobotModel(humanModel))
    {
        yError("Error in reading the human model!");
        return false;
    }
        
    m_q.resize(m_humanComputations.getNrOfDegreesOfFreedom());
    m_dq.resize(m_humanComputations.getNrOfDegreesOfFreedom());        
    
    /*
     * ------Model sensors initialization
     */
    iDynTree::SensorsList humanSensors = modelLoader.sensors();
     
    /*
     * ------Setting options and inizialization for Berdy obj
     */
    std::string base = rf.find("baseLink").asString();      // base for the model

    iDynTree::BerdyOptions berdyOpts;
    berdyOpts.baseLink = base;
    berdyOpts.includeAllNetExternalWrenchesAsSensors          = true;
    berdyOpts.includeAllNetExternalWrenchesAsDynamicVariables = true;
    berdyOpts.includeAllJointAccelerationsAsSensors           = true;
    berdyOpts.includeAllJointTorquesAsSensors                 = false;
    berdyOpts.includeFixedBaseExternalWrench                  = true;
    berdyOpts.checkConsistency();    
    
    // Remove sensors that are placed on the base (not supported by Berdy)
    humanSensors.removeSensor(iDynTree::ACCELEROMETER, base + "_accelerometer");
    humanSensors.removeSensor(iDynTree::GYROSCOPE, base + "_gyro");

    m_berdy.init(humanModel, humanSensors, berdyOpts);
    
    // Dimensions
    m_dynV = m_berdy.getNrOfDynamicVariables();   
    m_eq = m_berdy.getNrOfDynamicEquations();
    m_meas = m_berdy.getNrOfSensorsMeasurements();
        
    /*
     *  ------Creating a map to fill the y measurements vector with the order used in berdy
     */
    std::vector <iDynTree::BerdySensor> berdySensors; 
    berdySensors = m_berdy.getSensorsOrdering();
    for (int i=0; i<berdySensors.size(); ++i) 
    {
        m_y_map[berdySensors[i].id] = berdySensors[i].range;
    }

    // y measurements vector inizialization
    m_y.resize(m_meas);
    m_y.zero();

    /*
     * ------Set priors
     */
    m_sigmaD_prior.resize(m_eq,m_eq);
    m_sigmaD_prior.reserve(m_eq);    
    iDynTree::Triplets Tris_eq;
    double defaultSigmaD = 1e-4;
    double constSigmaD = rf.check("constSigmaD", defaultSigmaD).asDouble();
    if (constSigmaD <= 0)
    {
        yError("value for SigmaD is negative");
        return false;
    }    
    Tris_eq.addDiagonalMatrix(0,0,1/constSigmaD,m_eq);
    m_sigmaD_prior.setFromTriplets(Tris_eq);
    
    m_sigmad_prior.resize(m_dynV,m_dynV);
    m_sigmad_prior.reserve(m_dynV); 
    iDynTree::Triplets Tris_dynV;  
    double defaultSigmad = 1e+4;
    double constSigmad = rf.check("constSigmad", defaultSigmad).asDouble();
    if (constSigmad <= 0)
    {
        yError("value for Sigmad is negative");
        return false;
    } 
    Tris_dynV.addDiagonalMatrix(0,0,1/constSigmad,m_dynV);
    m_sigmad_prior.setFromTriplets(Tris_dynV);
     
    m_sigmay_prior.resize(m_meas,m_meas);
    m_sigmay_prior.reserve(m_meas);
    iDynTree::Triplets Tris_meas;   
    Tris_meas.addDiagonalMatrix(0,0,1,m_meas);
    m_sigmay_prior.setFromTriplets(Tris_meas);  
    for (int j=0; j<berdySensors.size(); ++j)
    {
        switch(berdySensors[j].type) 
        {
            case iDynTree::ACCELEROMETER_SENSOR:
            {
                double varAcc[] = {rf.find("constSigmaYacc").asDouble()};
                for (int i=0; i<3; ++i)
                {
                    m_sigmay_prior(berdySensors[j].range.offset+i,berdySensors[j].range.offset+i) *= varAcc[i]; 
                }
                break;
            } 
            case iDynTree::DOF_ACCELERATION_SENSOR:
            {
                m_sigmay_prior(berdySensors[j].range.offset,berdySensors[j].range.offset) *= rf.find("constSigmaYddq").asDouble(); 
                break;
            }
            case iDynTree::NET_EXT_WRENCH_SENSOR:
            {
                m_sigmay_prior(berdySensors[j].range.offset,berdySensors[j].range.offset) *= rf.find("constSigmaYfext").asDouble();
                if (berdySensors[j].id == "LeftFoot" || berdySensors[j].id == "RightFoot" || berdySensors[j].id == "LeftHand" || berdySensors[j].id == "RightHand" ) 
                {
                    double varFknown[] = {rf.find("constSigmaYfknown").asDouble()};
                    for (int i=0; i<6; ++i)
                    {
                        m_sigmay_prior(berdySensors[j].range.offset+i,berdySensors[j].range.offset+i) *= varFknown[i] / (rf.find("constSigmaYfext").asDouble()) ; // EXT KNOWN FORCES
                    }
                }
            }          
        }
    }
 
    m_mud_prior.resize(m_dynV);
    m_mud_prior.zero();

    /*
    * ------Set berdy matrices and gravity
    */
    m_berdy.resizeAndZeroBerdyMatrices(m_D, m_bD, m_Y, m_bY);    

    m_gravity.zero(); 
    m_gravity(2) = -9.81;    
    
    m_rhs_BarD.resize(m_dynV);
    m_mu_BarD.resize(m_dynV);    
    m_rhs.resize(m_dynV);    
    m_mu_dgiveny.resize(m_dynV); 
    
    /* 
     * ------Creating map for the d dynamics variables vector
     */  
    m_berdyDynVariables = m_berdy.getDynamicVariablesOrdering();
   
    for (int index=0; index < m_berdyDynVariables.size(); ++index) 
    {
        switch(m_berdyDynVariables[index].type) 
        {
            case iDynTree::LINK_BODY_PROPER_ACCELERATION: 
            case iDynTree::NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            case iDynTree::JOINT_WRENCH:
            case iDynTree::NET_EXT_WRENCH:
            {    
                BerdyOutputRangeMap &map = m_link_map[m_berdyDynVariables[index].id];
                map[static_cast<int>(m_berdyDynVariables[index].type)] = m_berdyDynVariables[index].range;
                break;
            }
           
            case iDynTree::DOF_TORQUE:
            case iDynTree::DOF_ACCELERATION:
            {    
                BerdyOutputRangeMap &map = m_joint_map[m_berdyDynVariables[index].id];
                map[static_cast<int>(m_berdyDynVariables[index].type)] = m_berdyDynVariables[index].range;
                break;
            }
        }
    }    

    /* 
     * ------Open and prepare a port:o for the output
     */     
    if (! m_outputPort.open("/humanDynamics:o"))
    {
        yError() << "Unable to open port /humanDynamics:o";
        return false;
    } 
    
    m_firststep = true;    
    
    return true;
}

//---------------------------------------------------------------------
bool HumanDynamicsEstimator::updateModule()
{   
    /*
    * ------Read from ports
    */
    human::HumanState *tmp_state = m_humanJointConfiguration_port.read();     
    if (tmp_state == NULL)
    {
        yError("Something wrong in reading the human configuration port!");
        return false;
    }
            
    // transform yarp Vector in iDynTree vector
    if (!iDynTree::toiDynTree(tmp_state->positions,m_q))
    {
        yError("Somenthing wrong in the conversion from a YARP to iDynTree vector!");
        return false;
    }

    // transform yarp Vector in iDynTree vector           
    if (!iDynTree::toiDynTree(tmp_state->velocities,m_dq))
    {
         yError("Somenthing wrong in the conversion from a YARP to iDynTree vector!");
         return false;
    }

//     human::HumanForces *tmp_force = m_humanForces_port.read();
// 
//     if (tmp_force == NULL)
//     {
//         yError("Something wrong in reading the human forces port!");
//         return false;
//     }
    
//     for (std::vector<human::Force6D>::iterator it(tmp_force->forces.begin()); it!=tmp_force->forces.end(); ++it ) 
//     {
// //         std::string appliedLink = it->appliedLink;
//         std::unordered_map<std::string,iDynTree::IndexRange>::const_iterator found = m_y_map.find(it->appliedLink);
//         if (found->second.size != 6)
//         {
//             yError("Somenthing wrong with the reading of the forces");
//             return false;
//         }
//         m_y(found->second.offset) = it->fx;
//         m_y(found->second.offset +1) = it->fy;
//         m_y(found->second.offset +2) = it->fz;
//         m_y(found->second.offset +3) = it->ux;
//         m_y(found->second.offset +4) = it->uy;
//         m_y(found->second.offset +5) = it->uz;
//     }
    
    //TODO: fill y with measurements from ACCELEROMETER, GYROSCOPE and DOF_ACCELERATION.
    // At this stage they are 0!  

    /*
     * Set the kinematic information necessary for the dynamics estimation
     */
    m_berdy.updateKinematicsFromTraversalFixedBase(m_q,m_dq,m_gravity);
    
    /*
     * Get berdy matrices
     */
    m_berdy.getBerdyMatrices(m_D,m_bD,m_Y,m_bY);    
    
    /*
     * MAP computation
     */ 
    //TODO: create a MAP function/class
    Eigen::SparseMatrix<double> sigma_BarD_inv(m_dynV,m_dynV);   

    Eigen::SparseMatrix<double> sigma_dgiveny_inv(m_dynV,m_dynV);

    sigma_BarD_inv = (toEigen(m_D).transpose() * toEigen(m_sigmaD_prior) * toEigen(m_D)) + toEigen(m_sigmad_prior); 
   
    if(m_firststep == true) 
    {                    
        m_CholeskyLU_sigma_BarD_inv.analyzePattern(sigma_BarD_inv);  
    }
    m_CholeskyLU_sigma_BarD_inv.factorize(sigma_BarD_inv); 
    
    toEigen(m_rhs_BarD) = (toEigen(m_sigmad_prior) * toEigen(m_mud_prior)) - ((toEigen(m_D).transpose() * (toEigen(m_sigmaD_prior) * toEigen(m_bD))));
    toEigen(m_mu_BarD) = m_CholeskyLU_sigma_BarD_inv.solve(toEigen(m_rhs_BarD)); 
    
    sigma_dgiveny_inv = sigma_BarD_inv + (toEigen(m_Y).transpose() * (toEigen(m_sigmay_prior) * toEigen(m_Y)));
      
    if(m_firststep==true) 
    {
        m_CholeskyLU_sigma_dgiveny_inv.analyzePattern(sigma_dgiveny_inv); 
        m_firststep = false;
    }
    m_CholeskyLU_sigma_dgiveny_inv.factorize(sigma_dgiveny_inv);

    toEigen(m_rhs) = (toEigen(m_Y).transpose() * (toEigen(m_sigmay_prior) * (toEigen(m_y) - toEigen(m_bY)))) + (sigma_BarD_inv * toEigen(m_mu_BarD));    
    toEigen(m_mu_dgiveny) = m_CholeskyLU_sigma_dgiveny_inv.solve(toEigen(m_rhs));
    /*
     * Prepare the port:o
     */
    human::HumanDynamics &output = m_outputPort.prepare();

    //fill the outputVector by using the template thrift for the struct LinkDynamicsEstimation 
    for(int index =0; index < m_linkName.size(); ++index) 
    {
        BerdyOutputMap::const_iterator found_firstMap = m_link_map.find(m_linkName[index]);
        
        // TODO: if(found_firstMap!)
        
        human::LinkDynamicsEstimation &linkEstimate = output.linkVariables[m_linkName[index]];
        
        BerdyOutputRangeMap::const_iterator found_secondMap = found_firstMap->second.find(static_cast<int>(m_berdyDynVariables[index].type));
        // TODO: if(found_secondMap!)
        Eigen::Map<Eigen::VectorXd> outputVector(linkEstimate.spatialAcceleration.data(), linkEstimate.spatialAcceleration.size());
        outputVector = (toEigen(m_mu_dgiveny)).segment(found_secondMap->second.offset, found_secondMap->second.size);
       
        found_secondMap = found_firstMap->second.find(static_cast<int>(m_berdyDynVariables[index].type));
        // TODO: if(found_secondMap!)
        new(&outputVector) Eigen::Map<Eigen::VectorXd>(linkEstimate.netWrench.data(), linkEstimate.netWrench.size());
        outputVector = (toEigen(m_mu_dgiveny)).segment(found_secondMap->second.offset, found_secondMap->second.size);
       
        found_secondMap = found_firstMap->second.find(static_cast<int>(m_berdyDynVariables[index].type));
        // TODO: if(found_secondMap!)
        new(&outputVector) Eigen::Map<Eigen::VectorXd> (linkEstimate.transmittedWrench.data(), linkEstimate.transmittedWrench.size());
        outputVector = toEigen(m_mu_dgiveny).segment(found_secondMap->second.offset, found_secondMap->second.size);
        
        found_secondMap = found_firstMap->second.find(static_cast<int>(m_berdyDynVariables[index].type));
        // TODO: if(found_secondMap!)
        new(&outputVector) Eigen::Map<Eigen::VectorXd> (linkEstimate.externalWrench.data(), linkEstimate.externalWrench.size());
        outputVector = toEigen(m_mu_dgiveny).segment(found_secondMap->second.offset, found_secondMap->second.size);
    }

    //fill the outputVector by using the template thrift for the struct JointDynamicsEstimation    
    for(int index =0; index < m_jointName.size(); ++index) 
    {
        BerdyOutputMap::const_iterator found_firstMap = m_joint_map.find(m_jointName[index]);
        // TODO: if(found_firstMap!)
        
        human::JointDynamicsEstimation &jointEstimate = output.jointVariables[m_jointName[index]];
        
        BerdyOutputRangeMap::const_iterator found_secondMap = found_firstMap->second.find(static_cast<int>(m_berdyDynVariables[index].type));
        // TODO: if(found_secondMap!)
        Eigen::Map<Eigen::VectorXd> outputVector(&jointEstimate.torque,1);
        outputVector = toEigen(m_mu_dgiveny).segment(found_secondMap->second.offset, found_secondMap->second.size);
        
        found_secondMap = found_firstMap->second.find(static_cast<int>(m_berdyDynVariables[index].type));
        // TODO: if(found_secondMap!)
        new(&outputVector) Eigen::Map<Eigen::VectorXd>(&jointEstimate.acceleration,1);
        outputVector = toEigen(m_mu_dgiveny).segment(found_secondMap->second.offset, found_secondMap->second.size);
    }   
    
    /*
    * ------Write on port:o
    */
    m_outputPort.write();
    
    return true; 
}

//---------------------------------------------------------------------
bool HumanDynamicsEstimator::close()
{
    
    //TODO: release allocated memory with outputVector?
    
    m_outputPort.close();
//     m_humanForces_port.close();
    m_humanJointConfiguration_port.close();
    
    return true;
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

    for (int i = 0; i < frames->size(); ++i) 
    {
        if (frames->get(i).isString()) 
        {
            parsedJoints.push_back(frames->get(i).asString());
        }
    }
    return true;
}
