#include <human-ik/InverseKinematics.h>
#include "URDFdir.h"
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <matio.h>
#include <ctime>
#include <map>

struct IKErrorLog{
    std::vector< int > timeInstants;
    std::vector< double > jointsIK;
    std::vector< double > ik_minus_opensim;
};

struct IKSolverLog{
    std::vector< int > timeInstants;
    std::vector< double > angleError;
    std::vector<double> elapsedTime;
};

int main(int argc, char **argv) {
    
    iDynTree::ModelLoader modelLoader;
    iDynTree::Model model;
    
    std::cerr<<"Load model from "<<getAbsModelPath("XSensURDF_subj1.urdf")<< std::endl;
    bool ok=modelLoader.loadModelFromFile(getAbsModelPath("XSensURDF_subj1.urdf"));
    model = modelLoader.model();
    
    iDynTree::assertTrue(ok);
    
    std::cerr <<"Model Loaded"<< std::endl;
    
    mat_t *pHumanState;

    std::cerr<<"Load human_state "<<getAbsModelPath("human_state.mat")<< std::endl;
    
    pHumanState = Mat_Open(getAbsModelPath("human_state.mat").c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pHumanState!=NULL);
    
    //LOADING OF human_state.mat
    
    matvar_t *humanStateVar;
    humanStateVar = Mat_VarRead(pHumanState,"human_state");
    iDynTree::assertTrue(humanStateVar != NULL);
    std::cerr<<"Found human_state variable"<<std::endl;
    
    matvar_t *humanStateQ;
    
    humanStateQ = Mat_VarGetStructFieldByName(humanStateVar, "q", 0);
    iDynTree::assertTrue(humanStateQ != NULL);
    
    std::cerr << "Dimensions: "<< humanStateQ->dims[0] << "x" << humanStateQ->dims[1] << std::endl;
    
    Eigen::Map< Eigen::MatrixXd > mapHumanState((double*)humanStateQ->data, humanStateQ->dims[0], humanStateQ->dims[1]);
    
    iDynTree::MatrixDynSize humanStateQi(humanStateQ->dims[0], humanStateQ->dims[1]);
    
    iDynTree::toEigen(humanStateQi) = mapHumanState;
    
    
    //LOAD OF selectedjoints.mat
    mat_t *pSelectedJoints;

    std::cerr<<"Load selectedjoints at "<<getAbsModelPath("selectedJoints.mat")<< std::endl;
    
    pSelectedJoints = Mat_Open(getAbsModelPath("selectedJoints.mat").c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pSelectedJoints!=NULL);
    std::cerr << "selectedjoints loaded" << std::endl;
    
    matvar_t *selectedJointsVar;
    selectedJointsVar = Mat_VarRead(pSelectedJoints,"selectedJoints");
    iDynTree::assertTrue(selectedJointsVar != NULL);
    std::cerr<<"Found 'selectedJoints' variable with dimension "<< selectedJointsVar->dims[0] << std::endl;
    
    matvar_t *cell;
    std::vector< std::string > selectedJointsVector;
    selectedJointsVector.resize(selectedJointsVar->dims[0]);
    
    for(int i=0; i<selectedJointsVar->dims[0]; ++i){
        
        cell = Mat_VarGetCell(selectedJointsVar, i);
        selectedJointsVector[i] = (char*)cell->data;
        
    }

    //LOAD OF suit.mat
    mat_t *pSuit;
    
    std::cerr<<"Load suit.mat at "<<getAbsModelPath("suit.mat")<< std::endl;
    pSuit = Mat_Open(getAbsModelPath("suit.mat").c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pSuit!=NULL);
    std::cerr << "suit loaded" << std::endl;
    
    matvar_t *suitVar;
    suitVar = Mat_VarRead(pSuit,"suit");
    iDynTree::assertTrue(suitVar != NULL);
    std::cerr<<"Found suit variable"<<std::endl;
    
    matvar_t *linksVar;
    linksVar = Mat_VarGetStructFieldByName(suitVar,"links",0);
    iDynTree::assertTrue(linksVar != NULL);
    std::cerr<<"Found 'links' variable with dimension "<< linksVar->dims[0] << std::endl;
    
    matvar_t *linkCell;
    
    std::vector< std::string > linksName;
    linksName.resize(linksVar->dims[0]);
    
    std::vector< iDynTree::MatrixDynSize > linksPositions;
    linksPositions.resize(linksVar->dims[0]);
    
    std::vector< iDynTree::MatrixDynSize > linksQuaternions;
    linksQuaternions.resize(linksVar->dims[0]);
    
    matvar_t *temp;
    matvar_t *tempMeas;
    
    for(int i=0; i < linksVar->dims[0] ;++i){
        
        linkCell = Mat_VarGetCell(linksVar, i);
        
        //Get Name
        temp = Mat_VarGetStructFieldByName(linkCell,"label", 0);
        iDynTree::assertTrue(temp != NULL);
        linksName[i] =(char*) temp->data;
        
        //Get meas
        tempMeas = Mat_VarGetStructFieldByName(linkCell,"meas", 0);
        iDynTree::assertTrue(tempMeas != NULL);
        
        //Get positions
        temp = Mat_VarGetStructFieldByName(tempMeas,"position", 0);
        iDynTree::assertTrue(temp != NULL);
        Eigen::Map< Eigen::MatrixXd > tempMapPosition((double*)temp->data, temp->dims[0], temp->dims[1]);
        linksPositions[i].resize(temp->dims[0], temp->dims[1]);
        toEigen(linksPositions[i]) = tempMapPosition;
        
        //Get orientations
        temp = Mat_VarGetStructFieldByName(tempMeas,"orientation", 0);
        iDynTree::assertTrue(temp != NULL);
        Eigen::Map< Eigen::MatrixXd > tempMapOrientation((double*)temp->data, temp->dims[0], temp->dims[1]);
        linksQuaternions[i].resize(temp->dims[0], temp->dims[1]);
        toEigen(linksQuaternions[i]) = tempMapOrientation;
    }
    //Deleting stuff
    std::cerr<< "Deleting human_state variables" << std::endl;
    Mat_VarFree(humanStateVar);
    
    std::cerr<<"Closing human_state.mat file."<<std::endl;
    Mat_Close(pHumanState);
    
        std::cerr<< "Deleting human_state variable" << std::endl;
    Mat_VarFree(selectedJointsVar);
    
    std::cerr<<"Closing selectedjoints.mat file."<<std::endl;
    Mat_Close(pSelectedJoints);
    
    Mat_VarFree(suitVar);
    
    std::cerr<<"Closing suit.mat file."<<std::endl;
    Mat_Close(pSuit);
    
    
    //START Test
    
    
    std::vector< human::InverseKinematics* > solvers(linksName.size()-1);
    
    for(int solverIterator = 0; solverIterator < solvers.size(); ++solverIterator){
        solvers[solverIterator] = new human::InverseKinematics();
    }
    
    
    
    std::string parentFrame, targetFrame;
    int solverIterator = 0;
    std::vector < iDynTree::JointIndex > selectedJointsList(model.getNrOfJoints());
    
    for (int i = 0; i < model.getNrOfJoints(); ++i){ //let's create a checklist will all the joint indeces
        selectedJointsList[i] = i;
    }
    
    std::vector< iDynTree::JointIndex >::iterator pickJ = selectedJointsList.begin();
    std::vector< iDynTree::JointIndex >::iterator findJ = selectedJointsList.begin();
    iDynTree::LinkIndex tempFirst, tempSecond, childLink;


    do{
        pickJ = selectedJointsList.begin();
        tempFirst = model.getJoint( *pickJ )->getFirstAttachedLink();
        tempSecond = model.getJoint( *pickJ )->getSecondAttachedLink();
        while((std::find(linksName.begin(),linksName.end(),model.getFrameName(tempFirst)) ==  linksName.end()) &&       //first find a joint which is connected to a link contained in linksName (thus does not connect two fake links)
                (std::find(linksName.begin(),linksName.end(),model.getFrameName(tempSecond)) ==  linksName.end()) ){
            
            pickJ++;
            iDynTree::assertTrue(pickJ < selectedJointsList.end());
            
            tempFirst = model.getJoint(*pickJ)->getFirstAttachedLink();
            tempSecond = model.getJoint(*pickJ)->getSecondAttachedLink();
            
        }
        
        if(std::find(linksName.begin(),linksName.end(),model.getFrameName(tempFirst)) !=  linksName.end()){ //first link is a good one
            parentFrame = model.getFrameName(tempFirst);
            childLink = tempSecond;
            selectedJointsList.erase(pickJ); //remove joints already considered
        }
        else{ //second link is the good one
            parentFrame = model.getFrameName(tempSecond);
            childLink = tempFirst;
            selectedJointsList.erase(pickJ); //remove joints already considered
        }
        
        //now we have to test whether the child is a fake link or not
        while(std::find(linksName.begin(),linksName.end(),model.getFrameName(childLink)) ==  linksName.end()){
            
            iDynTree::assertTrue(model.getNrOfNeighbors(childLink) == 2); //fake links should be in the middle between two joints
            findJ = std::find(selectedJointsList.begin(),selectedJointsList.end(), model.getNeighbor(childLink,0).neighborJoint);
            if(findJ !=  selectedJointsList.end()){
                childLink = model.getNeighbor(childLink,0).neighborLink;
                selectedJointsList.erase(findJ);
            }
            else{
                findJ = std::find(selectedJointsList.begin(),selectedJointsList.end(), model.getNeighbor(childLink,1).neighborJoint);
                iDynTree::assertTrue(findJ != selectedJointsList.end());
                childLink = model.getNeighbor(childLink,1).neighborLink;
                selectedJointsList.erase(findJ);
            }

        }
        
        targetFrame = model.getFrameName(childLink);
        
        iDynTree::assertTrue(solverIterator < solvers.size());
        std::cerr << "Solver #" << solverIterator + 1 << std::endl;
        ok = solvers[solverIterator]->setModel(model, parentFrame, targetFrame);
        iDynTree::assertTrue(ok);
        solverIterator++;
        
    }while(selectedJointsList.size()!=0);

    iDynTree::assertTrue(solverIterator == solvers.size());
    
    iDynTree::Vector3 weights;
    weights(0) = 1;
    weights(1) = 100;
    weights(2) = 0.0;
    
    std::vector< std::string > tempConsideredJoints;
    iDynTree::VectorDynSize tempDesiredJoints;
    
    for(solverIterator = 0; solverIterator < solvers.size(); ++solverIterator){ //set gains and desired joints positions
        solvers[solverIterator]->setWeights(weights);
        
        solvers[solverIterator]->getConsideredJoints(tempConsideredJoints);
        tempDesiredJoints.resize(tempConsideredJoints.size());
        tempDesiredJoints.zero();
        solvers[solverIterator]->setDesiredJointPositions(tempDesiredJoints);
    }
    
    int selectedInstant;
    iDynTree::Transform w_H_parent;
    iDynTree::Transform w_H_target;
    std::string tempParentFrame;
    std::string tempTargetFrame;
    iDynTree::Position tempPosition;
    iDynTree::Vector4 tempQuaternion;
    iDynTree::Rotation tempRotation;
    iDynTree::Transform tempTransform;
    int namePosition;
    iDynTree::VectorDynSize jointsOut;
    iDynTree::Position positionError;
    iDynTree::Rotation rotationError;
    double angleError;
    clock_t now;
    double elapsed_time;
    int exitCode;

    
      /////////////////////////
     ///// EXAMPLE RUN ///////
    /////////////////////////
    srand ( clock() );
    selectedInstant = rand() % humanStateQi.cols();  //random time instant
    std::cerr << "Example Test. Selected time instant: " << selectedInstant << std::endl;
    
    for(solverIterator = 0; solverIterator < solvers.size(); ++solverIterator){
        solvers[solverIterator]->getFrames(tempParentFrame, tempTargetFrame);
        /* std::cerr << "PreSolver #" << solverIterator << " parent:" << tempParentFrame << " target: "<< tempTargetFrame << std::endl;
        solvers[solverIterator].getConsideredJoints(tempConsideredJoints);
        std::cerr << "Considered joints are:"<< std::endl;
        for (std::vector< std::string >::const_iterator i = tempConsideredJoints.begin(); i != tempConsideredJoints.end(); ++i){
            std::cerr <<"-"<< *i << std::endl;
        }*/
        
        namePosition = std::distance(linksName.begin(), std::find(linksName.begin(),linksName.end(),tempParentFrame)); //search for its position in linksName
        
        iDynTree::toEigen(tempPosition) = iDynTree::toEigen(linksPositions[namePosition]).col(selectedInstant);
        iDynTree::toEigen(tempQuaternion) = iDynTree::toEigen(linksQuaternions[namePosition]).col(selectedInstant);
        tempRotation =  iDynTree::Rotation::RotationFromQuaternion(tempQuaternion);
        w_H_parent.setPosition(tempPosition);
        w_H_parent.setRotation(tempRotation);
        
        namePosition = std::distance(linksName.begin(), std::find(linksName.begin(),linksName.end(),tempTargetFrame)); 
        
        iDynTree::toEigen(tempPosition) = iDynTree::toEigen(linksPositions[namePosition]).col(selectedInstant);
        iDynTree::toEigen(tempQuaternion) = iDynTree::toEigen(linksQuaternions[namePosition]).col(selectedInstant);
        tempRotation =  iDynTree::Rotation::RotationFromQuaternion(tempQuaternion);
        
        w_H_target.setPosition(tempPosition);
        w_H_target.setRotation(tempRotation);
        tempTransform = w_H_parent.inverse()*w_H_target;
        solvers[solverIterator]->setDesiredTransformation(tempTransform);
        
        std::cerr << "Solver #" << solverIterator+1 << std::endl;
        now = clock();
        exitCode = solvers[solverIterator]->runIK(jointsOut);
        iDynTree::assertTrue(exitCode >= 0);
        solvers[solverIterator]->getErrors(positionError, rotationError, &angleError);
        elapsed_time = clock() - now;
        elapsed_time = elapsed_time/CLOCKS_PER_SEC;
        
        solvers[solverIterator]->getConsideredJoints(tempConsideredJoints);
        
        for(int jIterator = 0; jIterator < tempConsideredJoints.size(); ++jIterator){
            namePosition = std::distance(selectedJointsVector.begin(), std::find(selectedJointsVector.begin(),selectedJointsVector.end(), tempConsideredJoints[jIterator])); 
            std::cerr << tempConsideredJoints[jIterator] <<": (IK) " << jointsOut(jIterator)*180/M_PI << " vs " << humanStateQi(namePosition,selectedInstant)*180/M_PI << " (OpenSim)" << std::endl;
        }
        std::cerr <<"Exit code: " << exitCode << std::endl;
        std::cerr <<"Angle Error: " << angleError*180/M_PI << std::endl;
        std::cerr << "Elapsed time: "<< elapsed_time << std::endl << std::endl;
        
    }
        
      ////////////////////////////////////////////
     ///////// END OF EXAMPLE RUN ///////////////
    ////////////////////////////////////////////
    
    
    double tempDifference;
    std::string jointName;
    std::map < std::string, IKErrorLog> loggerOpensim;
    std::map < int, IKSolverLog> loggerSolver;
    int attempts = 0;
    std::vector< iDynTree::VectorDynSize > guessVector(solvers.size());
    
    for(int i=0; i<solvers.size(); ++i){
        solvers[i]->getConsideredJoints(tempConsideredJoints);
        guessVector[i].resize(tempConsideredJoints.size());
        guessVector[i].zero();
    }

    
    for(selectedInstant = 0; selectedInstant < humanStateQi.cols(); ++selectedInstant){
        std::cerr << humanStateQi.cols() << "/" << selectedInstant; 
        for(solverIterator = 0; solverIterator < solvers.size(); ++solverIterator){
            solvers[solverIterator]->getFrames(tempParentFrame, tempTargetFrame);
            /* std::cerr << "PreSolver #" << solverIterator << " parent:" << tempParentFrame << " target: "<< tempTargetFrame << std::endl;
            solvers[solverIterator].getConsideredJoints(tempConsideredJoints);
            std::cerr << "Considered joints are:"<< std::endl;
            for (std::vector< std::string >::const_iterator i = tempConsideredJoints.begin(); i != tempConsideredJoints.end(); ++i){
                std::cerr <<"-"<< *i << std::endl;
            }*/
            
            namePosition = std::distance(linksName.begin(), std::find(linksName.begin(),linksName.end(),tempParentFrame)); //search for its position in linksName
            
            iDynTree::toEigen(tempPosition) = iDynTree::toEigen(linksPositions[namePosition]).col(selectedInstant);
            iDynTree::toEigen(tempQuaternion) = iDynTree::toEigen(linksQuaternions[namePosition]).col(selectedInstant);
            tempRotation =  iDynTree::Rotation::RotationFromQuaternion(tempQuaternion);
            w_H_parent.setPosition(tempPosition);
            w_H_parent.setRotation(tempRotation);
            
            namePosition = std::distance(linksName.begin(), std::find(linksName.begin(),linksName.end(),tempTargetFrame)); 
            
            iDynTree::toEigen(tempPosition) = iDynTree::toEigen(linksPositions[namePosition]).col(selectedInstant);
            iDynTree::toEigen(tempQuaternion) = iDynTree::toEigen(linksQuaternions[namePosition]).col(selectedInstant);
            tempRotation =  iDynTree::Rotation::RotationFromQuaternion(tempQuaternion);
            
            w_H_target.setPosition(tempPosition);
            w_H_target.setRotation(tempRotation);
            tempTransform = w_H_parent.inverse()*w_H_target;
            solvers[solverIterator]->setDesiredTransformation(tempTransform);
            solvers[solverIterator]->setGuess(guessVector[solverIterator]);
            //std::cerr << "Solver #" << solverIterator+1 << std::endl;
            attempts = 0;
            now = clock();
            do{
                if(attempts > 0){
                    ok = solvers[solverIterator]->setRandomGuess(clock(), guessVector[solverIterator], guessVector[solverIterator], 0.15, 5);
                    iDynTree::assertTrue(ok);
                    //solver.setDesiredJointPositions(jointsTest);
                    //std::cerr << "Trying again with guess: " << guess.toString() << std::endl;
                }
                
                exitCode = solvers[solverIterator]->runIK(jointsOut);
                solvers[solverIterator]->getErrors(positionError, rotationError, &angleError);
                elapsed_time = clock() - now;
                elapsed_time = elapsed_time/CLOCKS_PER_SEC;
                iDynTree::assertTrue(exitCode >= 0);
                angleError = angleError*180/M_PI;
                guessVector[solverIterator] = jointsOut;
                attempts++;
            }while((attempts < 3)&&(angleError > (2*M_PI/180)));
            
            
            if(angleError > 2){
                loggerSolver[solverIterator].timeInstants.push_back(selectedInstant);
                loggerSolver[solverIterator].elapsedTime.push_back(elapsed_time);
                loggerSolver[solverIterator].angleError.push_back(angleError);
            }
            
            solvers[solverIterator]->getConsideredJoints(tempConsideredJoints);
            
            for(int jIterator = 0; jIterator < tempConsideredJoints.size(); ++jIterator){
                namePosition = std::distance(selectedJointsVector.begin(), std::find(selectedJointsVector.begin(),selectedJointsVector.end(), tempConsideredJoints[jIterator])); 
                tempDifference = jointsOut(jIterator)*180/M_PI - humanStateQi(namePosition,selectedInstant)*180/M_PI;
                
                if (tempDifference > 2){
                    jointName = tempConsideredJoints[jIterator];
                    loggerOpensim[jointName].timeInstants.push_back(selectedInstant);
                    loggerOpensim[jointName].jointsIK.push_back(jointsOut(jIterator)*180/M_PI);
                    loggerOpensim[jointName].ik_minus_opensim.push_back(tempDifference);
                }
                //std::cerr << tempConsideredJoints[jIterator] <<": (IK) " << jointsOut(jIterator)*180/M_PI << " vs " << humanStateQi(namePosition,selectedInstant)*180/M_PI << " (OpenSim)" << std::endl;
            }
            //std::cerr <<"Exit code: " << exitCode << std::endl;
            //std::cerr <<"Angle Error: " << angleError*180/M_PI << std::endl;
            //std::cerr << "Elapsed time: "<< elapsed_time << std::endl << std::endl;
            
        }
        std::cerr << "\r";
    }
    
    //////// PRINTING LOGGERS /////
    int failed =1;
    
    for(int i=0; i<selectedJointsVector.size(); ++i){
        if(loggerOpensim.count(selectedJointsVector[i])){
            std::cerr << failed << "# ";
            std::cerr  << selectedJointsVector[i] << " was different of at least 2[deg] for "<< loggerOpensim[selectedJointsVector[i]].timeInstants.size();
            std::cerr <<" times with maximum [deg] " << *std::max_element(loggerOpensim[selectedJointsVector[i]].ik_minus_opensim.begin(), loggerOpensim[selectedJointsVector[i]].ik_minus_opensim.end()) << std::endl;
            failed ++;
        }
    }
    
    int solverFailed = 1;
    std::cerr << std::endl;
    for(int i=0; i<solvers.size(); ++i){
        if(loggerSolver.count(i)){
            std::cerr << solverFailed << "* ";
            std::cerr << "Solver #" << i << " experienced angle errors greater than 2[deg] for " << loggerSolver[i].timeInstants.size() << " times with maximum [deg]" << *std::max_element(loggerSolver[i].angleError.begin(), loggerSolver[i].angleError.end()) << std::endl;
            solverFailed++;
            
        }
    }
    
    for(int solverIterator = 0; solverIterator < solvers.size(); ++solverIterator){
        delete(solvers[solverIterator]);
    }
    
}
