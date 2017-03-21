#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "InverseKinematicsV2IPOPT.h"
#include <string>

class InverseKinematics {

    Ipopt::SmartPtr< InverseKinematicsV2IPOPT > solverPointer;
    Ipopt::SmartPtr< Ipopt::IpoptApplication > loader;
    bool updated;
    bool initialized;
    bool alreadyOptimized;
    std::string _parentFrame;
    std::string _targetFrame;
    unsigned verbosityLevel;
    unsigned maxIterations;

    bool update();
    bool getReducedModel(const iDynTree::Model& fullModel, const std::vector< std::string >& consideredJoints, iDynTree::Model& modelOutput);
    bool getFrameIndeces(const std::string& parentFrame, const std::string& endEffectorFrame, const iDynTree::Model model, iDynTree::FrameIndex& parentFrameIndex, iDynTree::FrameIndex& endEffectorFrameIndex);
    void removeUnsupportedJoints(const iDynTree::Model& modelInput, iDynTree::Model& modelOutput);
    bool autoSelectJointsFromTraversal(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, iDynTree::Model& modelOutput);
    
    iDynTree::VectorDynSize tempGuess; //auxiliary variable for random initialization
    iDynTree::VectorDynSize lastSolution;

    //private copy constructor
    InverseKinematics(const InverseKinematics&);
    InverseKinematics& operator=(const InverseKinematics&);

public:
    
    /**
     * \brief Constructor
     * It creates an InverseKinematicsV2IPOPT object together with an istance of IpoptApplication.
     */
    InverseKinematics();
    
    /**
     * \brief Constructor
     * It creates an InverseKinematicsV2IPOPT object together with an istance of IpoptApplication, while selecting solverName as a solver. 
     * \param[in] solverName The name of the solver.
     * \warning The solver should be installed within Ipopt, or the file libhsl.so should be available, pointing the right solver. An error will be thrown otherwise.
     * 
     */
    InverseKinematics(const std::string& solverName);
    
    /**
     * Destructor
     */
    ~InverseKinematics();

    /**
     * \brief Allows to define the file from which to load the model, and the frame to be used as reference and target when computing inverse kinematics.
     * \param[in] URDFfilename It is the adress of the .xml file cointaining the description of the robot model.
     * \param[in] parentFrame Name of the frame to be considered as a reference when computing inverse kinematics.
     * \param[in] endEffectorFrame Name of the frame to be used as a target when computing inverse kinematics.
     * \param[in] consideredJoints List of joints to be considered when computing inverse kinematics. In the case the first element of this vector cointains "All" (or "ALL" or "all"), 
     * all the joints described in the URDF are considered.
     * \note If consideredJoints is empty, the joints between parentFrame and endEffectorFrame will be automatically selected.
     * \warning Only 1DOF joints are supported. All the other joints will be automatically ignored. A warning message will be thrown in this case.
     * \return It returns true in the case the model has been successfully loaded. An error messaged is thrown otherwise.
     */
    bool setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame, const std::vector< std::string >& consideredJoints);
    
    /**
     * \brief Allows to define the file from which to load the model, and the frame to be used as reference and target when computing inverse kinematics. 
     * It automatically selects all the joints between
     * parentFrame and endEffectorFrame.
     * \param[in] URDFfilename It is the adress of the .xml file cointaining the description of the robot model.
     * \param[in] parentFrame Name of the frame to be considered as a reference when computing inverse kinematics.
     * \param[in] endEffectorFrame Name of the frame to be used as a target when computing inverse kinematics.
     * \warning Only 1DOF joints are supported. All the other joints will be automatically ignored. A warning message will be thrown in this case.
     * \return It returns true in the case the model has been successfully loaded. An error messaged is thrown otherwise.
     */
    bool setModelfromURDF(const std::string& URDFfilename, const std::string& parentFrame, const std::string& endEffectorFrame);

    /**
     * \brief Allows to define directly the model the frames to be used when computing inverse kinematics.
     * \param[in] modelInput The model to be loaded.
     * \param[in] parentFrame Name of the frame to be considered as a reference when computing inverse kinematics.
     * \param[in] endEffectorFrame Name of the frame to be used as a target when computing inverse kinematics.
     * \param[in] autoSelectJoints (Default TRUE) If true, it lets the solver to automatically select the joints between parentFrame and endEffectorFrame from those already available in the model.
     * \warning Only 1DOF joints are supported. All the other joints will be automatically ignored. A warning message will be thrown in this case.
     * \return It returns true in the case the model has been successfully loaded. An error messaged is thrown otherwise.
     */
    bool setModel(const iDynTree::Model& modelInput, const std::string& parentFrame, const std::string& endEffectorFrame, const bool autoSelectJoints = true);
    
    /**
     * \brief (Optional) Set the desired weights to be used during the optimization process.
     * \param[in] weights It is a 3D vector which weights in a different way the square norm on the position error (first element), the square norm of the orientation error expressed in quaternion 
     * (second element), and the square norm between the actual and the desired joints'configuration.
     * \note The default is [100, 100, 0.01];
     */
    void setWeights(const iDynTree::Vector3& weights);
    
    /**
     * \brief Defines the desired position for the endEffectorFrame origin.
     * \param[in] desiredPosition Desired 3D position for the origin of endEffectorFrame, expressed in the parentFrame frame of reference.
     */
    void setDesiredPosition(const iDynTree::Position& desiredPosition);
    
    /**
     * \brief Defines the desired quaternion.
     * It expresses the orientation the endEffectorFrame should have with respect to the parentFrame.
     * \param[in] desiredQuaternion It defines the orientation the endEffectorFrame should have, expressed in the parentFrame frame of reference;
     */
    void setDesiredQuaternion(const iDynTree::Vector4& desiredQuaternion); 
    
    /**
     * \brief Defines the relative transformation the endEffectorFrame should have with respect to the parentFrame.
     * \param[in] p_H_t The position and orientation the endEffectorFrame should have with respect to the parentFrame, based on which the inverse kinematics should be computed.
     */
    void setDesiredTransformation(const iDynTree::Transform& p_H_t);

    /**
     * \brief Defines the desired parent and effector frame transformations w.r.t. a common generic frame
     * \param[in] w_H_p The position and orientation of the parent frame with respect to a generic frame
     * \param[in] w_H_t The position and orientation the endEffectorFrame should have with respect to a generic frame
     */
    void setDesiredParentFrameAndEndEffectorTransformations(const iDynTree::Transform& w_H_p, const iDynTree::Transform& w_H_t);
    
    /**
     * \brief (Optional) Defines the desired set of joints values to which the result should be closed to.
     * \param[in] desiredJoints A vector of dimension equal to the number of DoFs of the considered model.
     * This terms allows to define a sort of regularization in case of redundancy.
     * \note The default is a zero vector.
     */
    void setDesiredJointPositions(const iDynTree::VectorDynSize& desiredJoints);
    
    /**
     * \brief (Optional) Defines the optimization starting point
     * \param[in] guess A vector of dimension equal to the number of Dofs of the considered model.
     * By defining it, the optimization starts from the point defined in guess. This can be useful in case a local minima is detected. The definition of a guess is optional. If it is not defined, 
     * desiredJoints will be used as initial point. Thus, guess allows to define a particular starting point, without affecting the regularization term. 
     * \note At the end of the optimization, the guess is automatically deleted. Thus, if the optimization is started again without specifying a guess, desiredJoints will be used as initial point.
     */
    void setGuess(const iDynTree::VectorDynSize& guess);
    
    /**
     * \brief (Optional) Set a random guess.
     * \param[in] feed Random number feed. It initialize the pseudo-random number generator.
     * \param[out] guess The guess that will be used as initialization during the optimization.
     * It works as setGuess, with the difference that the guess is chosen automatically, respecting joints limits. It can be useful to reinitialize the optimizer in order to avoid local minima.
     * \return false when the the model has not been loaded yet, thus the joints limits are not defined.
     * \warning It may perform dynamic memory allocation, depending on the size of guess.
     */
    bool setRandomGuess(const double feed, iDynTree::VectorDynSize& guess);
    
    /**
     * \brief (Optional) Set a random guess.
     * \param[in] feed Random number feed. It initialize the pseudo-random number generator.
     * \param[out] guess The guess that will be used as initialization during the optimization.
     * \param[in] oldGuess A previous guess.
     * \param[in] minDistance The minimum "distance" the new guess should be from oldGuess.
     * \param[in] maxIter The maximum number of iteration available to find a new guess. Default is 10
     * It works as setRandomGuess, with the difference that the guess is chosen to be far enough from oldGuess. In particular, the 2-norm of the difference between guess and oldGuess, normalized by the
     * number of DoFs, will be greater than minDistance. It may happen that minDistance is too high, thus to avoid infinite loop, a maximum on the number of iterations is inserted. Its default value is 10.
     * \note A reasonable value for minDistance is 0.3.
     * \return false when the the model has not been loaded yet, thus the joints limits are not defined, or when the size of oldGuess is not correct.
     * \warning It may perform dynamic memory allocation, depending on the size of guess (if it is different from the number of DoFs).
     */
    bool setRandomGuess(const double feed, iDynTree::VectorDynSize& guess, const iDynTree::VectorDynSize& oldGuess, double minDistance, const int maxIter = 10);
    
    /**
     * \brief Update the problem between two optimization runs.
     * \param[in] weights It is a 3D vector which weights in a different way the square norm on the position error (first element), the square norm of the orientation error expressed in quaternion 
     * (second element), and the square norm between the actual and the desired joints'configuration.
     * \param[in] desiredPosition Desired 3D position for the origin of endEffectorFrame, expressed in the parentFrame frame of reference.
     * \param[in] desiredQuaternion It defines the orientation the endEffectorFrame should have, expressed in the parentFrame frame of reference;
     * \param[in] desiredJoints A vector of dimension equal to the number of DoFs of the considered model.
     * \return false when:
     *                    - The model has not been loaded yet,
     *                    - The dimension of desiredJoints id different from the number of model DoFs,
     *                    - The desiredQuaternion is not an unitary quaternion.
     * 
     * In case one of the above mentioned case occurs, a corresponding error is thrown.
     *
     */
    bool update(const iDynTree::Vector3& weights, const iDynTree::Position &desiredPosition, const iDynTree::Vector4 &desiredQuaternion, const iDynTree::VectorDynSize &desiredJoints);
    
    /**
     * \brief Runs the inverse kinematics
     * \param[out] jointsOut The result of the optimization.
     * It automatically update the problem when setWeights, setDesiredPosition, setDesiredQuaternion, setDesiredTransformation or setDesiredJointPositions are used.
     * \note If one of the above mentioned function is not called properly, an error could be thrown when runIK is called, failing in the same case of update method. It deals automatically with multiple 
     * runs, avoiding destroying the solver objects between consecutive runs.
     *
     * \return Exit Code:
     *                  - ( 0) Success 
     *                  - ( 1) Stop at acceptable point (still a success)
     *                  - (-1) Infeasible problem
     *                  - (-2) Premature stop (either due to timeout or when the solver is converging too slowly)
     *                  - (-3) Invalid data insertion
     *                  - (-4) Ipopt internal problem
     *                  - (-5) User requested stop
     *                  - (-6) Error during initialization or solver not started at all.
     * 
     */
    signed int runIK(iDynTree::VectorDynSize& jointsOut);

    signed int runIK();

    iDynTree::VectorDynSize& getLastSolution();

    bool initialize();

    /**
     * \brief Obtain the errors between desired and actual values after the success of the solver.
     * \param[out] positionError The 3D position error obtained by subtracting the 3D target position (obtained by forward kinematics from jointsOut) from the desiredPosition.
     * \param[out] rotationError Rotation matrix obtained by multiplicating the rotation matrix obtained from jointsOut times the inverse of the desired rotation.
     * \param[out] angleError The angle in radiants of the axis-angle transformation from the desired rotation to the actual one.
     * 
     *\note angleError can be useful to detect local minima. In the global optima, this value should be less than 0.05 radiants.
     */
    bool getErrors(iDynTree::Position& positionError, iDynTree::Rotation& rotationError, double* angleError);
    
    /**
     * \brief Retrieve the names of the frames used when performing inverse kinematics
     * \param[out] parentFrame The name of the parent frame
     * \param[out] endEffectorFrame The name of the target frame
     */
    void getFrames(std::string& parentFrame, std::string& endEffectorFrame);
    
    /**
     * \brief Retrieve the set of joints considered when performing inverse kinematics
     * This can be useful when the solver is left free to select the joints when passing the model, or to understad the order adopted in jointsOut vector.
     */
    bool getConsideredJoints(std::vector< std::string >& consideredJoints);

    void setVerbosityLevel(unsigned level);

    void setMaxIterations(unsigned maxIteration);
};
#endif /* end of include guard: INVERSEKINEMATICS_H */
