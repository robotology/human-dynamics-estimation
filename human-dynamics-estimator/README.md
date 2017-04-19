## YARP human-dynamics-estimator module.

Module for computing dynamic variables estimation starting from the data coming from 
[human-forces-provider](https://github.com/robotology-playground/human-dynamics-estimation/tree/master/human-forces-provider) module (forces6D) and 
from [human-state-provider](https://github.com/robotology-playground/human-dynamics-estimation/tree/master/human-state-provider) module (human kinematics data).

The module is structured as follow:

<img src="/misc/human-dynamics-estimator.png">

where:
- **MAP computation** is the function that solves the inverse dynamics problem with a maximum-a-posteriori estimation by using the Newton-Euler algorithm and 
a set of redundant sensor measurements. It needs as input the human state (joints configuration and joints velocity), the berdy iDynTree object 
and the vector (y) of measurements.
  - **y measurements**: vector of sensor measurements that includes external forces (forces6D) and data coming from accelerometres, 
  gyroscopes and DOF acceleration sensors (human kinematics data).
  - **berdy**: iDynTree object of [BerdyHelper](http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1BerdyHelper.html) class, that is a class for algorithms to compute the maximum-a-posteriori estimation of the dynamic variables 
  of a multibody model assuming the knowledge of measurements of an arbitrary set of sensors and of the kinematics and inertial characteristics of the model. 
  Berdy initialization needs as input the human model along with the information of the related sensors and berdy options.
  
**Note**: berdy initialization is done only for the first step. At each timestamp y measurements vector is filled and MAP computation is done.

## Theoretical background on MAP computation with Cholesky decomposition

The Cholesky decomposition is a decomposition of a Hermitian, positive-definite matrix into the product of a lower triangular matrix and its conjugate transpose:

	**A** = **L L'**

where **L** is a lower triangular matrix with real and positive diagonal entries, and **L'** denotes the conjugate transpose of **L**.
The Cholesky decomposition is mainly used for the numerical solution of linear equations **A x** = **b**. If **A** is symmetric and positive definite, then we can solve **A x** = **b**  by first computing the Cholesky decomposition **A** = **L L'**, then solving **L y** = **b** for **y** by forward substitution, and finally solving **L' x** = **y** for **x** by back substitution.
When **A** is sparse, a permutation matrix **P**, such that **L L'** = **P' A P**, speeds up the linear system solution. The Cholesky factorisation of **P' A P** tends to be sparser than that of **A**. 

**Eigen SimplicialLLT** class performs Cholesky factorisations of sparse matrices that are selfadjoint and positive definite. In order to reduce the fill-in, a symmetric permutation matrix **P** is applied prior to the factorisation.
When solving for several problems having the same structure the Cholesky decomposition can be divided into two steps.
First, _analyzePattern_ function performs a symbolic decomposition on the sparsity of a matrix. This function reorders the non zeros elements of the matrix, such that the factorisation step creates less fill-in. This step exploits only the structure of the matrix, hence the results of this step can be used for other linear system where the matrix has the same structure.
Second, _factorize_ function performs a numeric decomposition of the matrix, that must have the same sparsity than the matrix on which the symbolic decomposition has been performed. The factors of the matrix are computed in this step. In this way, if the sparsity of the matrix remains the same, _analyzePattern_ function is run only once, while _factorize_ function should be called each time the values of the matrix change.


