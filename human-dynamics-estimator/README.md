## YARP human-dynamics-estimator module.

Module for computing dynamic variables estimation starting from the data coming from 
[human-forces-provider](https://github.com/robotology-playground/human-dynamics-estimation/tree/master/human-forces-provider) module (forces6D) and 
from [human-state-provider](https://github.com/robotology-playground/human-dynamics-estimation/tree/master/human-state-provider) module (human kinematics data).

The module is structured as follow:

<img src="/misc/human-dynamics-estimator.png">

where:
- **MAP computation** is the function that solves the inverse dynamics problem with a maximum-a-posteriori estimation by using the Newton-Euler algorithm and 
a set of redundant sensor measurements. It needs as input the human state (q and $$\dot{q}$$), the berdy iDynTree object and the vector (y) of measurements
  - **y measurements**: vector of sensor measurements that includes external forces (forces6D) and data coming from accelerometres, 
  gyroscopes and DOF acceleration sensors (human kinematics data).
  - **berdy**: iDynTree object of [BerdyHelper](http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1BerdyHelper.html) class, that is a class for algorithms to compute the maximum-a-posteriori estimation of the dynamic variables 
  of a multibody model assuming the knowledge of measurements of an arbitrary set of sensors and of the kinematics and inertial characteristics of the model. 
  Berdy initialization needs as input the human model along with the information of the related sensors and berdy options.
  
**Note**: berdy initialization is done only for the first step. At each timestamp y measurements vector is filled and MAP computation is done.
