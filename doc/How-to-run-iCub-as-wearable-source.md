# How to run iCub as Wearable Device

This page describes how to run the [`iCub`](http://www.icub.org/) humanoid robot as a wearable device source, by exposing its sensors measurement and joints state.


The configuration file to run `iCub` as wearable device is [`ICubWearableDevice.xml`](https://github.com/robotology/wearables/blob/master/app/xml/ICubWearableDevice.xml), and it can be launched with:
```
yarprobotinterface --config ICubWearableDevice.xml
```


#### Before running
Before running the device make sure that:
- [`yarpserver`](https://www.yarp.it/yarpserver.html) is running
- The `YARP_ROBOT_NAME` is set to the proper robot name. 
- The robot is currently running (either [Gazebo simulator](https://github.com/robotology/icub-gazebo) or real robot making sure it is runned with the [proper configuration for whole-body-dynamics](https://github.com/robotology/whole-body-controllers/blob/master/doc/How-to-setup-the-robot-for-wbc-experiments.md))
- If the robot is running in simulation, the `whole-body-dynamics` estimatior has to be launched
  ```
  yarprobotinterface --config launch-wholebodydynamics.xml
  ```
  When the real robot is running, `whole-body-dynamics` estimatior should be running if the robot is launched with the [proper configuration](https://github.com/robotology/whole-body-controllers/blob/master/doc/How-to-setup-the-robot-for-wbc-experiments.md). 
  To verify if the estimator is running, you can check whether data are streamed for the end-effectors to be used as sensors (e.g. `/wholeBodyDynamics/right_arm/endEffectorWrench:o`).
- In the [configuration file](https://github.com/robotology/wearables/blob/master/app/xml/ICubWearableDevice.xml), the following parameters are set properly:
  - `ft-sensors`: wrenche measurements that have to be exposed as wearable data. Those measurements should be provided by `whole-body-dynamics` estimator, or directly from sensors measurement.
  - `joint-sensors`: joints for which the state has to be exposed as wearable data.


#### Calibration
The estimated wrench data computed by `whole-body-dynamics` may be characterized by an offset due to ft sensors meaasurement.
If using the real-robot, the following command can be sent to remove the offset (while the robot is lifted from the ground):
```
yarp rpc /wholeBodyDynamics/rpc
calib all 300
```
If instead simulated robot is used, the command is the following (while the robot is on the ground):
```
yarp rpc /wholeBodyDynamics/rpc
resetOffset all 300
```

#### Data
If the device is running correctly, the stream of wearable data can be read with:
```
/ICub/WearableData/data:o
```