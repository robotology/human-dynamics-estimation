##YARP module for the Human Dynamics Estimation (HDE).

[![Build Status](https://travis-ci.org/robotology-playground/human-dynamics-estimation.svg?branch=master)](https://travis-ci.org/robotology-playground/human-dynamics-estimation)

Human Dynamics Estimation (HDE) is a YARP module architecture for the estimation of the dynamics in humans while are physically interacting with a robot.

### Rationale
The HDE theoretical background is described [here](http://www.mdpi.com/1424-8220/16/5/727).  HDE is the *on-line* evolution of the Matlab code present in [MAPest](https://github.com/claudia-lat/MAPest) repository.  The general idea is to be able in real-time to estimate the forces acting on the human body during an physical interaction with a robot. 
A ROS visualizer allows to visualize in real-time the interaction.

### Overview
A general overview of HDE is described as follows: 
- a [human-state-provider]() module;
- a [human-forces-provider]() module;
- a [human-dynamics-estimator]() module;
- a [xxxx]() module for the visualization.

Except for the robot, raw data are typically coming from drivers.

### Dependencies