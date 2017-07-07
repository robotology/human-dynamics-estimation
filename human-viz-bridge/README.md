## Tools for interfacing HDE module architecture with ROS RViz.

In order to visualize but also to assess the results of the HDE module architecture, some YARP modules that take as input Xsens system, human state, and human dynamics data and publish them in the form of ros topics (so as to be read from Rviz) have been implemented.

Information about RViz can be found here: [http://wiki.ros.org/rviz](http://wiki.ros.org/rviz).

Information about how using yarp with ROS can be found here: [yarp with ROS](http://www.yarp.it/yarp_with_ros.html).

The I/O scheme of these modules is the following:

<img src="/misc/human-viz-bridge.png">

Each module is described in detail below.

- **HumanTFBridge** is the module in charge of reading from the Xsens system YARP interface data regarding the position and the orientation in the global reference frame of each human segment and publishing them in the form of ros topic. 
The workflow is the following:
	- the urdf model of the human is loaded and a publisher to the topic "/tf" is opened;
	- since the urdf model we use includes "real" and "fake" segments to deal with joints characterised by more degreees of freedom, a list of the "real" segments is read from the module configuration file and put in a vector; then, comparing the list of all the segments taken from the urdf with the former list also a vector of the "fake" segments is created;
	- a class tf2_msgs_TFMessage compatible with ROS is created containing the coordinate frame transforms for each segment of the urdf model;
	- for the fake segments the class is filled with an identity transform since these segments are not visible in RViz while for the real segments is filled with data coming from the Xsens system YARP interface. These transforms are updated by means of a callback on a class BufferedPort < XsensSegmentsFrame > of the Xsens data and then published by means of a Publisher < tf2_msgs_TFMessage > class.
NB: the module period is set to 100 seconds since the data updating is controlled by the callback.

- **HumanJointStateBridge** is the module in charge of reading from the human-state-provider module data regarding the state of the human in the form of joints angles and the transform between the human base link and the global reference frame and publishing them in the form of ros topic. 
The workflow is the following:
	- the urdf model of the human is loaded and publishers to the topic "/tf" and to the topic "/joint_state" are opened;
	- the list of the joints is read both from the module configuration file and from the urdf model and then the two list are compared to check possible mismatches; then, a vector containing all joints is created; 
	- a class tf2_msgs_TFMessage compatible with ROS is created containing the transform between the base link and the global reference frame; 
	- a class sensor_msgs_jointState compatible with ROS is creted containing the state of each joint of the urdf model;
	- these class are filled with data coming from human-state-provider and updated by means of a callback on a class BufferedPort < HumanState > of the human state data and then published by means of a Publisher < tf2_msgs_TFMessage > class and a Publisher < sensor_msgs_jointState > class.
  NB: the module period is set to 100 seconds since the data updating is controlled by the callback.

- **HumanEffortBridge** is the module in charge of reading from the human-dynamics-estimator module data regarding the dynamics of the human in the form of joint torques and publishing them in the form of ros topic.
The workflow is the following:
	- the urdf model of the human is loaded and multiple publishers to the topic "/< JointName >" are opened;
	- the list of the joints whose efforts the user wants to display and the list of the relative links are read from the module configuration file while the list of all the joints is read from the urdf model;
	- multiple classes sensor_msgs_Temperature compatible with ROS are created containing all the joints torques the user wants to display;
	- these classes are filled with data coming from human-dynamics-estimator and updated by means of a callback on a class BufferedPort < HumanDynamics > of the human dynamics data and then published by means of multiple Publisher < sensor_msgs_Temperature > classes. For each "real" joint, the joints torques on each	degree of freedom of such "real" joint are summed together.
  NB: the module period is set to 100 seconds since the data updating is controlled by the callback. 

- **RobotBasePosePublisher** is the module in charge of reading the human-state-provider data regarding the transform between the human base link and the global reference frame and the robot state, and then, publishing the transform between the robot base link and the global reference frame on their basis.
The workflow is the following:
	- the list of the robot joints is read from the module configuration file and the urdf model of the robot is loaded;
	- a publisher to the topic "/tf" is opened;
	- in the configuration section of the module the transform between the human base link and the global reference frame is set to an identity transform, the transform between the robot kinematic frame and the human base link is read from configuration file and the transform; 
	- in the updating section even the transform between the robot base link and the robot kinematic frame is computed and the final transformation between the robot base link and the global reference frame as a product of the former transforms is computed. Each time the user use the command "setRobotPose" the transform between the human base link and the global reference frame is updated and thus the final transform.

NB: In the **HumanTFBridge** and in the **HumanJointStateBridge** modules when the ros topics are opened a prefix is added in order to display simultaneously in Rviz the same urdf model.
	
## How to use these modules with RViz

- Make sure you have ROS installed on your computer.
- if you do not have a workspace for catkin, create one following the [tutorial] (http://wiki.ros.org/catkin/Tutorials/create_a_workspace) in ros documentation;
- [on Terminal 1] move to the src folder in the catkin workspace you have created and create a ros package with the following dependencies: 
```
catkin_create_pkg <package_name> catkin roscpp rospy message_generation std_msgs geometry_msgs sensor_msgs tf urdf 
```
- Move to the catkin_ws folder and run catkin_make:
```
cd ~/catkin_ws
catkin_make
```
- [on Terminal 2] launch `roscore`

- [on Terminal 3] launch rviz:
```
rosrun rviz rviz
```
- In order to launch urdf models, possible related publishers and rviz itself, copy the launch files and the human-viz-bridge.rviz in the ros package you have created and use this command (each time for each launch file in a different terminal):
```
roslaunch <ros_package> <displayFiles.launch>
```
