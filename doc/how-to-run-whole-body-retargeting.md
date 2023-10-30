# How to run Whole-Body Geometric Retargeting
This page describes how to run the geometric retargeting of human motion into a robot model, using kinematic sensors.

The configuration file to run the Whole-Body Retargeting for iCub2.5 robot is [`RobotStateProvider_iCub2_5.xml`](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/xml/RobotStateProvider_iCub2_5.xml), and it can be launched with:
```
yarprobotinterface --config RobotStateProvider_iCub2_5.xml
```

### Before running
Before running the device make sure that:
- [`yarpserver`](https://www.yarp.it/yarpserver.html) is running
- [`TransformServer`](https://www.yarp.it/git-master/classTransformServer.html) is running. In order to run it 
  ```bash
  yarprobotinterface --config TransformServer.xml
  ```
- In the [`RobotStateProvider` configuration file](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/xml/RobotStateProvider_iCub2_5.xml) (the example file is given for iCub2.5 robot), the following parameters are set propery:
  - `wearableDataPorts`: Is the name of the port streaming the required wearable data
  - `urdf`: Name of the urdf file corresponding to the robot
  - `MODEL_TO_DATA_LINK_NAMES`: describes the map between model frame names and wearable kineamtic data with the following format
    ```xml
    <param name="ENTRY_NAME">(MODEL_FRAME_NAME, WEARABLE_SENSOR_NAME)</param>
    ```
- `CUSTOM_CONSTRAINTS` are set for the robot if required.
- The following wearable data sources are available:
  - `XsensSuit` ([How to run Xsens suit](https://github.com/robotology/wearables/blob/master/doc/How-to-run-XsensSuit.md))

### Calibration
For kinematic calibration, please refere to [Inverse Kinematics Calibration](/doc/how-to-run-inverse-kinematics.md#calibration).

### Output
If the device is running correctly, the terminal will show the frequency at which the dynamical inverse kinematics step is running.
The computed model configuration is accessible reading the following port:
```bash
yarp read ... /iCub/RobotStateServer/state:o
```

## Visualization
Two visualizer are currently supported.

### Human State Visualizer
The [human state visualizer](https://github.com/robotology/human-dynamics-estimation/tree/master/modules/HumanStateVisualizerWithDynamics) is an application based on `iDynTree` visualizer.

In order to run the visualization:
- Set the correct subject urdf model in the [configuration file](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/app/HumanStateVisualizer_iCub2_5.ini)
- Run the following commnad
  ```bash
  HumanStateVisualizer --from HumanStateVisualizer_iCub2_5.ini
  ```
  
https://user-images.githubusercontent.com/35487806/133115690-872fcffb-4d58-454a-865f-713e6a1ae536.mov

### RViz Visualizer
Data can be visualized on [`rviz`](http://wiki.ros.org/rviz) trough the following steps:
- Launch [`yarpserver` with `ros`](http://www.yarp.it/git-master/yarp_with_ros_nameservers.html) with the following two commands
  ```bash
  roscore
  ```
  ```bash
  yarpserver --ros
  ```
- Launch the wearable sources and inverse kinematics as described above
- Launch the [`yarprobotstatepublisher`](https://github.com/robotology/idyntree/tree/master/src/tools/yarprobotstatepublisher) with the following options
  ```bash
  yarprobotstatepublisher --period 0.0001 --name-prefix iCub --tf-prefix /iCub/ --model teleoperation_iCub_model_V_2_5.urdf --reduced-model true --base-frame root_link_fake --jointstates-topic "/iCub/joint_states"
  ```
- Set the correct subject urdf model in the [`rivz` configuration file](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/ros/launch/iCubRviz.launch)
- Run `rviz` with the following commands
  ```bash
  roslaunch HDERviz iCubRviz.launch
  ```

## Robot Control
It is possible to direcly control the robot position with the computed configuration.
In order to do so:
- make sure that the robot is running withing the same `yarp` network, exposing [remoteControlBoard](http://www.yarp.it/git-master/classRemoteControlBoard.html)
- Set the correct port/joint names in the [configuration file](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/xml/RobotPositionController_iCub.xml)
- Run the `RobotPositionController` with the following command
  ```bash
  yarprobotinterface --config RobotPositionController_iCub.xml
  ``` 

## Reference

~~~
Whole-Body Geometric Retargeting for Humanoid Robots.
Darvish, K., Tirupachuri, Y., Romualdi, G., Rapetti, L., Ferigo, D., Chavez, F. J. A., Pucci, D.
IEEE-RAS 19th International Conference on Humanoid Robots (Humanoids) (pp. 679-686), 2019, doi:
10.1109/Humanoids43949.2019.9035059
https://ieeexplore.ieee.org/abstract/document/9035059
~~~

The bibtex code for including this citation is provided:

~~~
@inproceedings{darvish2019whole,
  title={Whole-body geometric retargeting for humanoid robots},
  author={Darvish, Kourosh and Tirupachuri, Yeshasvi and Romualdi, Giulio and Rapetti, Lorenzo and Ferigo, Diego and Chavez, Francisco Javier Andrade and Pucci, Daniele},
  booktitle={2019 IEEE-RAS 19th International Conference on Humanoid Robots (Humanoids)},
  pages={679--686},
  year={2019},
  organization={IEEE}
}
~~~
