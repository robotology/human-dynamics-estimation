# How to run Dynamics Estimation
This page describes how to run the dynamics estimation for a human subject equipped with kinematic sensors and sensorized shoes.

The configuration file to run the Dynamics Estimation is [`Human.xml`](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/xml/Human.xml), and it can be launched with:
```bash
yarprobotinterface --config Human.xml
```

### Before running
Before running the device make sure that:
- [`yarpserver`](https://www.yarp.it/yarpserver.html) is running
- In the [configuration file](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/xml/Human.xml), the following parameters are set propery:
  - `wearableDataPorts`: Is the name of the port streaming the required wearable data
  - `urdf`: Name of the urdf file corresponding to the subject (some models can be found in [`human-gazebo`](https://github.com/robotology/human-gazebo))
  - `MODEL_TO_DATA_LINK_NAMES`: describes the map between model frame names and wearable kineamtic data with the following format
    ```xml
    <param name="ENTRY_NAME">(MODEL_FRAME_NAME, WEARABLE_SENSOR_NAME)</param>
    ```
- The following wearable data sources are available:
  - `XsensSuit` ([How to run Xsens suit](https://github.com/robotology/wearables/blob/master/doc/How-to-run-XsensSuit.md))
  - `FTShoes` [How to run FTShoes](https://github.com/robotology/wearables/blob/master/doc/How-to-run-FTshoes.md)

### Calibration
For kinematic calibration, please refere to [Inverse Kinematics Calibration](/doc/how-to-run-inverse-kinematics.md#calibration).

### Output
If the device is running correctly, the terminal will show the frequency at which the dynamical inverse kinematics step is running.
The computed model configuration is accessible reading the following port:
```bash
yarp read ... /HDE/HumanStateServer/state:o
```
The computed joint torques are accessible reading the following port:
```bash
yarp read ... /HDE/HumanDynamicsServer/torques:o
```

## Visualization
Two visualizer are currently supported.

### Human State Visualizer
The [human state visualizer](https://github.com/robotology/human-dynamics-estimation/tree/master/modules/HumanStateVisualizerWithDynamics) is an application based on `iDynTree` visualizer.

In order to run the visualization:
- Set the correct subject urdf model in the [configuration file](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/app/HumanStateVisualizerWithDynamics.ini)
- Run the following commnad
  ```
  HumanStateVisualizer --from HumanStateVisualizerWithDynamics.ini
  ```

This visualizer will show both the **model configuration** and the **measured wrenches**.

https://user-images.githubusercontent.com/35487806/133111079-dda6f367-7665-40da-ac68-ff0f484af461.mov


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
  yarprobotstatepublisher --period 0.0001 --name-prefix Human --tf-prefix /Human/ --model humanSubject01_66dof.urdf --reduced-model true --base-frame Pelvis --jointstates-topic "/Human/joint_states"
  ```
- Set the correct subject urdf model in the [`rivz` configuration file](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/ros/launch/HDERviz.launch)
- Run `rviz` with the following commands
  ```bash
  roslaunch HDERviz HDERviz.launch
  ```

This visualizer will show both the **model configuration**, **measured wrenches**, and **joint torques**.

## Reference

~~~
Simultaneous Floating-Base Estimation of Human Kinematics and Joint Torques.
Latella, C., Traversaro, S., Ferigo, D., Tirupachuri, Y., Rapetti, L., Andrade Chavez, F. J., Nori F., Pucci, D.
Sensors, 19(12), 2794., 2019, doi:
10.3390/s19122794
https://www.mdpi.com/1424-8220/19/12/2794
~~~

The bibtex code for including this citation is provided:

~~~
@article{latella2019simultaneous,
  title={Simultaneous floating-base estimation of human kinematics and joint torques},
  author={Latella, Claudia and Traversaro, Silvio and Ferigo, Diego and Tirupachuri, Yeshasvi and Rapetti, Lorenzo and Andrade Chavez, Francisco Javier and Nori, Francesco and Pucci, Daniele},
  journal={Sensors},
  volume={19},
  number={12},
  pages={2794},
  year={2019},
  publisher={Multidisciplinary Digital Publishing Institute}
}
~~~
