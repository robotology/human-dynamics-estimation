# How to run Inverse Kinematics
This page describes how to run the inverse kineamtics for a human subject equipped with kinematic sensors.

The configuration file to run the Inverse Kinematics is [`HumanStateProvider.xml`](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/xml/HumanStateProvider.xml), and it can be launched with:
```
yarprobotinterface --config HumanStateProvider.xml
```

### Before running
Before running the device make sure that:
- [`yarpserver`](https://www.yarp.it/yarpserver.html) is running
- [`TransformServer`](https://www.yarp.it/git-master/classTransformServer.html) is running. In order to run it 
  ```bash
  yarprobotinterface --config TransformServer.xml
  ```
- In the [configuration file](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/xml/HumanStateProvider.xml), the following parameters are set propery:
  - `wearableDataPorts`: Is the name of the port streaming the required wearable data
  - `urdf`: Name of the urdf file corresponding to the subject (some models can be found in [`human-gazebo`](https://github.com/robotology/human-gazebo))
  - `MODEL_TO_DATA_LINK_NAMES`: describes the map between model frame names and wearable kineamtic data with the following format
    ```xml
    <param name="ENTRY_NAME">(MODEL_FRAME_NAME, WEARABLE_SENSOR_NAME)</param>
    ```
- The following wearable data sources are available:
  - `XsensSuit` ([How to run Xsens suit](https://github.com/robotology/wearables/blob/master/doc/How-to-run-XsensSuit.md))

### Calibration
There exist different calibration procedure that can be used. In order to send a calibration command you need to open the `huamn-state-provider` `rpc` port
```bash
yarp rpc /HumanStateProvider/rpc:i 
```
the available calibraiton procedure can be browsed sending `help` command to the port
```bash
>> help
```

### Output
If the device is running correctly, the terminal will show the frequency at which the dynamical inverse kinematics step is running.
The computed model configuration is accessible reading the following port:
```bash
yarp read ... /HDE/HumanStateWrapper/state:o
```

## Visualization
Two visualizer are currently supported.

### Human State Visualizer
The [human state visualizer](https://github.com/robotology/human-dynamics-estimation/tree/master/modules/HumanStateVisualizer) is an application based on `iDynTree` visualizer.

In order to run the visualization:
- Set the correct subject urdf model in the [configuration file](https://github.com/robotology/human-dynamics-estimation/blob/master/conf/app/HumanStateVisualizer.ini)
- Run the following commnad
  ```bash
  HumanStateVisualizer --from HumanStateVisualizer.ini
  ```
  
https://user-images.githubusercontent.com/35487806/133111845-11abbf1c-deff-4c0e-a2f7-072fa764a09e.mov


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
  
https://user-images.githubusercontent.com/35487806/133117050-2eee9f6b-3c85-428f-a122-e29eaf9456b8.mp4

## Reference

~~~
Model-Based Real-Time Motion Tracking Using Dynamical Inverse Kinematics.
Rapetti, L., Tirupachuri, Y., Darvish, K., Dafarra, S., Nava, G., Latella, C. Pucci, D.
Algorithms, 13(10), 266, 2020, doi:
10.3390/a13100266
https://www.mdpi.com/1999-4893/13/10/266
~~~

The bibtex code for including this citation is provided:

~~~
@article{rapetti2020model,
  title={Model-based real-time motion tracking using dynamical inverse kinematics},
  author={Rapetti, Lorenzo and Tirupachuri, Yeshasvi and Darvish, Kourosh and Dafarra, Stefano and Nava, Gabriele and Latella, Claudia and Pucci, Daniele},
  journal={Algorithms},
  volume={13},
  number={10},
  pages={266},
  year={2020},
  publisher={Multidisciplinary Digital Publishing Institute}
}
~~~
