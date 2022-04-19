## HapticGlove

Here we are exposing the [SenseGlove SDK](https://github.com/Adjuvo/SenseGlove-API) as a wearable device. This device expose the interfaces in order to stream hand motion data and setting the haptic feedback to the glove through the YARP netwrok.

The following interfaces are exposed for each hand/glove:

- human hand joint anlges (computed by SenseGlove SDK) `[20 DoF (each finger 4 DoF), rad]`
- glove fingertip poses attached to the human fingertip wrt human frame `5 pose vectors [<position, quaternion>]`
- glove hand pose (using the hand IMU data) wrt human hand inertial frame`Pose [<position, quaternion>]`
- hand fingertip vibrotactile feedbacks `[5 values: 0-100]`
- hand fingertip forceFeedback feedbacks `[5 values: 0-40 N]`
- hand palm vibrotactile feedback `[1 value: `[`Enumerated in "senseGlove::ThumperCmd"`](./include/SenseGloveHelper.hpp)`]`


### Dependencies

- Sense Glove SDK
```
git clone https://github.com/Adjuvo/SenseGlove-API
cd SenseGlove-API
git checkout baad587d4e165d7bfafd7f0c8ee6a1ce8ad651d6
```
In Linux machine add following environment variable:

```
export SenseGlove_DIR= <path tho the SenseGlove-API Folder>
export PATH=${PATH}:${SenseGlove_DIR}/Core/SGCoreCpp/lib/linux/<build type(Release or Debug)> 
```
In Windows machine following environment variable:

```
set SenseGlove_DIR= <path tho the SenseGlove-API Folder>
set PATH=${PATH}:${SenseGlove_DIR}/Core/SGCoreCpp/lib/win/<build type(Release or Debug)> 
```

### BUILD

Enable the option `ENABLE_HapticGlove` in Wearables when you are building it. Install the repository as well.

### RUN


If you have installed correctly the Wearabes repository, and you have set the environment varaibles as mentioned in [Superbuild](https://github.com/robotology/robotology-superbuild)
you can run the HapticGlove wearable device by the following command:


- Run the following SenseGlove communication executable:
    - Linux: run `SenseCom.x86_64` located in `<path tho the SenseGlove-API Folder>/SenseCom/Linux`
    - Windows: run `SenseCom` located in `<path tho the SenseGlove-API Folder>/SenseCom/Win`
    
- Then run:
```
yarprobotinterface --config HapticGlove.xml
```

To check and visualize the the Sense Glove frames. 

In order to build the frame visualizer, enable `ENABLE_FrameVisualizer` option in Wearables.
 
- You can run the following wearable frame visualizer module:
```
IWearFrameVisualizerModule --from HapticGloveFramesVisualizationConfig.ini
```

**N.B. when `SenseCom.x86_64` runs the glove colors in GUI should be blue. If it is not, try to do:**
``` 
sudo adduser $USER dialout
```

**N.B. In order to add `SenseCom.x86_64` executable to the list of application, follow the instructions in `config/SenseGlove.desktop`**
