# How to run FTShoes as Wearable Device

This page describes how to run the [FTShoes](https://github.com/robotology/forcetorque-yarp-devices/tree/master/ftShoe) as a wearable device source. Further information on how to install and use the shoes can be found at https://github.com/robotology/forcetorque-yarp-devices/tree/master/ftShoe.

The configuration file to run the FTShoes as wearable device is [`FTShoesWearableDevice.xml`](https://github.com/robotology/wearables/blob/master/app/xml/FTShoesWearableDevice.xml), and it can be launched with:
```
yarprobotinterface --config FTShoesWearableDevice.xml
```
The two shoes can also be launched separately using [`FTShoeLeftWearableDevice.xml`](https://github.com/robotology/wearables/blob/master/app/xml/FTShoeLeftWearableDevice.xml) and [`FTShoeRightWearableDevice.xml`](https://github.com/robotology/wearables/blob/master/app/xml/FTShoeRightWearableDevice.xml).

### Before running
Before running the device make sure that:
- [`yarpserver`](https://www.yarp.it/yarpserver.html) is running
- The shoes are connected and recognized by the laptop
- The can address of the FT sensors in the shoes is the same of the configuration file (e.g. `ftShoe_Left_Front` -> `<param name="canAddress"> 0x01 </param>`)
- There is not another device running that is using the port names. In case multiple pairs of shoes have to run simultaneously, it is necessary to differentiate the name of the ports like in [`FTShoesWearableDevice_2.xml`](https://github.com/robotology/wearables/blob/master/app/xml/FTShoesWearableDevice_2.xml).
- The `inSituMatrices` contained in the configuration file correspond to the shoes you are using.


#### Calibration
The FT data streamed by the shoes may be characterized by an offset. The procedure for removing the offset is described at https://github.com/robotology/forcetorque-yarp-devices/tree/master/ftShoe#how-to-use-the-ftshoes.

#### Data
If the device is running correctly, the stream of wearable data can be read with:
```
yarp read ... /FTShoeLeft/WearableData/data:o
yarp read ... /FTShoeRight/WearableData/data:o
```
