# How to run Xsens suit as Wearable Device

This page describes how to run the [Xsens suit](https://www.xsens.com/motion-capture) as a wearable device source. Further information on how to install can be found at https://github.com/robotology/human-dynamics-estimation/wiki/Set-up-Machine-for-running-HDE#xsens-only-for-windows, while for more information on the Xsens suit device tutorials are available at https://tutorial.xsens.com/.

The configuration file to run the XsensSuit as wearable device is [`XsensSuitWearableDevice.xml`](https://github.com/robotology/wearables/blob/master/app/xml/XsensSuitWearableDevice.xml), and it can be launched with:
```
yarprobotinterface --config XsensSuitWearableDevice.xml
```
Once the device is started, it will start searching for the suit until it finds all the sensor. You can check the output of the device to follow the status of the search. It may be required to move the sensor in order to let them be discovered.

**Before running**
Before running the device make sure that:
- [`yarpserver`](https://www.yarp.it/yarpserver.html) is running
- The router/receiver of the suit is connected to the laptop, the suit is powered on, and the pendrive with the licence is instered on the laptop.
- There is not another device running that is using the port names. In case multiple suits have to run simultaneously, it is necessary to differentiate the name of the ports.
- In the [configuration file](https://github.com/robotology/wearables/blob/master/app/xml/XsensSuitWearableDevice.xml), the following parameters are set propery:
  - `xsens-rundeps-dir`: Folder where XsensMVN runtime dependencies are stored.
  - `default-calibration-type`: Calibration type to be used (available values: `NposeWalk`, `TposeWalk`, `Npose`, `Tpose`).
  - `minimum-calibration-quality-required`: Minimum calibration quality to be considered good enought to be applied and allow the acquisition to start (available values: `Poor`, `Acceptable`, `Good`).
  - `body-dimensions`: Subject specific body dimensions.
- The subject is wearing the suit correctly 

**Calibration**
In order to acquire data from the Xsens suit it is required to perform a calibration procedure, the type of procedure is set through the configuration file (`default-calibration-type`).
The calibration is started sending the following command:
```
yarp rpc /XsensSuit/Control/rpc:i
calibrate
```
The device will give as output the indication for performing the calibration, and the final quality of the calibration.

In case the calibration does not meet the minimum quality requirement, new calibration can be performed with the same command. In case a new calibration is required after the acquisition is started, it is necessary to stop the acquisition befor performing a new calibration:
```
yarp rpc /XsensSuit/Control/rpc:i
stopAcquisition
calibrate
```

**Data**
Once the calibration procedure is completed with the required minimum quality level, the data acquisition is started sending the following command:
```
yarp rpc /XsensSuit/Control/rpc:i
startAcquisition
```
If the device is running correctly, the stream of wearable data can be read with:
```
yarp read ... /XsensSuit/WearableData/data:o
```