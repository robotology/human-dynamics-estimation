## hde-interface module

This module contains a `HDEControlBoardDriver` storing the values of joint positions, joint velocities, joint accelerations and joint torques
with the data coming from the output ports of `human-state-provider` and `human-dynamics-estimation` modules.

The module is structured as follows



The `HDEControlBoardDriver` is wrapped with `ControlBoardWrapper2` and implements the following interfaces
*  `IEncoders`- giving access to joint positions, velocities and accelerations
* `ITorqueControl`- giving access to joint torques

