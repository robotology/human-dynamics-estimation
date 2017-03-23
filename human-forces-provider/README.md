## YARP human-forces-provider module.

Module for reading the forces coming from the force plates and the robot and converting them to human frames.

The module is structured as follow:

<img src="/misc/human-forces-provider.png">

where:
- **ForceReader** is the interface in charge of reading forces coming into the module.
  - **FTForceReader**: generic class for reading forces coming from devices. This is exactly the case of the force plates.
  - **PortForceReader**: generic class for reading forces from a port (case of the robot).
  
- **FrameTransformer** is the interface that transforms the frames of the incoming forces into human frames.
  - **GenericFrameTransformer**: (when required) transforms forces coming from the previous interface into a new reference frame. It is in charge of applying a constant transformation to a given force.
  - **RobotFrameTransformation**: (when required) for the specific case in which the transformation is not constant and needs to be updated at each timestamp.
