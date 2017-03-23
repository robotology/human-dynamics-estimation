human-state-provider
==========

This module is responsible of reading segments frame information from an `yarp::dev::IFrameProvider` object.
This object provides a list of segments frame information, and for each segment frame, its pose with respect to a global frame
and its velocity.

The URDF model specified by the `--urdf_model` parameter describes the kinematic information of the 
mechanical system of which this module will provide the state information.
Note that the results will depend on the complexity of the model (e.g. few degrees of freedom model may lead to worst approximations) 
but the formulation is agnostic of the model complexity.

This module will then build a list of adjacent segments which can be found both in the `yarp::dev::IFrameProvider` segment list and in the URDF model
(note that the term *segment* is the *human* correspondence of the *link*). For each of these pairs, this module will provide 
an estimation of the configuration and velocity of the joints connecting the two element in the pair.



### WARNING

As **MUMPS** makes use of static variables not only it is not thread-safe, but it is
not even possible to use different instances of IPOpt confined in different threads.
As a result if you need to use ***MUMPS*** remember to specify as option to the module 
the `--ikpool 1` which will use only one thread to launch all the inverse kinematics routines.

It is of course advisable to use a different solver, see `--ik_linear_solver` option.
