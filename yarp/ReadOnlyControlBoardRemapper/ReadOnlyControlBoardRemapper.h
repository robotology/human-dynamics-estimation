/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Lorenzo Natale, Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef YARP_DEV_CONTROLBOARDREMAPPER_CONTROLBOARDREMAPPER_H
#define YARP_DEV_CONTROLBOARDREMAPPER_CONTROLBOARDREMAPPER_H

#include "ControlBoardRemapperHelpers.h"

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/Wrapper.h>

#include <string>

#include <vector>


#ifdef MSVC
    #pragma warning(disable:4355)
#endif

namespace yarp {
namespace os {
    class Searchable;
    class Property;
    class Stamp;
}

namespace dev {
    class PolyDriverList;

    class ReadOnlyControlBoardRemapper;
}
}


/**
 *  @ingroup dev_impl_wrapper
 *
 * \section ControlBoardRemapper
 * A device that takes a list of axes from multiple controlboards and
 *  expose them as a single controlboard.
 *
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | axesNames     |      -         | vector of strings  | -      |   -           | Yes     | Ordered list of the axes that are part of the remapped device. |  |
 *
 * The axes are then mapped to the wrapped controlboard in the attachAll method, using the
 * values returned by the getAxisName method of the controlboard. If different axes
 * in two attached controlboard have the same name, the behaviour of this device is undefined.
 *
 * Configuration file using .ini format.
 *
 * \code{.unparsed}
 *  device controlboardremapper
 *  axesNames (joint1 joint2 joint3)
 *
 * ...
 * \endcode
 *
 * For compatibility with the controlboardwrapper2, the
 * networks keyword can also be used to select the desired joints.
 * For more information on the syntax of the networks, see the
 * yarp::dev::ControlBoardWrapper class.
 *
 * \code{.unparsed}
 *  networks (net_larm net_lhand)
 *  joints 16
 *  net_larm    0 3  0 3
 *  net_lhand   4 6  0 2
 * \endcode
 *
 */

class yarp::dev::ReadOnlyControlBoardRemapper
: public yarp::dev::DeviceDriver
, public yarp::dev::IEncodersTimed
, public yarp::dev::IPreciselyTimed
, public yarp::dev::IAxisInfo
, public yarp::dev::IMultipleWrapper
{
private:
    std::vector<std::string> axesNames;
    yarp::dev::RemappedControlBoards remappedControlBoards;

    /** number of axes controlled by this controlboard */
    int controlledJoints;

    /** Verbosity of the class */
    bool _verb;

    // to open ports and print more detailed debug messages
    std::string partName;

    // Buffer data used to simplify implementation of multi joint methods
    yarp::dev::ControlBoardRemapperBuffers buffers;

    // Buffer data used for full controlboard methods
    yarp::dev::ControlBoardSubControlBoardAxesDecomposition allJointsBuffers;

    // Buffer data for multiple arbitary joint methods
    yarp::dev::ControlBoardArbitraryAxesDecomposition selectedJointsBuffers;

    /**
     * Set the number of controlled axes, resizing appropriatly
     * all the necessary buffers.
     */
    void setNrOfControlledAxes(const size_t nrOfControlledAxes);

    /**
     * If the class was configured using the networks format,
     * call this method to update the vector containing the
     * axesName .
     */
    bool updateAxesName();

    /**
     * Configure buffers used by the device
     */
    void configureBuffers();

    // Parse device options
    bool parseOptions(yarp::os::Property &prop);

    bool usingAxesNamesForAttachAll;
    bool usingNetworksForAttachAll;

    /***
     * Parse device options if networks option is passed
     *
     * This will fill the axesNames and controlledJoints attributes, while it
     * leaves empty the remappedDevices attribute that will be then
     * filled only at the attachAll method.
     */
    bool parseAxesNames(const yarp::os::Property &prop);

    /***
     * Parse device options if networks option is passed
     *
     * This will fill the remappedDevices and controlledJoints attributes, while it
     * leaves empty the axesNames attribute that will be then
     * filled only at the attachAll method.
     */
    bool parseNetworks(const yarp::os::Property &prop);

    /**
     * attachAll if the networks option is used for configuration.
     */
    bool attachAllUsingNetworks(const yarp::dev::PolyDriverList &l);

    /**
     * attachAll if the axesNames option is used for configuration.
     */
    bool attachAllUsingAxesNames(const yarp::dev::PolyDriverList &l);



public:
    /**
    * Constructor.
    */
    ReadOnlyControlBoardRemapper();

    virtual ~ReadOnlyControlBoardRemapper();

    /**
    * Return the value of the verbose flag.
    * @return the verbose flag.
    */
    bool verbose() const { return _verb; }

    /**
    * Default open() method.
    * @return always false since initialization requires parameters.
    */
    virtual bool open() { return false; }

    /**
    * Close the device driver by deallocating all resources and closing ports.
    * @return true if successful or false otherwise.
    */
    virtual bool close();


    /**
    * Open the device driver.
    * @param prop is a Searchable object which contains the parameters.
    * Allowed parameters are described in the class documentation.
    */
    virtual bool open(yarp::os::Searchable &prop);

    virtual bool detachAll();

    virtual bool attachAll(const yarp::dev::PolyDriverList &l);


    /* IEncoders */

    virtual bool getAxes(int *ax);

    virtual bool resetEncoder(int j);

    virtual bool resetEncoders();

    virtual bool setEncoder(int j, double val);

    virtual bool setEncoders(const double *vals);

    virtual bool getEncoder(int j, double *v);

    virtual bool getEncoders(double *encs);

    virtual bool getEncodersTimed(double *encs, double *t);

    virtual bool getEncoderTimed(int j, double *v, double *t);

    virtual bool getEncoderSpeed(int j, double *sp);

    virtual bool getEncoderSpeeds(double *spds);

    virtual bool getEncoderAcceleration(int j, double *acc);

    virtual bool getEncoderAccelerations(double *accs);

    /* IPreciselyTimed */

    virtual yarp::os::Stamp getLastInputStamp();

    /* IAxisInfo */
    virtual bool getAxisName(int j, yarp::os::ConstString &name);

    virtual bool getJointType(int j, yarp::dev::JointTypeEnum &type);


};


#endif // YARP_DEV_CONTROLBOARDREMAPPER_CONTROLBOARDREMAPPER_H
