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
#include <yarp/dev/ITorqueControl.h>
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
 * @section ReadOnlyControlBoardRemapper
 * 
 * @see ControlBoardRemapper
 * This device exposes only a limited subset of interfaces with respect 
 * to ControlBoardRemapper, but it fully inherit all the other behaviours.
 *
 */

class yarp::dev::ReadOnlyControlBoardRemapper
: public yarp::dev::DeviceDriver
, public yarp::dev::IEncodersTimed
, public yarp::dev::IPreciselyTimed
, public yarp::dev::ITorqueControl
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
    
    
    /* ITorqueControl */
    
    virtual bool getRefTorques(double *t);
     
    virtual bool getRefTorque(int j, double *t);

    virtual bool setRefTorques(const double *t);

    virtual bool setRefTorque(int j, double t);
    
    virtual bool getBemfParam(int j, double *bemf);

    virtual bool setBemfParam(int j, double bemf);
    
    virtual bool setTorquePid(int j, const Pid &pid);

    virtual bool getTorque(int j, double *t);

    virtual bool getTorques(double *t);

    virtual bool getTorqueRange(int j, double *min, double *max);

    virtual bool getTorqueRanges(double *min, double *max);

    virtual bool setTorquePids(const Pid *pids);

    virtual bool setTorqueErrorLimit(int j, double limit);

    virtual bool setTorqueErrorLimits(const double *limits);

    virtual bool getTorqueError(int j, double *err);

    virtual bool getTorqueErrors(double *errs);

    virtual bool getTorquePidOutput(int j, double *out);

    virtual bool getTorquePidOutputs(double *outs);

    virtual bool getTorquePid(int j, Pid *pid);

    virtual bool getTorquePids(Pid *pids);

    virtual bool getTorqueErrorLimit(int j, double *limit);

    virtual bool getTorqueErrorLimits(double *limits);

    virtual bool resetTorquePid(int j);

    virtual bool disableTorquePid(int j);

    virtual bool enableTorquePid(int j);

    virtual bool setTorqueOffset(int j, double v);
    
    
        
    virtual bool getAxisName(int j, yarp::os::ConstString &name);

    virtual bool getJointType(int j, yarp::dev::JointTypeEnum &type);


};


#endif // YARP_DEV_CONTROLBOARDREMAPPER_CONTROLBOARDREMAPPER_H
