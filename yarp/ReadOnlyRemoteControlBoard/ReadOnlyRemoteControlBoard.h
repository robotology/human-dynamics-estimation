/*
* Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
* Author: Francesco Romano
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef YARP_DEV_READONLYREMOTECONTROLBOARD_READONLYREMOTECONTROLBOARD_H
#define YARP_DEV_READONLYREMOTECONTROLBOARD_READONLYREMOTECONTROLBOARD_H

#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <yarp/os/Semaphore.h>
#include "stateExtendedReader.hpp"

#include <vector>

namespace yarp {
        namespace dev {
            class ReadOnlyRemoteControlBoard;
            
        }
}

class yarp::dev::ReadOnlyRemoteControlBoard
: public yarp::dev::IEncodersTimed
, public yarp::dev::DeviceDriver
, public yarp::dev::IAxisInfo
{

#ifndef DOXYGEN_SHOULD_SKIP_THIS

    // Buffer associated to the extendedOutputStatePort port; in this case we will use the type generated
    // from the YARP .thrift file
    StateExtendedInputPort m_extendedIntputStatePort;  // Buffered port storing new data
    yarp::os::Semaphore m_extendedPortMutex;

    std::vector<std::pair<yarp::os::ConstString, yarp::dev::JointTypeEnum> > m_axes;

    mutable Stamp m_lastStamp;  //this is shared among all calls that read encoders
    // Semaphore mutex;
    int m_numberOfJoints;

#endif /*DOXYGEN_SHOULD_SKIP_THIS*/

public:
    /**
     * Constructor.
     */
    ReadOnlyRemoteControlBoard();

    /**
     * Destructor.
     */
    virtual ~ReadOnlyRemoteControlBoard();

    virtual bool open();

    virtual bool open(yarp::os::Searchable& config);

    virtual bool close();

    virtual bool getAxes(int *ax);
    virtual bool resetEncoder(int j);
    virtual bool resetEncoders();
    virtual bool setEncoder(int j, double val);
    virtual bool setEncoders(const double *vals);
    virtual bool getEncoder(int j, double *v);
    virtual bool getEncoderTimed(int j, double *v, double *t);
    virtual bool getEncoders(double *encs);
    virtual bool getEncodersTimed(double *encs, double *ts);
    virtual bool getEncoderSpeed(int j, double *sp);
    virtual bool getEncoderSpeeds(double *spds);
    virtual bool getEncoderAcceleration(int j, double *acc);
    virtual bool getEncoderAccelerations(double *accs);

    /* IAxisInfo */
    virtual bool getAxisName(int j, yarp::os::ConstString &name);

    virtual bool getJointType(int j, yarp::dev::JointTypeEnum &type);


};

#if defined(_MSC_VER) && !defined(YARP_NO_DEPRECATED) // since YARP 2.3.65
YARP_WARNING_POP
#endif


#endif // YARP_DEV_READONLYREMOTECONTROLBOARD_READONLYREMOTECONTROLBOARD_H
