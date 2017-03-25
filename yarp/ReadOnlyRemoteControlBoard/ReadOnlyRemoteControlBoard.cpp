/*
* Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
* Author: Francesco Romano
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include "ReadOnlyRemoteControlBoard.h"

#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>

#include <yarp/sig/Vector.h>


#include <algorithm>
#include <cassert>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#ifndef DOXYGEN_SHOULD_SKIP_THIS

const double TIMEOUT=0.5;

#endif /*DOXYGEN_SHOULD_SKIP_THIS*/


#if defined(_MSC_VER) && !defined(YARP_NO_DEPRECATED) // since YARP 2.3.65
// A class implementing setXxxxxMode() causes a warning on MSVC
YARP_WARNING_PUSH
YARP_DISABLE_DEPRECATED_WARNING
#endif

/**
* @ingroup dev_impl_wrapper
*
* The client side of the control board, connects to a remote controlboard using the YARP network.
*
* This device communicates using the YARP ports opened the yarp::dev::ControlBoardWrapper device
* to use a device exposing controlboard method even from a different process (or even computer)
* from the one that opened the controlboard device.
*
*  Parameters required by this device are:
* | Parameter name | SubParameter   | Type    | Units | Default Value | Required     | Description                       | Notes |
* |:--------------:|:--------------:|:-------:|:-----:|:-------------:|:-----------: |:---------------------------------:|:-----:|
* | remote         |       -        | string  | -     |   -           | Yes          | Prefix of the port to which to connect.  |       |
* | local          |       -        | string  | -     |   -           | Yes          | Port prefix of the port openend by this device.  |       |
* | writeStrict    |       -        | string  | -     | See note      | No           |                                   |       |
*
*/

namespace yarp {
namespace dev {

    ReadOnlyRemoteControlBoard::ReadOnlyRemoteControlBoard()
    : m_numberOfJoints(0) {}

    /**
     * Destructor.
     */
    ReadOnlyRemoteControlBoard::~ReadOnlyRemoteControlBoard() {}


    /**
     * Default open.
     * @return always true.
     */
    bool ReadOnlyRemoteControlBoard::open() {
        return true;
    }

    bool ReadOnlyRemoteControlBoard::open(Searchable& config) {
        ConstString remote = config.find("remote").asString();
        ConstString local = config.find("local").asString();

        if (local.empty()) {
            yError("Problem connecting to remote controlboard, 'local' port prefix not given");
            return false;
        }

        if (remote.empty()) {
            yError("Problem connecting to remote controlboard, 'remote' port name not given");
            return false;
        }

        ConstString carrier = config.check("carrier", Value("udp"), "default carrier for streaming robot state").asString();

        bool portProblem = false;
        if (!m_extendedIntputStatePort.open(local + "/stateExt:i")) {
            portProblem = true;
        }

        if (!portProblem) {
            m_extendedIntputStatePort.useCallback();
        }


        bool connectionProblem = false;
        if (!portProblem) {
            bool ok = false;
            ok = Network::connect(remote + "/stateExt:o", m_extendedIntputStatePort.getName(), carrier);
            if (!ok) {
                connectionProblem = true;
                yError("*** Extended port %s was not found on the controlBoardWrapper I'm connecting to.", (remote + "/stateExt:o").c_str());
            }
        }

        if (connectionProblem || portProblem) {
            m_extendedIntputStatePort.close();
            return false;
        }

        Value &axesDescription = config.find("axesDescription");
        if (axesDescription.isNull() || !axesDescription.isList()) {
            yError("*** Option 'axesDescription' not found or malformed.");
            m_extendedIntputStatePort.close();
            return false;
        }

        Bottle *axesDescriptionList = axesDescription.asList();
        m_numberOfJoints = axesDescriptionList->size();
        m_extendedIntputStatePort.init(m_numberOfJoints);
        m_axes.reserve(m_numberOfJoints);

        for (int index = 0; index < m_numberOfJoints; ++index) {
            const Value& axis = axesDescriptionList->get(index);
            if (axis.isNull() || !axis.isList() || axis.asList()->size() != 2) {
                yError("*** Option 'axesDescription' malformed at index %d.", index);
                m_extendedIntputStatePort.close();
                return false;
            }
            ConstString axisName = axis.asList()->get(0).asString();
            JointTypeEnum axisVocab = static_cast<JointTypeEnum>(axis.asList()->get(1).asVocab());
            m_axes.push_back(std::pair<yarp::os::ConstString, yarp::dev::JointTypeEnum>(axisName, axisVocab));
        }
        
        return true;
    }
    
    /**
     * Close the device driver and stop the port connections.
     * @return true/false on success/failure.
     */
     bool ReadOnlyRemoteControlBoard::close() {
        m_extendedIntputStatePort.close();
        return true;
    }

    /* IEncoder */

    bool ReadOnlyRemoteControlBoard::getAxes(int *ax)
    {
        if (!ax) return false;
        *ax = m_numberOfJoints;
        return true;
    }

    /**
     * Reset encoder, single joint. Set the encoder value to zero
     * @param j is the axis number
     * @return true/false on success/failure
     */
     bool ReadOnlyRemoteControlBoard::resetEncoder(int j) {
        return false;
    }

    /**
     * Reset encoders. Set the encoders value to zero
     * @return true/false
     */
     bool ReadOnlyRemoteControlBoard::resetEncoders() {
        return false;
    }

    /**
     * Set the value of the encoder for a given joint.
     * @param j encoder number
     * @param val new value
     * @return true/false on success/failure
     */
     bool ReadOnlyRemoteControlBoard::setEncoder(int j, double val) {
        return false;
    }

    /**
     * Set the value of all encoders.
     * @param vals pointer to the new values
     * @return true/false
     */
     bool ReadOnlyRemoteControlBoard::setEncoders(const double *vals) {
        return false;
    }

    /**
     * Read the value of an encoder.
     * @param j encoder number
     * @param v pointer to storage for the return value
     * @return true/false, upon success/failure
     */
     bool ReadOnlyRemoteControlBoard::getEncoder(int j, double *v)
    {
        if (j < 0 || j >= m_numberOfJoints || !v) return false;

        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastSingle(j, VOCAB_ENCODER, v, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    /**
     * Read the value of an encoder.
     * @param j encoder number
     * @param v pointer to storage for the return value
     * @return true/false, upon success/failure
     */
     bool ReadOnlyRemoteControlBoard::getEncoderTimed(int j, double *v, double *t)
    {
        if (j < 0 || j >= m_numberOfJoints || !v || !t) return false;

        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastSingle(j, VOCAB_ENCODER, v, m_lastStamp, localArrivalTime);
        *t = m_lastStamp.getTime();
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    /**
     * Read the position of all axes. This object receives encoders periodically
     * from a YARP port. You should check the return value of the function to
     * make sure that encoders have been received at least once and with the expected
     * rate.
     * @param encs pointer to the array that will contain the output
     * @return true/false on success/failure. Failure means encoders have not been received
     * from the server or that they are not being streamed with the expected rate.
     */
     bool ReadOnlyRemoteControlBoard::getEncoders(double *encs) {
        if (!encs) return false;

        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastVector(VOCAB_ENCODERS, encs, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    /**
     * Read the position of all axes.
     * @param encs pointer to the array that will contain the output
     * @param ts pointer to the array that will contain timestamps
     * @return true/false on success/failure
     */
     bool ReadOnlyRemoteControlBoard::getEncodersTimed(double *encs, double *ts) {
        if (!encs || !ts) return false;

        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastVector(VOCAB_ENCODERS, encs, m_lastStamp, localArrivalTime);
        std::fill_n(ts, m_numberOfJoints, m_lastStamp.getTime());
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }
    /**
     * Read the istantaneous speed of an axis.
     * @param j axis number
     * @param sp pointer to storage for the output
     * @return true if successful, false otherwise.
     */
     bool ReadOnlyRemoteControlBoard::getEncoderSpeed(int j, double *sp)
    {
        if (j < 0 || j >= m_numberOfJoints || !sp) return false;

        double localArrivalTime=0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastSingle(j, VOCAB_ENCODER_SPEED, sp, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }


    /**
     * Read the instantaneous speed of all axes.
     * @param spds pointer to storage for the output values
     * @return true/false on success or failure
     */
     bool ReadOnlyRemoteControlBoard::getEncoderSpeeds(double *spds)
    {
        if (!spds) return false;
        double localArrivalTime=0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastVector(VOCAB_ENCODER_SPEEDS, spds, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    /**
     * Read the instantaneous acceleration of an axis.
     * @param j axis number
     * @param acc pointer to the array that will contain the output
     */

     bool ReadOnlyRemoteControlBoard::getEncoderAcceleration(int j, double *acc)
    {
        if (j < 0 || j >= m_numberOfJoints || !acc) return false;

        double localArrivalTime = 0.0;
        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastSingle(j, VOCAB_ENCODER_ACCELERATION, acc, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    /**
     * Read the istantaneous acceleration of all axes.
     * @param accs pointer to the array that will contain the output
     * @return true if all goes well, false if anything bad happens.
     */
     bool ReadOnlyRemoteControlBoard::getEncoderAccelerations(double *accs)
    {
        if (!accs) return false;

        double localArrivalTime = 0.0;
        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastVector(VOCAB_ENCODER_ACCELERATIONS, accs, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    bool ReadOnlyRemoteControlBoard::getAxisName(int j, yarp::os::ConstString& name) {
        if (j < 0 || j >= m_numberOfJoints) return false;
        name = m_axes[j].first;
        return true;
    }

    bool ReadOnlyRemoteControlBoard::getJointType(int j, yarp::dev::JointTypeEnum& type) {
        if (j < 0 || j >= m_numberOfJoints) return false;
        type = m_axes[j].second;
        return true;
    }

}
}
