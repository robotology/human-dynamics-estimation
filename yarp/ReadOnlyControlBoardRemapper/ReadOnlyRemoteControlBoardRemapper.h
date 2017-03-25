/*
* Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
* Author: Lorenzo Natale, Silvio Traversaro, Francesco Romano
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef YARP_DEV_READONLYCONTROLBOARDREMAPPER_READONLYREMOTECONTROLBOARDREMAPPER_H
#define YARP_DEV_READONLYCONTROLBOARDREMAPPER_READONLYREMOTECONTROLBOARDREMAPPER_H

#include <yarp/dev/PolyDriver.h>

#include "ReadOnlyControlBoardRemapper.h"


namespace yarp {
namespace dev {
    class ReadOnlyRemoteControlBoardRemapper;
}
}

/**
 *  @ingroup dev_impl_wrapper
 *
 * @section ReadOnlyRemoteControlBoardRemapper
 * A device that takes a list of axes from multiple controlboards, a list
 * of remote controlboards in which this axes are located, that is opening
 * all the remote controlboards but is exposing them
 *
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required     | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:-----------: |:-----------------------------------------------------------------:|:-----:|
 * | axesNames      |      -         | vector of strings  | -   |   -           | Yes          | Ordered list of the axes that are part of the remapped device.    |       |
 * | remoteControlBoards |     -     | vector of strings  | -   |   -           | Yes          | List of remote prefix used by the readonly remote controlboards.           | The element of this list are then passed as "remote" parameter to the ReadOnlyRemoteControlBoard device. |
 * | localPortPrefix |     -         | string             | -   |   -           | Yes          | All ports opened by this device will start with this prefix       |       |
 * | REMOTE_CONTROLBOARD_OPTIONS | - | group              | -   |   -           | No           | Options that will be passed directly to the readonlyremotecontrolboard devices | |
 * | REMOTE_CONTROLBOARD_OPTIONS_{CONTROLBOARD_NAME} | - | group              | -   |   -           | No           | Options that will be passed directly only to the readonlyremotecontrolboard device identified by {CONTROLBOARD_NAME} | |
 * All the passed readonly remote controlboards are opened, and then the axesNames and the opened device are
 * passed to the ReadOnlyControlBoardRemapper device. If different axes
 * in two attached controlboard have the same name, the behaviour of this device is undefined.
 *
 *
 * Configuration file using .ini format.
 *
 * @code{.unparsed}
 *  device readonlyremotecontrolboardremapper
 *  axesNames (torso_pitch torso_roll torso_yaw neck_pitch neck_roll neck_yaw)
 *  remoteControlBoards (/icub/torso /icub/head)
 *
 *  [REMOTE_CONTROLBOARD_OPTIONS]
 *  writeStrict on
 *
 *  [REMOTE_CONTROLBOARD_OPTIONS_/icub/torso]
 *  axesDescription ((torso_yaw, atrv) (torso_roll, atrv) (torso_pitch, atrv))
 * ...
 * @endcode
 *
 * Configuration of the device from C++ code.
 * @code{.cpp}
 *   Property options;
 *   options.put("device","readonlyremotecontrolboardremapper");
 *   Bottle axesNames;
 *   Bottle & axesList = axesNames.addList();
 *   axesList.addString("torso_pitch");
 *   axesList.addString("torso_roll");
 *   axesList.addString("torso_yaw");
 *   axesList.addString("neck_pitch");
 *   axesList.addString("neck_roll");
 *   axesList.addString("neck_yaw");
 *   options.put("axesNames",axesNames.get(0))
 *
 *   Bottle remoteControlBoards;
 *   Bottle & remoteControlBoardsList = remoteControlBoards.addList();
 *   remoteControlBoardsList.addString("/icub/torso");
 *   remoteControlBoardsList.addString("/icub/head");
 *   options.put("remoteControlBoards",remoteControlBoards.get(0));
 *
 *   options.put("localPortPrefix",/test");
 *
 *   Property & remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
 *   remoteControlBoardsOpts.put("writeStrict","on");
 *
 *   Property & torsoRemoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS_/icub/torso");
 *   // ...
 *
 *   // Actually open the device
 *   PolyDriver robotDevice(options);
 *
 *   // Use it as  you would use any controlboard device
 *   // ...
 * @endcode
 *
 *
 *
 * @section Caveat
 * @see RemoteControlBoardRemapper as this devices inherit most of the behaviours but
 * will open ReadOnlyRemoteControlBoards instead of RemoteControlBoards
 */

class yarp::dev::ReadOnlyRemoteControlBoardRemapper : public yarp::dev::ReadOnlyControlBoardRemapper
{
private:
    /**
     * List of remote_controlboard devices opened by the RemoteControlBoardRemapper device.
     */
    std::vector<PolyDriver*> m_remoteControlBoardDevices;


    // Close all opened remote controlboards
    void closeAllRemoteControlBoards();

public:
    /**
    * Constructor.
    */
    ReadOnlyRemoteControlBoardRemapper();

    virtual ~ReadOnlyRemoteControlBoardRemapper();

    /**
     * Default open() method.
     * @return always false since initialization requires parameters.
     */
    virtual bool open() { return false; }

   /**
     * Open the device driver.
     * @param prop is a Searchable object which contains the parameters.
     * Allowed parameters are described in the class documentation.
     */
    virtual bool open(yarp::os::Searchable &prop);

    /**
     * Close the device driver by deallocating all resources and closing ports.
     * @return true if successful or false otherwise.
     */
    virtual bool close();
};

#endif
