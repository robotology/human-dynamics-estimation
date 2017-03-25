/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "ReadOnlyRemoteControlBoardRemapper.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <cassert>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace std;


ReadOnlyRemoteControlBoardRemapper::ReadOnlyRemoteControlBoardRemapper()
{
}

ReadOnlyRemoteControlBoardRemapper::~ReadOnlyRemoteControlBoardRemapper()
{
}

void ReadOnlyRemoteControlBoardRemapper::closeAllRemoteControlBoards()
{
    for(size_t ctrlBrd=0; ctrlBrd < m_remoteControlBoardDevices.size(); ctrlBrd++)
    {
        if( m_remoteControlBoardDevices[ctrlBrd] )
        {
            m_remoteControlBoardDevices[ctrlBrd]->close();
            delete m_remoteControlBoardDevices[ctrlBrd];
            m_remoteControlBoardDevices[ctrlBrd] = 0;
        }
    }

    m_remoteControlBoardDevices.resize(0);
}


bool ReadOnlyRemoteControlBoardRemapper::close()
{
    bool ret = true;

    bool ok = ReadOnlyControlBoardRemapper::detachAll();

    ret = ret && ok;

    ok = ReadOnlyControlBoardRemapper::close();

    ret = ret && ok;

    closeAllRemoteControlBoards();

    return ret;
}

bool ReadOnlyRemoteControlBoardRemapper::open(Searchable& config)
{
    Property prop;
    prop.fromString(config.toString().c_str());

    std::string localPortPrefix;
    std::vector<std::string> remoteControlBoardsPorts;

    // Check if the required parameters  are found
    if( prop.check("localPortPrefix") && prop.find("localPortPrefix").isString() )
    {
        localPortPrefix = prop.find("localPortPrefix").asString().c_str();
    }
    else
    {
        yError() <<"ReadOnlyRemoteControlBoardRemapper: Error parsing parameters: \"localPortPrefix\" should be a string.";
        return false;
    }

    Bottle *remoteControlBoards=prop.find("remoteControlBoards").asList();
    if(remoteControlBoards==0)
    {
        yError() <<"ReadOnlyRemoteControlBoardRemapper: Error parsing parameters: \"remoteControlBoards\" should be followed by a list.";
        return false;
    }

    remoteControlBoardsPorts.resize(remoteControlBoards->size());
    for(int ax=0; ax < remoteControlBoards->size(); ax++)
    {
        remoteControlBoardsPorts[ax] = remoteControlBoards->get(ax).asString().c_str();
    }

    // Load the REMOTE_CONTROLBOARD_OPTIONS, containg any additional option to pass to the remote control boards
    Property remoteControlBoardsOptions;

    Bottle & optionsGroupBot = prop.findGroup("REMOTE_CONTROLBOARD_OPTIONS");
    if (!optionsGroupBot.isNull())
    {
        remoteControlBoardsOptions.fromString(optionsGroupBot.toString());
    }

    // Parameters loaded, open all the remote controlboards

    m_remoteControlBoardDevices.resize(remoteControlBoardsPorts.size(),0);

    PolyDriverList remoteControlBoardsList;

    for(size_t ctrlBrd=0; ctrlBrd < remoteControlBoardsPorts.size(); ctrlBrd++ )
    {
        std::string remote = remoteControlBoardsPorts[ctrlBrd];
        // Note: as local parameter we use localPortPrefix+remoteOfTheReportControlBoard
        std::string local = localPortPrefix+remote;

        Property options = remoteControlBoardsOptions;
        options.put("device", "readonlyremotecontrolboard");
        options.put("local", local);
        options.put("remote", remote);

        //Look also for options for specific control boards
        Bottle & customGroup = prop.findGroup("REMOTE_CONTROLBOARD_OPTIONS_" + remote);
        if (!customGroup.isNull()) {
            options.fromString(customGroup.toString(), false);
        }

        m_remoteControlBoardDevices[ctrlBrd] = new PolyDriver();

        bool ok = m_remoteControlBoardDevices[ctrlBrd]->open(options);

        if( !ok || !(m_remoteControlBoardDevices[ctrlBrd]->isValid()) )
        {
            yError() << "ReadOnlyRemoteControlBoardRemapper: error opening remote_controlboard with remote \"" << remote << "\", opening the device failed.";
            closeAllRemoteControlBoards();
            return false;
        }

        // We use the remote name of the remote_controlboard as the key for it, in absense of anything better
        remoteControlBoardsList.push((m_remoteControlBoardDevices[ctrlBrd]),remote.c_str());
    }

    // Device opened, now we open the ControlBoardRemapper and then we call attachAll
    bool ok = ReadOnlyControlBoardRemapper::open(prop);

    if( !ok )
    {
        yError() << "ReadOnlyRemoteControlBoardRemapper: error opening the controlboardremapper device, opening the device failed.";
        ReadOnlyControlBoardRemapper::close();
        closeAllRemoteControlBoards();
        return false;
    }

    // If open went ok, we now call attachAll
    ok = ReadOnlyControlBoardRemapper::attachAll(remoteControlBoardsList);

    if( !ok )
    {
        yError() << "ReadOnlyRemoteControlBoardRemapper: error calling attachAll in the controlboardremapper device, opening the device failed.";
        ReadOnlyControlBoardRemapper::close();
        closeAllRemoteControlBoards();
        return false;
    }

    // All went ok, return true
    // TODO: close devices that are not actually used by the remapper
    return true;
}

