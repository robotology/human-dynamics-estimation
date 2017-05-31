/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Lorenzo Natale, Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "ReadOnlyControlBoardRemapper.h"
#include "ControlBoardRemapperHelpers.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace yarp {
namespace dev {

ReadOnlyControlBoardRemapper::ReadOnlyControlBoardRemapper()
{
    controlledJoints = 0;
    _verb = false;

    axesNames.clear();
}

ReadOnlyControlBoardRemapper::~ReadOnlyControlBoardRemapper()
{
}

bool ReadOnlyControlBoardRemapper::close()
{
    return detachAll();
}

bool ReadOnlyControlBoardRemapper::open(Searchable& config)
{
    Property prop;
    prop.fromString(config.toString().c_str());

    _verb = (prop.check("verbose","if present, give detailed output"));
    if (_verb)
    {
        yInfo("ReadOnlyControlBoardRemapper: running with verbose output\n");
    }

    if(!parseOptions(prop))
    {
        return false;
    }

    return true;
}

bool ReadOnlyControlBoardRemapper::parseOptions(Property& prop)
{
    bool ok = true;

    usingAxesNamesForAttachAll  = prop.check("axesNames", "list of networks merged by this wrapper");
    usingNetworksForAttachAll = prop.check("networks", "list of networks merged by this wrapper");


    if( usingAxesNamesForAttachAll &&
        usingNetworksForAttachAll )
    {
        yError() << "controlBoardRemapper: Both axesNames and networks option present, this is not supported.\n";
        return false;
    }

    if( !usingAxesNamesForAttachAll &&
        !usingNetworksForAttachAll )
    {
        yError() << "controlBoardRemapper: axesNames option not found";
        return false;
    }

    if( usingAxesNamesForAttachAll )
    {
        ok = parseAxesNames(prop);
    }

    if( usingNetworksForAttachAll )
    {
        ok = parseNetworks(prop);
    }

    return ok;
}

bool ReadOnlyControlBoardRemapper::parseAxesNames(const Property& prop)
{
    Bottle *propAxesNames=prop.find("axesNames").asList();
    if(propAxesNames==0)
    {
       yError() <<"ReadOnlyControlBoardRemapper: Error parsing parameters: \"axesNames\" should be followed by a list\n";
       return false;
    }

    axesNames.resize(propAxesNames->size());
    for(int ax=0; ax < propAxesNames->size(); ax++)
    {
        axesNames[ax] = propAxesNames->get(ax).asString().c_str();
    }

    this->setNrOfControlledAxes(axesNames.size());

    return true;
}

bool ReadOnlyControlBoardRemapper::parseNetworks(const Property& prop)
{
    Bottle *nets=prop.find("networks").asList();
    if(nets==0)
    {
       yError() <<"ReadOnlyControlBoardRemapper: Error parsing parameters: \"networks\" should be followed by a list\n";
       return false;
    }

    if (!prop.check("joints", "number of joints of the part"))
    {
        yError() <<"ReadOnlyControlBoardRemapper: joints options not found when reading networks option";
        return false;
    }

    this->setNrOfControlledAxes((size_t)prop.find("joints").asInt());

    int nsubdevices=nets->size();
    remappedControlBoards.lut.resize(controlledJoints);
    remappedControlBoards.subdevices.resize(nsubdevices);

    // configure the devices
    for(int k=0;k<nets->size();k++)
    {
        Bottle parameters;
        int wBase;
        int wTop;
        int base;
        int top;

        parameters=prop.findGroup(nets->get(k).asString().c_str());

        if(parameters.size()==2)
        {
            Bottle *bot=parameters.get(1).asList();
            Bottle tmpBot;
            if(bot==NULL)
            {
                // probably data are not passed in the correct way, try to read them as a string.
                ConstString bString(parameters.get(1).asString());
                tmpBot.fromString(bString);

                if(tmpBot.size() != 4)
                {
                    yError() << "Error: check network parameters in part description"
                             << "--> I was expecting "<<nets->get(k).asString().c_str() << " followed by a list of four integers in parenthesis"
                             << "Got: "<< parameters.toString().c_str() << "\n";
                    return false;
                }
                else
                {
                    bot = &tmpBot;
                }
            }

            // If I came here, bot is correct
            wBase=bot->get(0).asInt();
            wTop=bot->get(1).asInt();
            base=bot->get(2).asInt();
            top=bot->get(3).asInt();
        }
        else if (parameters.size()==5)
        {
            // yError<<"Parameter networks use deprecated syntax\n";
            wBase=parameters.get(1).asInt();
            wTop=parameters.get(2).asInt();
            base=parameters.get(3).asInt();
            top=parameters.get(4).asInt();
        }
        else
        {
            yError() <<"Error: check network parameters in part description"
                     <<"--> I was expecting "<<nets->get(k).asString().c_str() << " followed by a list of four integers in parenthesis"
                     <<"Got: "<< parameters.toString().c_str() << "\n";
            return false;
        }

        RemappedSubControlBoard *tmpDevice=remappedControlBoards.getSubControlBoard((size_t)k);
        tmpDevice->setVerbose(_verb);

        if( (wTop-wBase) != (top-base) )
        {
            yError() <<"Error: check network parameters in network "<<nets->get(k).asString().c_str() <<
             "I was expecting a well form quadruple of numbers, got instead: "<< parameters.toString().c_str();
        }

        tmpDevice->id = nets->get(k).asString().c_str();

        for(int j=wBase;j<=wTop;j++)
        {
            int off = j-wBase;
            remappedControlBoards.lut[j].subControlBoardIndex=k;
            remappedControlBoards.lut[j].axisIndexInSubControlBoard=base+off;
        }
    }

    return true;
}

void ReadOnlyControlBoardRemapper::setNrOfControlledAxes(const size_t nrOfControlledAxes)
{
    controlledJoints = nrOfControlledAxes;
    buffers.controlBoardModes.resize(nrOfControlledAxes,0);
    buffers.dummyBuffer.resize(nrOfControlledAxes,0.0);
}


bool ReadOnlyControlBoardRemapper::updateAxesName()
{
    bool ret = true;
    axesNames.resize(controlledJoints);

    for(int l=0; l < controlledJoints; l++)
    {
        yarp::os::ConstString axNameYARP;
        bool ok = this->getAxisName(l,axNameYARP);
        if( ok )
        {
            axesNames[l] = axNameYARP.c_str();
        }

        ret = ret && ok;
    }

    return ret;
}

bool ReadOnlyControlBoardRemapper::attachAll(const PolyDriverList &polylist)
{
    // For both cases, now configure everything that need
    // all the attribute to be correctly configured
    bool ok = false;

    if( usingAxesNamesForAttachAll )
    {
        ok = attachAllUsingAxesNames(polylist);
    }

    if( usingNetworksForAttachAll )
    {
        ok = attachAllUsingNetworks(polylist);
    }

    //check if all devices are attached to the driver
    bool ready=true;
    for(unsigned int k=0; k<remappedControlBoards.getNrOfSubControlBoards(); k++)
    {
        if (!remappedControlBoards.subdevices[k].isAttached())
        {
            ready=false;
        }
    }

    if (!ready)
    {
        yError("ReadOnlyControlBoardRemapper: AttachAll failed, some subdevice was not found or its attach failed\n");
        return false;
    }

    if( ok )
    {
        configureBuffers();
    }

    return ok;
}

// First we store a map between each axes name
// in the passed PolyDriverList and the device in which they belong and their index
struct axisLocation
{
    ConstString subDeviceKey;
    size_t indexOfSubDeviceInPolyDriverList;
    int indexInSubDevice;
};


bool ReadOnlyControlBoardRemapper::attachAllUsingAxesNames(const PolyDriverList& polylist)
{
    std::map<std::string, axisLocation> axesLocationMap;

    for(int p=0;p<polylist.size();p++)
    {
        // If there is a device with a specific device key, use it
        // as a calibrator, otherwise rely on the subcontrolboards
        // as usual
        std::string deviceKey=polylist[p]->key.c_str();
        if(deviceKey == "Calibrator" || deviceKey == "calibrator")
        {
            yWarning("Calibrator not supported");
            continue;
        }

        // find if one of the desired axis is in this device
        yarp::dev::IAxisInfo *iaxinfos = 0;
        yarp::dev::IEncoders *iencs = 0;
        polylist[p]->poly->view(iaxinfos);
        polylist[p]->poly->view(iencs);

        if( !iencs ||
            !iaxinfos )
        {
            yError() <<"ReadOnlyControlBoardRemapper: subdevice " << deviceKey << " does not implemented the required IAxisInfo or IEncoders interfaces";
            return false;
        }

        int nrOfSubdeviceAxes;
        bool ok = iencs->getAxes(&nrOfSubdeviceAxes);

        if( !ok )
        {
            yError() <<"ReadOnlyControlBoardRemapper: subdevice " << deviceKey << " does not implemented the required getAxes method";
            return false;
        }

        for(int axInSubDevice =0; axInSubDevice < nrOfSubdeviceAxes; axInSubDevice++)
        {
            yarp::os::ConstString axNameYARP;
            ok = iaxinfos->getAxisName(axInSubDevice,axNameYARP);

            std::string axName = axNameYARP.c_str();

            if( !ok )
            {
                yError() <<"ReadOnlyControlBoardRemapper: subdevice " << deviceKey << " does not implemented the required getAxisName method";
                return false;
            }

            std::map<std::string, axisLocation>::iterator it = axesLocationMap.find(axName);
            if( it != axesLocationMap.end() )
            {
                yError() <<"ReadOnlyControlBoardRemapper: multiple axes have the same name " << axName
                         <<" on on device " << polylist[p]->key << " with index  " << axInSubDevice
                         <<" and another on device " << it->second.subDeviceKey << " with index " << it->second.indexInSubDevice;
                return false;
            }

            axisLocation newLocation;
            newLocation.subDeviceKey = polylist[p]->key;
            newLocation.indexInSubDevice = axInSubDevice;
            newLocation.indexOfSubDeviceInPolyDriverList = p;
            axesLocationMap[axName] = newLocation;
        }
    }

    // We store the key of all the devices that we actually use in the remapped control device
    std::vector<std::string> subControlBoardsKeys;
    std::map<std::string, size_t> subControlBoardKey2IndexInPolyDriverList;
    std::map<std::string, size_t> subControlBoardKey2IndexInRemappedControlBoards;


    // Once we build the axis map, we build the mapping between the remapped axes and
    // the couple subControlBoard, axis in subControlBoard
    for(size_t l=0; l < axesNames.size(); l++)
    {
        std::map<std::string, axisLocation>::iterator it = axesLocationMap.find(axesNames[l]);
        if( it == axesLocationMap.end() )
        {
            yError() <<"ReadOnlyControlBoardRemapper: axis " << axesNames[l]
                     <<" specified in axesNames was not found in the axes of the controlboards "
                     <<"passed to attachAll, attachAll failed. ";
            return false;
        }

        axisLocation loc = it->second;
        std::string key = loc.subDeviceKey;

        if(std::find(subControlBoardsKeys.begin(), subControlBoardsKeys.end(), key) == subControlBoardsKeys.end())
        {
            /* subControlBoardsKeys does not contain key */
            subControlBoardKey2IndexInRemappedControlBoards[key] = subControlBoardsKeys.size();
            subControlBoardsKeys.push_back(key);
            subControlBoardKey2IndexInPolyDriverList[key] = loc.indexOfSubDeviceInPolyDriverList;
        }
    }

    assert(controlledJoints == (int) axesNames.size());

    // We have now the number of controlboards to attach to
    size_t nrOfSubControlBoards = subControlBoardsKeys.size();
    remappedControlBoards.lut.resize(controlledJoints);
    remappedControlBoards.subdevices.resize(nrOfSubControlBoards);

    // Open the controlboards
    for(size_t ctrlBrd=0; ctrlBrd < nrOfSubControlBoards; ctrlBrd++)
    {
        size_t p = subControlBoardKey2IndexInPolyDriverList[subControlBoardsKeys[ctrlBrd]];
        RemappedSubControlBoard *tmpDevice = remappedControlBoards.getSubControlBoard(ctrlBrd);
        tmpDevice->setVerbose(_verb);
        tmpDevice->id = subControlBoardsKeys[ctrlBrd];
        bool ok = tmpDevice->attach(polylist[p]->poly,subControlBoardsKeys[ctrlBrd]);

        if( !ok )
        {
            return false;
        }
    }


    for(size_t l=0; l < axesNames.size(); l++)
    {
        axisLocation loc = axesLocationMap[axesNames[l]];
        remappedControlBoards.lut[l].subControlBoardIndex = subControlBoardKey2IndexInRemappedControlBoards[loc.subDeviceKey];
        remappedControlBoards.lut[l].axisIndexInSubControlBoard = (size_t)loc.indexInSubDevice;
    }

    return true;
}


bool ReadOnlyControlBoardRemapper::attachAllUsingNetworks(const PolyDriverList &polylist)
{
    for(int p=0;p<polylist.size();p++)
    {
        // look if we have to attach to a calibrator
        std::string subDeviceKey = polylist[p]->key.c_str();
        if(subDeviceKey == "Calibrator" || subDeviceKey == "calibrator")
        {
            yWarning("Calibrator not supported");
            continue;
        }

        // find appropriate entry in list of subdevices and attach
        unsigned int k=0;
        for(k=0; k<remappedControlBoards.getNrOfSubControlBoards(); k++)
        {
            if (remappedControlBoards.subdevices[k].id==subDeviceKey)
            {
                if (!remappedControlBoards.subdevices[k].attach(polylist[p]->poly, subDeviceKey))
                {
                    yError("ReadOnlyControlBoardRemapper: attach to subdevice %s failed\n", polylist[p]->key.c_str());
                    return false;
                }
            }
        }
    }

    bool ok = updateAxesName();

    if( !ok )
    {
        yWarning() << "ReadOnlyControlBoardRemapper: unable to update axesNames";
    }

    return true;
}


bool ReadOnlyControlBoardRemapper::detachAll()
{
    //check if we already instantiated a subdevice previously
    int devices=remappedControlBoards.getNrOfSubControlBoards();
    for(int k=0;k<devices;k++)
        remappedControlBoards.getSubControlBoard(k)->detach();

    return true;
}

void ReadOnlyControlBoardRemapper::configureBuffers()
{
    allJointsBuffers.configure(remappedControlBoards);
    selectedJointsBuffers.configure(remappedControlBoards);
}


//////////////////////////////////////////////////////////////////////////////
/// ControlBoard methods
//////////////////////////////////////////////////////////////////////////////

/* IPositionControl */
bool ReadOnlyControlBoardRemapper::getAxes(int *ax)
{
    *ax=controlledJoints;
    return true;
}


/* IEncoders */
bool ReadOnlyControlBoardRemapper::resetEncoder(int j)
{
    int off=(int)remappedControlBoards.lut[j].axisIndexInSubControlBoard;
    size_t subIndex=remappedControlBoards.lut[j].subControlBoardIndex;

    yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);
    if (!p)
    {
        return false;
    }

    if (p->iJntEnc)
    {
        return p->iJntEnc->resetEncoder(off);
    }

    return false;
}

bool ReadOnlyControlBoardRemapper::resetEncoders()
{
    bool ret=true;

    for(int l=0;l<controlledJoints;l++)
    {
        int off=(int)remappedControlBoards.lut[l].axisIndexInSubControlBoard;
        size_t subIndex=remappedControlBoards.lut[l].subControlBoardIndex;

        yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);
        if (!p)
        {
            return false;
        }

        if (p->iJntEnc)
        {
            bool ok = p->iJntEnc->resetEncoder(off);
            ret = ret && ok;
        }
        else
        {
            ret=false;
        }
    }
    return ret;
}

bool ReadOnlyControlBoardRemapper::setEncoder(int j, double val)
{
    int off=(int)remappedControlBoards.lut[j].axisIndexInSubControlBoard;
    size_t subIndex=remappedControlBoards.lut[j].subControlBoardIndex;

    yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

    if (!p)
    {
        return false;
    }

    if (p->iJntEnc)
    {
        return p->iJntEnc->setEncoder(off,val);
    }

    return false;
}

bool ReadOnlyControlBoardRemapper::setEncoders(const double *vals)
{
    bool ret=true;

    for(int l=0;l<controlledJoints;l++)
    {
        int off = (int) remappedControlBoards.lut[l].axisIndexInSubControlBoard;
        size_t subIndex = remappedControlBoards.lut[l].subControlBoardIndex;

        yarp::dev::RemappedSubControlBoard *p = remappedControlBoards.getSubControlBoard(subIndex);

        if (!p)
        {
            return false;
        }

        if (p->iJntEnc)
        {
            bool ok = p->iJntEnc->setEncoder(off, vals[l]);
            ret = ret && ok;
        }
        else
        {
            ret = false;
        }
    }
    return ret;
}

bool ReadOnlyControlBoardRemapper::getEncoder(int j, double *v)
{
    int off=(int)remappedControlBoards.lut[j].axisIndexInSubControlBoard;
    size_t subIndex=remappedControlBoards.lut[j].subControlBoardIndex;

    yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

    if (!p)
    {
        return false;
    }

    if (p->iJntEnc)
    {
        return p->iJntEnc->getEncoder(off, v);
    }

    return false;
}

bool ReadOnlyControlBoardRemapper::getEncoders(double *encs)
{
    bool ret=true;

    for(int l=0;l<controlledJoints;l++)
    {
        int off=(int)remappedControlBoards.lut[l].axisIndexInSubControlBoard;
        size_t subIndex=remappedControlBoards.lut[l].subControlBoardIndex;

        yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

        if (!p)
        {
            return false;
        }

        if (p->iJntEnc)
        {
            bool ok = p->iJntEnc->getEncoder(off, encs+l);
            ret = ret && ok;
        }
        else
        {
            ret = false;
        }
    }
    return ret;
}

bool ReadOnlyControlBoardRemapper::getEncodersTimed(double *encs, double *t)
{
    bool ret=true;

    for(int l=0;l<controlledJoints;l++)
    {
        int off=(int)remappedControlBoards.lut[l].axisIndexInSubControlBoard;
        size_t subIndex=remappedControlBoards.lut[l].subControlBoardIndex;

        yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

        if (!p)
        {
            return false;
        }

        if (p->iJntEnc)
        {
            bool ok = p->iJntEnc->getEncoderTimed(off, encs+l, t+l);
            ret = ret && ok;
        }
        else
        {
            ret = false;
        }
    }
    return ret;
}

bool ReadOnlyControlBoardRemapper::getEncoderTimed(int j, double *v, double *t)
{
    int off=(int)remappedControlBoards.lut[j].axisIndexInSubControlBoard;
    size_t subIndex=remappedControlBoards.lut[j].subControlBoardIndex;

    yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

    if (!p)
    {
        return false;
    }

    if (p->iJntEnc)
    {
        return p->iJntEnc->getEncoderTimed(off, v, t);
    }

    return false;
}

bool ReadOnlyControlBoardRemapper::getEncoderSpeed(int j, double *sp)
{
    int off=(int)remappedControlBoards.lut[j].axisIndexInSubControlBoard;
    size_t subIndex=remappedControlBoards.lut[j].subControlBoardIndex;

    yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

    if (!p)
    {
        return false;
    }

    if (p->iJntEnc)
    {
        return p->iJntEnc->getEncoderSpeed(off, sp);
    }

    return false;
}

bool ReadOnlyControlBoardRemapper::getEncoderSpeeds(double *spds)
{
    bool ret=true;

    for(int l=0;l<controlledJoints;l++)
    {
        int off=(int)remappedControlBoards.lut[l].axisIndexInSubControlBoard;
        size_t subIndex=remappedControlBoards.lut[l].subControlBoardIndex;

        yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

        if (!p)
        {
            return false;
        }

        if (p->iJntEnc)
        {
            bool ok = p->iJntEnc->getEncoderSpeed(off, spds+l);
            ret = ret && ok;
        }
        else
        {
            ret = false;
        }
    }
    return ret;
}

bool ReadOnlyControlBoardRemapper::getEncoderAcceleration(int j, double *acc)
{
    int off=(int)remappedControlBoards.lut[j].axisIndexInSubControlBoard;
    size_t subIndex=remappedControlBoards.lut[j].subControlBoardIndex;

    yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

    if (!p)
    {
        return false;
    }

    if (p->iJntEnc)
    {
        return p->iJntEnc->getEncoderAcceleration(off,acc);
    }

    return false;
}

bool ReadOnlyControlBoardRemapper::getEncoderAccelerations(double *accs)
{
    bool ret=true;

    for(int l=0;l<controlledJoints;l++)
    {
        int off=(int)remappedControlBoards.lut[l].axisIndexInSubControlBoard;
        size_t subIndex=remappedControlBoards.lut[l].subControlBoardIndex;

        yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

        if (!p)
        {
            return false;
        }

        if (p->iJntEnc)
        {
            bool ok = p->iJntEnc->getEncoderSpeed(off, accs+l);
            ret = ret && ok;
        }
        else
        {
            ret = false;
        }
    }
    return ret;
}

yarp::os::Stamp ReadOnlyControlBoardRemapper::getLastInputStamp()
{
    double averageTimestamp = 0.0;
    int collectedTimestamps = 0;

    for(int l=0;l<controlledJoints;l++)
    {
        size_t subIndex=remappedControlBoards.lut[l].subControlBoardIndex;

        yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

        if (!p)
        {
            return Stamp();
        }

        if(p->iTimed)
        {
            averageTimestamp = averageTimestamp + p->iTimed->getLastInputStamp().getTime();
            collectedTimestamps++;
        }
    }


    yarp::os::LockGuard(buffers.mutex);

    if( collectedTimestamps > 0 )
    {
        buffers.stamp.update(averageTimestamp/collectedTimestamps);
    }
    else
    {
        buffers.stamp.update();
    }
    
    return buffers.stamp;
}

/* IAxisInfo */

bool ReadOnlyControlBoardRemapper::getAxisName(int j, yarp::os::ConstString& name)
{
    int off=(int)remappedControlBoards.lut[j].axisIndexInSubControlBoard;
    size_t subIndex=remappedControlBoards.lut[j].subControlBoardIndex;

    yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

    if (!p)
    {
        return false;
    }

    if (p->info)
    {
        return p->info->getAxisName(off, name);
    }

    return false;
}

bool ReadOnlyControlBoardRemapper::getJointType(int j, yarp::dev::JointTypeEnum& type)
{
    int off = (int) remappedControlBoards.lut[j].axisIndexInSubControlBoard;
    size_t subIndex = remappedControlBoards.lut[j].subControlBoardIndex;

    yarp::dev::RemappedSubControlBoard *p = remappedControlBoards.getSubControlBoard(subIndex);

    if (!p)
    {
        return false;
    }
    
    if (p->info)
    {
        return p->info->getJointType(off, type);
    }
    
    return false;
}

    /* ITorqueControl */
    
    bool ReadOnlyControlBoardRemapper::getRefTorques(double *t)
    {    
        return false;
    }
     
    bool ReadOnlyControlBoardRemapper::getRefTorque(int j, double *t)
    {    
        return false;
    }
    
    bool ReadOnlyControlBoardRemapper::setRefTorques(const double *t)
    {    
        return false;
    }
    
    bool ReadOnlyControlBoardRemapper::setRefTorque(int j, double t)
    {    
        return false;
    }
    
    bool ReadOnlyControlBoardRemapper::getBemfParam(int j, double *bemf)
    {    
        return false;
    }
    
    bool ReadOnlyControlBoardRemapper::setBemfParam(int j, double bemf)
    {    
        return false;
    }
    
    bool ReadOnlyControlBoardRemapper::setTorquePid(int j, const Pid &pid)
    {    
        return false;
    }
         
    bool ReadOnlyControlBoardRemapper::getTorque(int j, double *t)
    {    
        int off=(int)remappedControlBoards.lut[j].axisIndexInSubControlBoard;
        size_t subIndex=remappedControlBoards.lut[j].subControlBoardIndex;
       
        yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);
       
        if (!p)
        {
            return false;
        }
       
        if (p->iTorque)
        {
            return p->iTorque->getTorque(off, t);
        }
        return false;

    }
         
    bool ReadOnlyControlBoardRemapper::getTorques(double *t)
    {    
    bool ret=true;

    for(int l=0;l<controlledJoints;l++)
    {
        int off=(int)remappedControlBoards.lut[l].axisIndexInSubControlBoard;
        size_t subIndex=remappedControlBoards.lut[l].subControlBoardIndex;

        yarp::dev::RemappedSubControlBoard *p=remappedControlBoards.getSubControlBoard(subIndex);

        if (!p)
        {
            return false;
        }

        if (p->iTorque)
        {
            bool ok = p->iTorque->getTorque(off, t+l);
            ret = ret && ok;
        }
        else
        {
            ret = false;
        }
    }
    return ret;
    
    }
        
    bool ReadOnlyControlBoardRemapper::getTorqueRange(int j, double *min, double *max)
    {    
        return false;
    }
    
    bool ReadOnlyControlBoardRemapper::getTorqueRanges(double *min, double *max)
    {    
        return false;
    }
        
    bool ReadOnlyControlBoardRemapper::setTorquePids(const Pid *pids)
    {    
        return false;
    }
    
    bool ReadOnlyControlBoardRemapper::setTorqueErrorLimit(int j, double limit)
    {    
        return false;
    }
    
    bool ReadOnlyControlBoardRemapper::setTorqueErrorLimits(const double *limits)
    {    
        return false;
    }
      
    bool ReadOnlyControlBoardRemapper::getTorqueError(int j, double *err)
    {    
        return false;
    }
       
    bool ReadOnlyControlBoardRemapper::getTorqueErrors(double *errs)
    {    
        return false;
    }
        
    bool ReadOnlyControlBoardRemapper::getTorquePidOutput(int j, double *out)
    {    
        return false;
    }
      
    bool ReadOnlyControlBoardRemapper::getTorquePidOutputs(double *outs)
    {    
        return false;
    }
        
    bool ReadOnlyControlBoardRemapper::getTorquePid(int j, Pid *pid)
    {    
        return false;
    }
       
    bool ReadOnlyControlBoardRemapper::getTorquePids(Pid *pids)
    {    
        return false;
    }
    
    bool ReadOnlyControlBoardRemapper::getTorqueErrorLimit(int j, double *limit)
    {    
        return false;
    }
     
    bool ReadOnlyControlBoardRemapper::getTorqueErrorLimits(double *limits)
    {    
        return false;
    }
      
    bool ReadOnlyControlBoardRemapper::resetTorquePid(int j)
    {    
        return false;
    }
        
    bool ReadOnlyControlBoardRemapper::disableTorquePid(int j)
    {    
        return false;
    }
       
    bool ReadOnlyControlBoardRemapper::enableTorquePid(int j)
    {    
        return false;
    }
       
    bool ReadOnlyControlBoardRemapper::setTorqueOffset(int j, double v)
    {    
        return false;
    }


}
}

