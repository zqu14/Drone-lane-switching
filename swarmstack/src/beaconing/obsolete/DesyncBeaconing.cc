/*
 * DesyncBeaconing.cc
 *
 *  Created on: Jul 19, 2019
 *      Author: awillig
 */



#include "DesyncBeaconing.h"

Define_Module(DesyncBeaconing);

using namespace inet;


// ----------------------------------------------------------------------

void DesyncBeaconing::initialize(int stage)
{
    dbgEnter("DesyncBeaconing::initialize");

    BeaconingProtocol::initialize(stage);
    if (stage == INITSTAGE_LOCAL)
    {
        hasPred          =  false;
        hasSucc          =  false;
        predIdentifier   =  nullIdentifier;
        succIdentifier   =  nullIdentifier;
        currPhase        =  0.0;
    }
}

// ----------------------------------------------------------------------


void DesyncBeaconing::finish(void)
{
    dbgEnter("DesyncBeaconing::finish");
    BeaconingProtocol::finish();
}

// ----------------------------------------------------------------------

void DesyncBeaconing::readParameters(void)
{
    dbgEnter("DesyncBeaconing::readParameters");

    BeaconingProtocol::readParameters();

    adjustmentAlpha   =   par("adjustmentAlpha");
}

// ----------------------------------------------------------------------

void DesyncBeaconing::recordFiring (NodeIdentifier id, simtime_t time)
{
    dbgEnter("DesyncBeaconing::recordFiring");

    /*
    Firing f;
    f.firingId   = beacon->getSenderId();
    f.firingTime = simTime();
    recordFiring(f);

    observedFirings.push_front(f);

    // check whether we have a successor / predecessor
    if (observedFirings.size() <= 2)
    {
        // with two or fewer observed firings we do not yet have
        // neighbors
        hasPred   =  false;
        hasSucc   =  false;
    }
    else
    {
        auto it = observedFirings.begin();
        f0 = *it; ++it;
        f1 = *it; ++it;
        f2 = *it;

        if ((f0.firingId != ownIdentifier) && (f1.firingId == ownIdentifier) && (f2.firingId != ownIdentifier))
        {
            hasPred  = true;
            hasSucc  = true;
            predIdentifier = f2.firingId;
            succIdentifier = f0.firingId;
        }
        else
        {

        }
    }

    // cut off firing list after my own second-oldest firing
    std::vector<int>  ownIndices;
    int               currIdx = 0;
    for (auto it = observedFirings.begin(); it != observedFirings.end(); ++it)
    {
        if ((*it).firingId == ownIdentifier)
        {
            ownIndices.push_back(currIdx);
        }
        currIdx++;
    }
    if (ownIndices.size() > 1)
    {
        int cutCnt = (ownIndices[1])+1;
        observedFirings.resize(cutCnt);
    }
    */
    dbgLeave("DesyncBeaconing::recordFiring");
}

// ----------------------------------------------------------------------


void DesyncBeaconing::processReceivedBeacon(const Beacon* beacon)
{
    dbgEnter("DesyncBeaconing::processReceivedBeacon");

    BeaconingProtocol::processReceivedBeacon(beacon);

    recordFiring(beacon->getSenderId(), simTime());

    dbgLeave("DesyncBeaconing::processReceivedBeacon");
}


// ----------------------------------------------------------------------


