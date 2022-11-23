/*
 * StaticBeaconing.cc
 *
 *  Created on: Apr 15, 2019
 *      Author: awillig
 */


#include "StaticBeaconing.h"

Define_Module(StaticBeaconing);

void StaticBeaconing::initialize(int stage)
{
    dbgEnter("StaticBeaconing::initialize");
    BeaconingProtocol::initialize(stage);
}

void StaticBeaconing::handleMessage(cMessage *msg)
{
    dbgEnter("StaticBeaconing::handleMessage");
    BeaconingProtocol::handleMessage(msg);
}

void StaticBeaconing::finish(void)
{
    dbgEnter("StaticBeaconing::finish");
    BeaconingProtocol::finish();
}
