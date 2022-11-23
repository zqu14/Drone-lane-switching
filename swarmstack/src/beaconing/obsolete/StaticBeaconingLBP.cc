/*
 * StaticBeaconingLBP.cc
 *
 *  Created on: Sep 5, 2020
 *      Author: awillig
 */

#include "StaticBeaconingLBP.h"

Define_Module(StaticBeaconingLBP);

void StaticBeaconingLBP::initialize(int stage)
{
    dbgEnter("StaticBeaconingLBP::initialize");
    BeaconingProtocolLBP::initialize(stage);
}

void StaticBeaconingLBP::handleMessage(cMessage *msg)
{
    dbgEnter("StaticBeaconingLBP::handleMessage");
    BeaconingProtocolLBP::handleMessage(msg);
}

void StaticBeaconingLBP::finish(void)
{
    dbgEnter("StaticBeaconingLBP::finish");
    BeaconingProtocolLBP::finish();
}



