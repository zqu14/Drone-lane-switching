/*
 * StaticBeaconing.h
 *
 *  Created on: Apr 15, 2019
 *      Author: awillig
 */


#ifndef __SWARMSTACK_STATICBEACONING_H_
#define __SWARMSTACK_STATICBEACONING_H_

#include <omnetpp.h>
#include "../obsolete/BeaconingProtocolBase.h"

using namespace omnetpp;

class StaticBeaconing : public BeaconingProtocol
{
  protected:
    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleMessage(cMessage *msg);
};

#endif
