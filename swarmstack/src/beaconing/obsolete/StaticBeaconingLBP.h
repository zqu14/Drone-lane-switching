/*
 * StaticBeaconingLBP.h
 *
 *  Created on: Sep 5, 2020
 *      Author: awillig
 */

#ifndef BEACONINGLAYER_STATICBEACONINGLBP_H_
#define BEACONINGLAYER_STATICBEACONINGLBP_H_


#include <omnetpp.h>
#include "../obsolete/BeaconingProtocolBaseLBP.h"

using namespace omnetpp;

class StaticBeaconingLBP : public BeaconingProtocolLBP
{
  protected:
    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleMessage(cMessage *msg);
};


#endif /* BEACONINGLAYER_STATICBEACONINGLBP_H_ */
