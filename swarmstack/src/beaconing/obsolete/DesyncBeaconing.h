/*
 * DesyncBeaconing.h
 *
 *  Created on: Jul 19, 2019
 *      Author: awillig
 */

#ifndef __SWARMSTACK_DESYNCBEACONING_H_
#define __SWARMSTACK_DESYNCBEACONING_H_

#include <omnetpp.h>
#include "../obsolete/BeaconingProtocolBase.h"
#include "../base/Beacon_m.h"
#include <list>

using namespace omnetpp;

// ======================================================
// Helper classes
// ======================================================

class Firing {
public:
    simtime_t       firingTime;
    NodeIdentifier  firingId;
};


// ======================================================
// Main class for Desync protocol
// ======================================================

/*
 * Basic strategy: we record all firings including our own.
 */


class DesyncBeaconing : public BeaconingProtocol
{
  public:
    virtual void finish();
    virtual void initialize(int stage);

  protected:

    // configuration parameters
    double     adjustmentAlpha;


    // runtime data
    bool               hasPred;
    bool               hasSucc;
    NodeIdentifier     predIdentifier;
    NodeIdentifier     succIdentifier;
    double             currPhase;
    std::list<Firing>  observedFirings;

  protected:
    virtual void readParameters(void);
    virtual void processReceivedBeacon(const Beacon* beacon);

  protected:
    virtual void recordFiring(NodeIdentifier id, simtime_t time);
};

#endif /* __SWARMSTACK_DESYNCBEACONING_H_ */
