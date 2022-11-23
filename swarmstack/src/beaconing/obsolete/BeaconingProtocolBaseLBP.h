/*
 * BeaconingProtocolBaseLBP.h
 *
 *  Created on: Sep 5, 2020
 *      Author: awillig
 */

#ifndef BEACONINGLAYER_BEACONINGPROTOCOLBASELBP_H_
#define BEACONINGLAYER_BEACONINGPROTOCOLBASELBP_H_


#include <omnetpp.h>
#include <inet/common/Ptr.h>
#include <inet/common/InitStages.h>
#include <inet/common/Protocol.h>
#include <inet/common/geometry/common/Coord.h>
#include <inet/mobility/contract/IMobility.h>
#include <inet/linklayer/common/MacAddress.h>
#include <inet/networklayer/common/InterfaceTable.h>
#include "../base/Beacon_m.h"
#include "../neighbortable/NeighborTable.h"

using namespace omnetpp;

typedef inet::Coord         Coord;
typedef inet::MacAddress    MacAddress;

/**
 * @brief
 * Skeleton of a beaconing protocol
 *
 */


class BeaconingProtocolLBP : public cSimpleModule {

public:

  virtual void initialize(int stage);
  virtual void finish();
  virtual void handleMessage (cMessage* msg);
  virtual int  numInitStages () const override { return inet::NUM_INIT_STAGES; };


protected:

  // configuration data
  double   nominalPeriod;
  double   periodJitter;
  double   startTime;
  double   positionSamplingPeriod;
  int      beaconLength;

  // pointers to important other (sub-)modules
  //AnnotationManager* annotations;
  //WaveAppToMac1609_4Interface* mac;
  inet::IMobility      *mobility;
  inet::InterfaceTable *interfaces;
  inet::InterfaceEntry *wlanInterface;
  int                   wlanInterfaceId;
  NeighborTable        *neighborTable;
  inet::Protocol       *pProtocolLBP;

  // pointers to self-messages
  cMessage  *pMsgGenerate;         // generate the next packet
  cMessage  *pMsgSamplePosition;   // sample mobility model for current position, speed, etc

  // state information
  Coord        currPosition;
  Coord        currVelocity;

  // statistics
  uint32_t   generatedBeacons;
  uint32_t   receivedBeacons;

  // other variables
  uint32_t        sequenceNumber;
  NodeIdentifier  ownIdentifier;

  int         lowerLayerOut;
  int         lowerLayerIn;
  int         neighborTableOut;

protected:

  // debugging functions
  virtual void debugMsg (const std::string text);
  virtual void dbgEnter (const std::string methodname);
  virtual void dbgLeave (const std::string methodname);

  virtual void readParameters (void);
  virtual void findModulePointers (void);
  virtual void startSelfMessages(void);

  virtual void samplePosition (void);

  virtual inet::Ptr<Beacon> composeBeacon (void);
  virtual void sendBeacon (void);

  virtual void sendDown(cMessage* msg);
  virtual void handleLowerMsg(cMessage *msg);
  virtual void handleSelfMsg (cMessage* msg);

  virtual void processReceivedBeacon(const Beacon* beacon);

  virtual ~BeaconingProtocolLBP();
};



#endif /* BEACONINGLAYER_BEACONINGPROTOCOLBASELBP_H_ */
