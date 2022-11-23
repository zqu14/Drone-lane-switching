/*
 * BeaconingProtocolBaseLBP.cc
 *
 *  Created on: Sep 5, 2020
 *      Author: awillig
 */


#include <omnetpp.h>
#include "../base/BeaconingBase.h"
#include "../obsolete/BeaconingProtocolBaseLBP.h"
#include <inet/common/packet/Packet.h>
#include <inet/common/ModuleAccess.h>
#include <inet/common/InitStages.h>
#include <inet/common/IProtocolRegistrationListener.h>
#include <inet/common/ProtocolGroup.h>
#include <inet/linklayer/common/MacAddress.h>
//#include <inet/linklayer/ieee80211/mac/CsmaCaMacHeader_m.h>
#include <inet/networklayer/contract/IInterfaceTable.h>
#include <inet/networklayer/common/InterfaceTable.h>
#include <inet/linklayer/common/InterfaceTag_m.h>
#include "inet/linklayer/common/MacAddressTag_m.h"
#include <cstdio>
#include <cassert>
#include <sstream>
#include <math.h>


// =======================================================================

const inet::Protocol beaconingProtocolLBP("beaconingLBP","Beaconing protocol based on LBP");

simsignal_t sigBeaconConsecutiveCountId = cComponent::registerSignal("beaconConsecutiveCount");
simsignal_t sigBeaconConsecutiveDelayId = cComponent::registerSignal("beaconConsecutiveDelay");
simsignal_t sigNumberOfNeighborsId      = cComponent::registerSignal("numberOfNeighbors");

// =======================================================================
// =======================================================================

using namespace inet;



// =======================================================================
// =======================================================================

void beaconingLBPDebugMsg (const std::string preamble, const std::string text)
{
  EV << "BeaconingLBP [ " << preamble << " ]: " << text << std::endl;
  EV.flush();
}


void BeaconingProtocolLBP::debugMsg (const std::string text)
{
  std::ostringstream os;
  os << xsprintf("time=%.4lf", simTime().dbl())
     << " , id=" << ownIdentifier;
  beaconingLBPDebugMsg (os.str(), text);
}

void BeaconingProtocolLBP::dbgEnter (const std::string methodname)
{
  std::string et = "Entering ";
  debugMsg (et.append(methodname));
}

void BeaconingProtocolLBP::dbgLeave (const std::string methodname)
{
  std::string lt = "Leaving ";
  debugMsg (lt.append(methodname));
}

// -------------------------------------------

void BeaconingProtocolLBP::initialize (int stage)
{
  dbgEnter("BeaconingProtocolLBP::initialize");
  debugMsg(xsprintf("BeaconingProtocolLBP::initialize: stage is %d",stage));

  if (stage == INITSTAGE_LOCAL)
    {
      readParameters ();
      startSelfMessages ();

      generatedBeacons   = 0;
      receivedBeacons    = 0;
      mobility           = nullptr;
      sequenceNumber     = 0;

    }

  if (stage == INITSTAGE_LAST)
    {
      findModulePointers ();

      lowerLayerIn     = findGate("lowerLayerIn");
      lowerLayerOut    = findGate("lowerLayerOut");
      neighborTableOut = findGate("neighborTableOut");

      registerProtocol(beaconingProtocolLBP, gate("lowerLayerOut"), nullptr);
      registerService(beaconingProtocolLBP, nullptr, gate("lowerLayerIn"));
      wlanInterfaceId = 1;
      wlanInterface = interfaces->getInterface(wlanInterfaceId);
      assert(wlanInterface);
      ownIdentifier   = wlanInterface->getMacAddress();

      EV << "Initialize: ownIdentifier = " << ownIdentifier << endl;

    }

  dbgLeave("BeaconingProtocolLBP::initialize");
}


// -------------------------------------------

void BeaconingProtocolLBP::readParameters (void)
{
  dbgEnter("BeaconingProtocolLBP::readParameters");

  nominalPeriod           =  par ("nominalPeriod");
  startTime               =  par ("startTime");
  positionSamplingPeriod  =  par ("positionSamplingPeriod");
  beaconLength            =  par ("beaconLength");

  dbgLeave("BeaconingProtocolLBP::readParameters");
}


// -------------------------------------------

void BeaconingProtocolLBP::findModulePointers (void)
{
  dbgEnter("BeaconingProtocolLBP::findModulePointers");

  cModule *host = getContainingNode (this);
  assert(host);
  mobility      = check_and_cast<IMobility *>(host->getSubmodule("mobility"));
  interfaces    = check_and_cast<InterfaceTable *>(host->getSubmodule("interfaceTable"));
  neighborTable = check_and_cast<NeighborTable *>(host->getSubmodule("neighborTable"));

  assert(mobility);
  assert(interfaces);
  assert(neighborTable);

  pProtocolLBP = (inet::Protocol*) Protocol::findProtocol("LocalBroadcastProtocol");
  assert(pProtocolLBP);


  samplePosition();

  dbgLeave("BeaconingProtocolLBP::findModulePointers");
}

// -------------------------------------------

void BeaconingProtocolLBP::startSelfMessages (void)
{
  dbgEnter("BeaconingProtocolLBP::startSelfMessages");

  pMsgGenerate = new cMessage ("BeaconingProtocolLBP::GenerateBeacon");
  assert (pMsgGenerate);
  scheduleAt (simTime() + startTime, pMsgGenerate);

  pMsgSamplePosition = new cMessage ("BeaconingProtocolLBP::SamplePosition");
  assert (pMsgSamplePosition);
  scheduleAt (simTime() + positionSamplingPeriod, pMsgSamplePosition);

  dbgLeave("BeaconingProtocolLBP::startSelfMessages");
}


// -------------------------------------------

void BeaconingProtocolLBP::samplePosition (void)
{
  dbgEnter("BeaconingProtocolLBP::samplePositions");

  assert (mobility);
  currPosition             = mobility->getCurrentPosition();
  currVelocity             = mobility->getCurrentVelocity();

  dbgLeave("BeaconingProtocolLBP::samplePositions");
}


// -------------------------------------------

void BeaconingProtocolLBP::handleMessage (cMessage *msg)
{
  dbgEnter("BeaconingProtocolLBP::handleMessage");

  if (msg->isSelfMessage ())
    {
      debugMsg("handleMessage: message is self-message");
      handleSelfMsg(msg);
      return;
    }

  if (msg->arrivedOn(lowerLayerIn))
    {
      debugMsg("handleMessage: message came from lower layers");
      handleLowerMsg(msg);
      return;
    }


  error("BeaconingProtocolLBP::handleMessage: cannot handle message");
}


// -------------------------------------------

void BeaconingProtocolLBP::handleSelfMsg (cMessage *msg)
{
  dbgEnter("BeaconingProtocolLBP::handleSelfMsg");

  if (msg == pMsgSamplePosition)
    {
      debugMsg(xsprintf("BeaconingProtocolLBP::handleSelfMsg: sampling positions"));
      scheduleAt (simTime() + positionSamplingPeriod, pMsgSamplePosition);
      samplePosition();
      return;
    }

  if (msg == pMsgGenerate)
    {
      debugMsg(xsprintf("BeaconingProtocolLBP::handleSelfMsg: sending beacon"));
      double jitter      = par ("periodJitter");
      scheduleAt (simTime()+nominalPeriod+jitter, pMsgGenerate);
      sendBeacon();
      return;
    }


  error("BeaconingProtocolLBP::handleSelfMsg: cannot handle message");
}

// -------------------------------------------

void BeaconingProtocolLBP::handleLowerMsg(cMessage* msg)
{
  dbgEnter("BeaconingProtocolLBP::handleLowerMsg");

  EV << "handleLowerMsg: arrived message is of kind " << msg->getKind()
     << " and has name " << msg->getName()
     << " and is of class " << msg->getClassName()
     << endl;
  auto packet = check_and_cast<Packet *>(msg);
  const auto& beacon    = packet->popAtFront<Beacon>();  // ### Check msg-getKind() before doing this

  if (beaconWellFormed(&(*beacon)))
  {
      // record statistics
      auto senderId = beacon->getSenderId();
      if (neighborTable->entryAvailable(senderId))
      {
          auto entry    = (*neighborTable)[senderId];
          auto currTime = simTime();
          auto lastTime = entry.getLastBeaconReceptionTime();
          auto currSeq  = beacon->getSeqno();
          auto lastSeq  = entry.getLastSequenceNumber();

          emit(sigBeaconConsecutiveCountId, (long) (currSeq - lastSeq));
          emit(sigBeaconConsecutiveDelayId, currTime - lastTime);
      }

      // process the beacon
      neighborTable->recordBeacon(&(*beacon));
      processReceivedBeacon (&(*beacon));

      // record table size statistics
      emit(sigNumberOfNeighborsId, (long) neighborTable->size());
  }

  delete msg;

  dbgLeave("BeaconingProtocolLBP::handleLowerMsg");
}


// -------------------------------------------

void BeaconingProtocolLBP::processReceivedBeacon(const Beacon* beacon)
{
    dbgEnter("BeaconingProtocolLBP::processReceivedBeacon");

    EV << "processReceivedBeacon " << ownIdentifier << ": received beacon from " << beacon->getSenderId()
       << " on position " << beacon->getCurrPosition()
       << " with seqno " << beacon->getSeqno()
       << ", updated neighbor table = " << neighborTable
       << endl;

    dbgLeave("BeaconingProtocolLBP::processReceivedBeacon");
}

// -------------------------------------------

Ptr<Beacon> BeaconingProtocolLBP::composeBeacon (void)
{
  dbgEnter("BeaconingProtocolLBP::composeBeacon");

  auto beacon = makeShared<Beacon>();
  assert(beacon);
  beacon->setMagicNo(SWARMSTACK_BEACON_MAGICNO);
  beacon->setVersion(SWARMSTACK_VERSION);
  beacon->setSenderId(ownIdentifier);
  beacon->setSeqno(sequenceNumber++);
  beacon->setCurrPosition(currPosition);
  beacon->setCurrVelocity(currVelocity);
  beacon->setChunkLength(B(beaconLength));

  dbgLeave("BeaconingProtocolLBP::composeBeacon");

  return beacon;
}

// -------------------------------------------

void BeaconingProtocolLBP::sendBeacon (void)
{
  dbgEnter("BeaconingProtocolLBP::sendBeacon");

  Ptr<Beacon> beacon  = composeBeacon();
  Packet     *packet  = new Packet ("BeaconPacket");
  assert(beacon);
  assert(packet);

  packet->insertAtBack(beacon);

  packet->setKind(SWARMSTACK_BEACON_KIND);
  packet->setName("Beacon");
  packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&beaconingProtocolLBP);
  auto req = packet->addTagIfAbsent<DispatchProtocolReq>();
  req->setProtocol(pProtocolLBP);
  //packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(wlanInterface->getInterfaceId());
  //packet->addTagIfAbsent<MacAddressReq>()->setDestAddress(MacAddress::BROADCAST_ADDRESS);

  EV << "BeaconingProtocolLBP::sendBeacon: the composed packet is " << packet << endl;

  sendDown(packet);
  generatedBeacons++;

  dbgLeave("BeaconingProtocolLBP::sendBeacon");
}

// -------------------------------------------

void BeaconingProtocolLBP::sendDown(cMessage* msg)
{
  dbgEnter("BeaconingProtocolLBP::sendDown");

  send(msg, lowerLayerOut);

  dbgLeave("BeaconingProtocolLBP::sendDown");
}

// -------------------------------------------

void BeaconingProtocolLBP::finish (void)
{
    dbgEnter("BeaconingProtocolLBP::finish");

    dbgEnter("BeaconingProtocolLBP::finish");
}

// -------------------------------------------

BeaconingProtocolLBP::~BeaconingProtocolLBP (void)
{
    dbgEnter("BeaconingProtocolLBP::~BeaconingProtocolLBP");

    cancelAndDelete(pMsgGenerate);
    cancelAndDelete(pMsgSamplePosition);

    dbgLeave("BeaconingProtocolLBP::~BeaconingProtocolLBP");
}


