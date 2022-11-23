/*
 * BeaconingProtocol.cc
 *
 *  Created on: Apr 15, 2019
 *      Author: awillig
 */

#include <omnetpp.h>
#include "../base/BeaconingBase.h"
#include "../obsolete/BeaconingProtocolBase.h"
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

const inet::Protocol beaconingProtocol("beaconing","BEACONING");

simsignal_t beaconConsecutiveCountId = cComponent::registerSignal("beaconConsecutiveCount");
simsignal_t beaconConsecutiveDelayId = cComponent::registerSignal("beaconConsecutiveDelay");
simsignal_t numberOfNeighborsId      = cComponent::registerSignal("numberOfNeighbors");

// =======================================================================
// =======================================================================

using namespace inet;



// =======================================================================
// =======================================================================

void beaconingDebugMsg (const std::string preamble, const std::string text)
{
  EV << "Beaconing [ " << preamble << " ]: " << text << std::endl;
  EV.flush();
}


void BeaconingProtocol::debugMsg (const std::string text)
{
  std::ostringstream os;
  os << xsprintf("time=%.4lf", simTime().dbl())
     << " , id=" << ownIdentifier;
  beaconingDebugMsg (os.str(), text);
}

void BeaconingProtocol::dbgEnter (const std::string methodname)
{
  std::string et = "Entering ";
  debugMsg (et.append(methodname));
}

void BeaconingProtocol::dbgLeave (const std::string methodname)
{
  std::string lt = "Leaving ";
  debugMsg (lt.append(methodname));
}

// -------------------------------------------

void BeaconingProtocol::initialize (int stage)
{
  dbgEnter("BeaconingProtocol::initialize");
  debugMsg(xsprintf("BeaconingProtocol::initialize: stage is %d",stage));

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

      registerProtocol(beaconingProtocol, gate("lowerLayerOut"), nullptr);
      registerService(beaconingProtocol, nullptr, gate("lowerLayerIn"));
      ProtocolGroup::ethertype.addProtocol(0x8999, &beaconingProtocol);
      wlanInterfaceId = 1;
      wlanInterface = interfaces->getInterface(wlanInterfaceId);
      assert(wlanInterface);
      ownIdentifier   = wlanInterface->getMacAddress();

      EV << "Initialize: ownIdentifier = " << ownIdentifier << endl;

    }

  dbgLeave("BeaconingProtocol::initialize");
}


// -------------------------------------------

void BeaconingProtocol::readParameters (void)
{
  dbgEnter("BeaconingProtocol::readParameters");

  nominalPeriod           =  par ("nominalPeriod");
  startTime               =  par ("startTime");
  positionSamplingPeriod  =  par ("positionSamplingPeriod");
  beaconLength            =  par ("beaconLength");

  dbgLeave("BeaconingProtocol::readParameters");
}


// -------------------------------------------

void BeaconingProtocol::findModulePointers (void)
{
  dbgEnter("BeaconingProtocol::findModulePointers");

  cModule *host = getContainingNode (this);
  assert(host);
  mobility      = check_and_cast<IMobility *>(host->getSubmodule("mobility"));
  interfaces    = check_and_cast<InterfaceTable *>(host->getSubmodule("interfaceTable"));
  neighborTable = check_and_cast<NeighborTable *>(host->getSubmodule("neighborTable"));
  
  assert(mobility);
  assert(interfaces);
  assert(neighborTable);

  samplePosition();

  dbgLeave("BeaconingProtocol::findModulePointers");
}

// -------------------------------------------

void BeaconingProtocol::startSelfMessages (void)
{
  dbgEnter("BeaconingProtocol::startSelfMessages");

  pMsgGenerate = new cMessage ("BeaconingProtocol::GenerateBeacon");
  assert (pMsgGenerate);
  scheduleAt (simTime() + startTime, pMsgGenerate);

  pMsgSamplePosition = new cMessage ("BeaconingProtocol::SamplePosition");
  assert (pMsgSamplePosition);
  scheduleAt (simTime() + positionSamplingPeriod, pMsgSamplePosition);

  dbgLeave("BeaconingProtocol::startSelfMessages");
}


// -------------------------------------------

void BeaconingProtocol::samplePosition (void)
{
  dbgEnter("BeaconingProtocol::samplePositions");

  assert (mobility);
  currPosition             = mobility->getCurrentPosition();
  currVelocity             = mobility->getCurrentVelocity();

  dbgLeave("BeaconingProtocol::samplePositions");
}


// -------------------------------------------

void BeaconingProtocol::handleMessage (cMessage *msg)
{
  dbgEnter("BeaconingProtocol::handleMessage");

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


  error("BeaconingProtocol::handleMessage: cannot handle message");
}


// -------------------------------------------

void BeaconingProtocol::handleSelfMsg (cMessage *msg)
{
  dbgEnter("BeaconingProtocol::handleSelfMsg");

  if (msg == pMsgSamplePosition)
    {
      debugMsg(xsprintf("BeaconingProtocol::handleSelfMsg: sampling positions"));
      scheduleAt (simTime() + positionSamplingPeriod, pMsgSamplePosition);
      samplePosition();
      return;
    }

  if (msg == pMsgGenerate)
    {
      debugMsg(xsprintf("BeaconingProtocol::handleSelfMsg: sending beacon"));
      double jitter      = par ("periodJitter");
      scheduleAt (simTime()+nominalPeriod+jitter, pMsgGenerate);
      sendBeacon();
      return;
    }


  error("BeaconingProtocol::handleSelfMsg: cannot handle message");
}

// -------------------------------------------

void BeaconingProtocol::handleLowerMsg(cMessage* msg)
{
  dbgEnter("BeaconingProtocol::handleLowerMsg");

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

          emit(beaconConsecutiveCountId, (long) (currSeq - lastSeq));
          emit(beaconConsecutiveDelayId, currTime - lastTime);
      }

      // process the beacon
      neighborTable->recordBeacon(&(*beacon));
      processReceivedBeacon (&(*beacon));

      // record table size statistics
      emit(numberOfNeighborsId, (long) neighborTable->size());
  }

  delete msg;

  dbgLeave("BeaconingProtocol::handleLowerMsg");
}


// -------------------------------------------

void BeaconingProtocol::processReceivedBeacon(const Beacon* beacon)
{
    dbgEnter("BeaconingProtocol::processReceivedBeacon");
    
    EV << "processReceivedBeacon " << ownIdentifier << ": received beacon from " << beacon->getSenderId()
       << " on position " << beacon->getCurrPosition()
       << " with seqno " << beacon->getSeqno()
       << ", updated neighbor table = " << neighborTable
       << endl;

    dbgLeave("BeaconingProtocol::processReceivedBeacon");
}

// -------------------------------------------

Ptr<Beacon> BeaconingProtocol::composeBeacon (void)
{
  dbgEnter("BeaconingProtocol::composeSwarmBeacon");

  auto beacon = makeShared<Beacon>();
  assert(beacon);
  beacon->setMagicNo(SWARMSTACK_BEACON_MAGICNO);
  beacon->setVersion(SWARMSTACK_VERSION);
  beacon->setSenderId(ownIdentifier);
  beacon->setSeqno(sequenceNumber++);
  beacon->setCurrPosition(currPosition);
  beacon->setCurrVelocity(currVelocity);
  beacon->setChunkLength(B(beaconLength));

  dbgLeave("BeaconingProtocol::composeSwarmBeacon");

  return beacon;
}

// -------------------------------------------

void BeaconingProtocol::sendBeacon (void)
{
  dbgEnter("BeaconingProtocol::sendBeacon");

  Ptr<Beacon> beacon  = composeBeacon();
  Packet     *packet  = new Packet ("BeaconPacket");
  assert(beacon);
  assert(packet);

  packet->insertAtBack(beacon);

  packet->setKind(SWARMSTACK_BEACON_KIND);
  packet->setName("SwarmBeacon");
  packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&beaconingProtocol);
  packet->addTagIfAbsent<InterfaceReq>()->setInterfaceId(wlanInterface->getInterfaceId());
  packet->addTagIfAbsent<MacAddressReq>()->setDestAddress(MacAddress::BROADCAST_ADDRESS);

  //EV << "BeaconingProtocol::sendBeacon: the composed packet is " << packet << endl;

  sendDown(packet);
  generatedBeacons++;

  dbgLeave("BeaconingProtocol::sendBeacon");
}

// -------------------------------------------

void BeaconingProtocol::sendDown(cMessage* msg)
{
  dbgEnter("BeaconingProtocol::sendDown");

  send(msg, lowerLayerOut);

  dbgLeave("BeaconingProtocol::sendDown");
}

// -------------------------------------------

void BeaconingProtocol::finish (void)
{
    dbgEnter("BeaconingProtocol::finish");

    dbgEnter("BeaconingProtocol::finish");
}

// -------------------------------------------

BeaconingProtocol::~BeaconingProtocol (void)
{
    dbgEnter("BeaconingProtocol::~BeaconingProtocol");

    cancelAndDelete(pMsgGenerate);
    cancelAndDelete(pMsgSamplePosition);

    dbgLeave("BeaconingProtocol::~BeaconingProtocol");
}
