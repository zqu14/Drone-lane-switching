/*
 * RenewalBeaconing.cc
 *
 *  Created on: Sep 6, 2020
 *      Author: awillig
 */



#include <inet/common/IProtocolRegistrationListener.h>
#include <inet/common/ModuleAccess.h>
#include "../base/BeaconReport_m.h"
#include "../neighbortable/NeighborTable.h"
#include "Nbt.h"
#include "../processor/Processor.h"
#include "RenewalBeaconing.h"
#include <cassert>
#include <sstream>

// =======================================================================

Define_Module(RenewalBeaconing);

const inet::Protocol renewalBeaconingProtocol("renewalBeaconing","Renewal Beaconing protocol based on LBP");

simsignal_t sigBeaconsSent = cComponent::registerSignal("renewalBeaconingBeaconsSent");

// =======================================================================
// =======================================================================

using namespace inet;



// =======================================================================
// =======================================================================

RenewalBeaconing::RenewalBeaconing ()
{
    debugMsgPrefix = "RenewalBeaconing";
}

const inet::Protocol& RenewalBeaconing::getProtocol(void) const
{
    return renewalBeaconingProtocol;
}

// -------------------------------------------

void RenewalBeaconing::initialize (int stage)
{
  enter("initialize");
  debugMsg(xsprintf("initialize: stage is %d",stage));

  LBPClientBase::initialize(stage);

  if (stage == INITSTAGE_LOCAL)
    {
      readParameters ();
      startSelfMessages ();

      mobility           = nullptr;
      sequenceNumber     = 0;

      reportingGate      = gate("beaconReport");
      toProcessor        = gate("beaconOut");

      preferredSpeed     = par("preferredSpeed");
      maxpSpeed          = par("maxSpeed");





      assert(reportingGate);
    }

  if (stage == INITSTAGE_LAST)
    {
      findModulePointers ();

      registerProtocol(renewalBeaconingProtocol, gate("toLBP"), nullptr);
      registerService(renewalBeaconingProtocol, nullptr, gate("fromLBP"));


    }

  leave("initialize");
}


// -------------------------------------------

void RenewalBeaconing::readParameters (void)
{
  enter("readParameters");

  initialWaitTime         =  par ("initialWaitTime");          assert(initialWaitTime>=0);
  positionSamplingPeriod  =  par ("positionSamplingPeriod");   assert(positionSamplingPeriod>0);
  beaconLength            =  par ("beaconLength");             assert(beaconLength>0);

  leave("readParameters");
}


// -------------------------------------------

void RenewalBeaconing::findModulePointers (void)
{
  enter("findModulePointers");

  cModule *host = getContainingNode (this);
  assert(host);
  mobility      = check_and_cast<IMobility *>(host->getSubmodule("mobility"));
  assert(mobility);

  samplePosition();

  leave("findModulePointers");
}

// -------------------------------------------

void RenewalBeaconing::startSelfMessages (void)
{
  enter("startSelfMessages");

  pMsgGenerate = new cMessage ("RenewalBeaconing::GenerateBeacon");
  assert (pMsgGenerate);
  scheduleAt (simTime() + initialWaitTime + par("iatDistribution"), pMsgGenerate);

  pMsgSamplePosition = new cMessage ("RenewalBeaconing::SamplePosition");
  assert (pMsgSamplePosition);
  scheduleAt (simTime() + positionSamplingPeriod, pMsgSamplePosition);

  leave("startSelfMessages");
}


// -------------------------------------------

void RenewalBeaconing::samplePosition (void)
{
  enter("samplePositions");

  assert (mobility);
  currPosition             = mobility->getCurrentPosition();
  currVelocity             = mobility->getCurrentVelocity();

  leave("samplePositions");
}

// -------------------------------------------

void RenewalBeaconing::handleSelfMessage (cMessage *msg)
{
  enter("handleSelfMessage");

  if (msg == pMsgSamplePosition)
    {
      debugMsg(xsprintf("handleSelfMessage: sampling positions"));
      scheduleAt (simTime() + positionSamplingPeriod, pMsgSamplePosition);
      samplePosition();
      return;
    }

  if (msg == pMsgGenerate)
    {
      debugMsg(xsprintf("handleSelfMessage: sending beacon"));
      scheduleAt (simTime()+par("iatDistribution"), pMsgGenerate);
      sendBeacon();
      return;
    }


  error("RenewalBeaconing::handleSelfMessage: cannot handle message");
}

// -------------------------------------------

void RenewalBeaconing::handleOtherMessage (cMessage *msg)
{
    error("RenewalBeaconing::handleOtherMessage: cannot handle message");
}

// -------------------------------------------

bool RenewalBeaconing::beaconWellFormed (const Beacon& beacon)
{
    //assert(beacon);

    if (beacon.getMagicNo() != SWARMSTACK_BEACON_MAGICNO)
    {
        EV << "beaconWellFormed: magicno is wrong, beacon = " << beacon << std::endl;
        return false;
    }

    if (beacon.getVersion() != SWARMSTACK_VERSION)
    {
        EV << "beaconWellFormed: wrong version number, beacon = " << beacon << std::endl;
        return false;
    }

    return true;
}

// -------------------------------------------


void RenewalBeaconing::handleReceivedBroadcast(Packet* packet)
{
  enter("handleReceivedBroadcast");

  const auto& beacon    = *(packet->popAtFront<Beacon>());

  if (beaconWellFormed(beacon))
  {
      processReceivedBeacon (beacon);
      //sendBeaconToProcessor(beacon);
  }

  delete packet;

  leave("handleReceivedBroadcast");
}

// -------------------------------------------

void RenewalBeaconing::sendBeaconReport(const Beacon& beacon)
{
    enter("sendBeaconReport");
    if ((beacon.getSenderId() != ownIdentifier) && reportingGate && reportingGate->isConnected())
    {
        debugMsg("sendBeaconReport: sending a report");
        BeaconReport  *report = new BeaconReport("BeaconReport");
        report->setSeqno(beacon.getSeqno());
        report->setSenderId(beacon.getSenderId());
        report->setSenderPosition(beacon.getCurrPosition());
        report->setSenderVelocity(beacon.getCurrVelocity());
        report->setNnbt(beacon.getNnbt());


        BeaconReport* copy = new BeaconReport(*report);

        send(report, reportingGate);
        send(copy, toProcessor);

    }
    leave("sendBeaconReport");
}

void RenewalBeaconing::sendBeaconToProcessor(Beacon* beacon)
{
    //Beacon* copy = new Beacon(*beacon);
    //send(copy, toProcessor);
}



// -------------------------------------------

void RenewalBeaconing::processReceivedBeacon(const Beacon& beacon)
{
    enter("processReceivedBeacon");

    sendBeaconReport(beacon);

    EV << "processReceivedBeacon " << ownIdentifier << ": received beacon from " << beacon.getSenderId()
       << " on position " << beacon.getCurrPosition()
       << " with seqno " << beacon.getSeqno()
       << endl;

    leave("processReceivedBeacon");
}

// -------------------------------------------

Ptr<Beacon> RenewalBeaconing::composeBeacon (void)
{
  enter("composeBeacon");

  Processor * pro = check_and_cast<Processor*>(getContainingNode(this)->getSubmodule("p"));
  auto nbTable = pro->getnbTable();
  auto seqTable = pro->getseqTable();

  auto beacon = makeShared<Beacon>();
  assert(beacon);
  beacon->setMagicNo(SWARMSTACK_BEACON_MAGICNO);
  beacon->setVersion(SWARMSTACK_VERSION);
  beacon->setSenderId(ownIdentifier);
  beacon->setSeqno(sequenceNumber++);
  beacon->setCurrPosition(currPosition);
  beacon->setCurrVelocity(currVelocity);
  //beacon->setChunkLength(B(beaconLength) + B(nbTable.size() * 13));
  beacon->setChunkLength(B(beaconLength));

  // Deprecated
  // add neighbor table to the packet
  /*
  std::list<Nbt> s;
  for (auto it = nbTable.begin(); it != nbTable.end(); ++it)
  {
      MacAddress id = it->first;
      Coord pos = it->second.getLastPosition();
      Coord vol = it->second.getLastVelocity();
      simtime_t txtime = it->second.getLastBeaconReceptionTime();
      uint32_t seq = seqTable[id];

      Nbt nbt = Nbt();

      nbt.setId(id);
      nbt.setLastBeaconReceptionTime(txtime);
      nbt.setLastPosition(pos);
      nbt.setLastVelocity(vol);
      nbt.setSeq(seq);
      s.push_back(nbt);

  }
  beacon->setNnbt(s);
  */
  leave("composeBeacon");
  return beacon;
}

// -------------------------------------------

void RenewalBeaconing::sendBeacon (void)
{
  enter("sendBeacon");

  Ptr<Beacon> beacon  = composeBeacon();
  Packet     *packet  = new Packet ("Beacon");
  assert(beacon);
  assert(packet);
  packet->insertAtBack(beacon);
  packet->setKind(SWARMSTACK_BEACON_KIND);
  sendViaLBP (packet);

  emit(sigBeaconsSent, true);

  leave("sendBeacon");
}

// -------------------------------------------

void RenewalBeaconing::finish (void)
{
}

// -------------------------------------------

RenewalBeaconing::~RenewalBeaconing (void)
{
    enter("~RenewalBeaconing");

    cancelAndDelete(pMsgGenerate);
    cancelAndDelete(pMsgSamplePosition);

    leave("~RenewalBeaconing");
}


