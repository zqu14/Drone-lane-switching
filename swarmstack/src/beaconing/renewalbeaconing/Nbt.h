/*
 * Nbt.h
 *
 *  Created on: 2022Äê4ÔÂ4ÈÕ
 *      Author: qzy0a
 */

#ifndef BEACONING_RENEWALBEACONING_NBT_H_
#define BEACONING_RENEWALBEACONING_NBT_H_

#include <iostream>
#include <omnetpp.h>

#include <inet/linklayer/common/MacAddress.h>

using namespace inet;

class Nbt {

 protected:
  MacAddress                nbId;
  simtime_t                 lastBeacon;
  Coord                     lastPosition;
  Coord                     lastVelocity;
  uint32_t                  seq;
 public:

  MacAddress getNeighborId() const {return nbId;};
  simtime_t  getLastBeaconReceptionTime() const {return lastBeacon;};
  Coord      getLastPosition() const {return lastPosition;};
  Coord      getLastVelocity() const {return lastVelocity;};
  uint32_t   getSeq() const {return seq;};
  void setId(MacAddress id) {nbId = id;};
  void setLastBeaconReceptionTime(simtime_t lastt) {lastBeacon=lastt;};
  void setLastPosition(Coord lastp) {lastPosition=lastp;};
  void setLastVelocity(Coord lastv) {lastVelocity=lastv;};
  void setSeq(uint32_t s) {seq = s;};
};

#endif /* BEACONING_RENEWALBEACONING_NBT_H_ */
