//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "Processor.h"
#include <inet/common/InitStages.h>
#include "../neighbortable/NeighborTable.h"
#include <inet/common/packet/Packet.h>
#include "../../lbp/LocalBroadcastProtocol.h"
#include <inet/common/ModuleAccess.h>


Define_Module(Processor);
simsignal_t countId = cComponent::registerSignal("overtakeCount");
simsignal_t perferredSpeedId = cComponent::registerSignal("perferredSpeedValue");
simsignal_t speedCostId = cComponent::registerSignal("speedCost");
simsignal_t switchCostId = cComponent::registerSignal("switchCost");
simsignal_t positionCostId = cComponent::registerSignal("positionCost");
simsignal_t totalCostId = cComponent::registerSignal("totalCost");


using namespace inet;

Processor::Processor (double alpha, double timeout)
{
  setEwmaAlpha(alpha);
  setTimeoutTime(timeout);
}

void Processor::finish()
{
    emit(speedCostId, totalSpeedCost);
    emit(switchCostId, totalSwitchCost);
    emit(positionCostId, totalPositionCost);
    emit(totalCostId, totalCost);


}

Processor::~Processor()
{

    cancelAndDelete(pMsgTableCleanup);
    cancelAndDelete(waitingOvertake);
    cancelAndDelete(stopOvertake);
    cancelAndDelete(startPlan);
    cancelAndDelete(checkOvertakeSlow);
}

// ----------------------------------------------------

void Processor::setEwmaAlpha (double alpha)
{
  assert ((0 <= alpha) && (alpha <= 1));
  ewmaAlpha = alpha;

  for (auto it = nbTable.begin(); it != nbTable.end(); ++it)
    {
      (it->second).getEstimator().setAlpha(alpha);
    }
}
// ----------------------------------------------------

void Processor::setTimeoutTime (simtime_t timeout)
{
  assert ((0 < timeout.dbl()));
  timeoutTime = timeout;
}

// ----------------------------------------------------

void Processor::setCleanupPeriod (simtime_t period)
{
  assert ((0 < period.dbl()));
  cleanupPeriod = period;
}
// ----------------------------------------------------


void Processor::initialize (int stage)
{
  if (stage == INITSTAGE_LOCAL)
    {

      gidBeaconsIn = findGate("beaconsIn");

      double alpha   =   par ("plrEstimatorAlpha");
      double timeout =   par ("tableTimeoutValue");
      double period  =   par ("tableCleanupPeriod");

      setEwmaAlpha (alpha);
      setTimeoutTime (timeout);
      setCleanupPeriod (period);

      minimumSafetyDistance = par("minimumSafetyDistance");
      deploymentRadiusMeter = par("deploymentRadiusMeter");
      laneRadiusMeter       = par("laneRadiusMeter");
      enableOvertake        = par("enableOvertake");
      isback                = par("isback");
      maxNeighborRange      = par("maxNeighborRange");
      clTimeThreshold       = par("clTimeThreshold");
      overtakeSpeed         = par("overtakeSpeed");
      otherLaneSafetyRange  = par("E2");
      targetLaneSafetyRange = par("E1");
      KAPPA_Speed = par("K2");
      KAPPA_Position = par("K1");
      KAPPA_Switch = par("K3");

      checkOvertakeSlow     = new cMessage("checkOvertakeSlow");
      stopOvertake          = new cMessage("stopOvertake");
      waitingOvertake       = new cMessage("waitingOvertake");
      startPlan             = new cMessage("startPlan");
      pMsgTableCleanup      = new cMessage("Processor::Cleanup");

      assert(pMsgTableCleanup);
      scheduleAt(simTime() + cleanupPeriod, pMsgTableCleanup);

    }

  if (stage == INITSTAGE_LAST)
  {
      cModule *host = getContainingNode(this);
      assert(host);
      LocalBroadcastProtocol *lbp = check_and_cast<LocalBroadcastProtocol*>(host->getSubmodule("lbp"));
      assert(lbp);
      ownIdentifier  =  lbp->getOwnMacAddress();
      rb = check_and_cast<RenewalBeaconing*>(host->getSubmodule("rb"));
      mobility  = check_and_cast<DroneLinearMobility*>(host->getSubmodule("mobility"));
      assert(mobility);

      preferredSpeed = rb->getPreferredSpeedd();
      maxSpeed = rb->getMaxSpeed();
      emit(perferredSpeedId, preferredSpeed);

      if (enableOvertake != 0) {
        scheduleAt(simTime()+0.4, startPlan);
      }

  }

}
// ----------------------------------------------------

void Processor::handleMessage(cMessage *msg)
{
  if (msg == pMsgTableCleanup)
  {
      cleanupTable();
      scheduleAt (simTime()+cleanupPeriod, pMsgTableCleanup);
      return;
  }
  else if (dynamic_cast<BeaconReport*>(msg) && msg->arrivedOn(gidBeaconsIn))
  {
      BeaconReport* beacon = (BeaconReport*) msg;
      recordBeacon(&(*beacon));
      delete msg;
      return;
  } else if (msg == stopOvertake)
  {
      Overtake();
      return;
  } else if (msg == waitingOvertake)
  {
      return;
  } else if (msg == startPlan)
  {
      runAlgorithm();
      return;
  } else if (msg == checkOvertakeSlow) {
      return;
  }
  EV << "Processor::handleMessage: got improper message type" << endl;
  endSimulation();
}
// ----------------------------------------------------

void Processor::printTable()
{
    EV << "Processor::handleMessage: Starting print Table for node "<< ownIdentifier << endl;
    EV << "----------------------------------------------------------------------------------" << endl;
    EV << "Address              Position               Velocity          Estimate Collision time" << endl;

    for (auto it = nbTable.begin(); it != nbTable.end(); ++it)
    {
        EV << it->first << "    "
           << it->second.getLastPosition() << "    "
           << it->second.getLastVelocity() << "       "
           << it->second.getEstCollsionTime() << endl;
    }
    EV << "Processor::handleMessage: Printing finished"<< endl;
}


// ----------------------------------------------------

void Processor::updateNbTable(std::list<Nbt> nnbt)
{

    for (auto it = nnbt.begin(); it != nnbt.end();++it) {
        MacAddress id = it->getNeighborId();
        if (id.equals(ownIdentifier)) continue;
        simtime_t rxTime = it->getLastBeaconReceptionTime();
        Coord       pos     = it->getLastPosition();
        // ************* ignore far drones *********************
        if (pos.distance(rb->getCurrPosition()) > maxNeighborRange) continue;
        // *****************************************************
        Coord       vel     = it->getLastVelocity();
        uint32_t    seq     = it->getSeq();
        double      clTime  = LaneUtil::estimateCollisionTime(rb->getCurrVelocity(), vel, rb->getCurrPosition(), pos, minimumSafetyDistance, laneRadiusMeter);
        if (entryAvailable(id))
        {
            auto entry = nbTable[id];
            if (nbSeqTable[id] < seq){
                entry.setLastBeaconReceptionTime (rxTime);
                entry.setLastPosition (pos);
                entry.setLastVelocity(vel);
                nblostTable[id] += (seq - nbSeqTable[id] -1);
                nbSeqTable[id] = seq;
                entry.setEstCollsionTime(clTime);
                nbTable[id] = entry;
            }
        } else {
            NeighborTableEntry newEntry = NeighborTableEntry(id, rxTime, pos, vel, ewmaAlpha, clTime);
            nbSeqTable[id] = seq;
            nblostTable[id] = 0;
            nbTable[id] = newEntry;
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------

/*
 * Record the beacon in neighbor table and start to try to avoid the detected collision
 */
void Processor::recordBeacon (BeaconReport* beacon)
{


  assert(beacon);
  MacAddress  id      = beacon->getSenderId();
  simtime_t   rxTime  = simTime();
  Coord       pos     = beacon->getSenderPosition();
  Coord       vel     = beacon->getSenderVelocity();
  uint32_t    seqno   = beacon->getSeqno();

  //update neighbor table using the neighbor table in the beacon
  /*
  std::list<Nbt> nnbt    = beacon->getNnbt();
  updateNbTable(nnbt);
  */

  double      clTime  = LaneUtil::estimateCollisionTime(rb->getCurrVelocity(), vel, rb->getCurrPosition(), pos, minimumSafetyDistance, laneRadiusMeter);
  // ************* ignore far drones *********************
  if (pos.distance(rb->getCurrPosition()) <= maxNeighborRange) {
  // *****************************************************

      if (entryAvailable(id)) {
          auto entry = nbTable[id];
          entry.setLastBeaconReceptionTime (rxTime);
          entry.setLastPosition (pos);
          entry.setLastVelocity(vel);
          entry.incrNumObservedBeacons();
          entry.getEstimator().recordObservation(seqno, rxTime);
          entry.setEstCollsionTime(clTime);
          nbTable[id] = entry;
          if (seqno - nbSeqTable[id] > 1) {
              nblostTable[id] += (seqno - nbSeqTable[id] -1);
          }
          nbSeqTable[id] = seqno;
      }
      else {
          NeighborTableEntry newEntry = NeighborTableEntry(id, rxTime, pos, vel, ewmaAlpha, clTime);
          nbSeqTable[id] = seqno;
          nblostTable[id] = 0;
          nbTable[id] = newEntry;
      }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------

/*
 * Check whether a drone can start lane switching or overtake
 */
void Processor::runAlgorithm()
{
    frontDroneSpeed = preferredSpeed;
    if (overtakePlan_arrivalTime != -1) {
        scheduleAt(simTime()+3, startPlan);
        return;
    }
    // select a feasible lane
    Coord lane;
    lane = selectLane(mobility->getCurrentPosition());
    // if that lane existed, start overtake or switching
    if (!LaneUtil::isSameLane(mobility->getCurrentPosition(), lane, laneRadiusMeter)) {
        //if (rb->getCurrVelocity().y != preferredSpeed) mobility->setSpeed(KEEP, preferredSpeed, KEEP);
        setOvertakeParam(lane);
        Overtake(overtakePlan_velocity);
        scheduleAt (simTime()+overtakePlan_arrivalTime, stopOvertake);
    // if no, slow down
    } else {
        mobility->setSpeed(KEEP, frontDroneSpeed, KEEP);
    }
    scheduleAt(simTime()+3, startPlan);
}

// ------------------------------------------------------------------------------------------------------------------------------------

/*
 * Set overtake parameters
 */
void Processor::setOvertakeParam(Coord lane)
{
    back_lane = mobility->getCurrentPosition();
    Coord overtakeVelocity;
    Coord ownPosition = rb->getCurrPosition();
    Coord ownVelocity = rb->getCurrVelocity();
    Coord diff = lane - ownPosition;
    double x = diff.x;
    double z = diff.z;
    diff.y = 0;
    Coord n = diff.normalize();
    //double x_z_velocity = pow(maxSpeed, 2) - pow(preferredSpeed, 2);
    double x_z_velocity = overtakeSpeed;
    //overtakeVelocity.x = sqrt(x_z_velocity / (1+(pow(n.z,2)/pow(n.x,2))));
    //overtakeVelocity.z = sqrt(x_z_velocity / (1+(pow(n.x,2)/pow(n.z,2))));
    overtakeVelocity.x = x_z_velocity * n.x;
    overtakeVelocity.z = x_z_velocity * n.z;
    //if (n.x < 0) overtakeVelocity.x *= -1;
    //if (n.z < 0) overtakeVelocity.z *= -1;
    double arriveTime;
    if (overtakeVelocity.x != 0) {
        arriveTime = x / overtakeVelocity.x;
    } else {
        arriveTime = z / overtakeVelocity.z;
    }
    overtakePlan_arrivalTime = arriveTime;
    overtakePlan_velocity = overtakeVelocity;
}

// ------------------------------------------------------------------------------------------------------------------------------------

/*
 * Try to find a feasible lane to switch into
 */
Coord Processor::selectLane(Coord currentPosi)
{
    double minCost = MAX;
    double minPositionCost = MAX;
    double minSpeedCost = MAX;
    double minSwitchCost = MAX;
    Coord target;
    std::vector<Coord> neighbors = LaneUtil::getNeighborPosi(currentPosi, laneRadiusMeter);
    std::vector<double> speeds = getSpeeds();
    std::vector<double> positionCosts = getPositionCosts();
    std::vector<double> switchCosts = getSwitchCosts();
    std::vector<double> speedCosts;
    std::vector<double> costs;
    for(int i=0; i<=NEIGHBOR_LANE_NUM+1; i++) {
        speedCosts.push_back(0);
        speedCosts[i] = KAPPA_Speed * (preferredSpeed - speeds[i]) * (preferredSpeed - speeds[i]);
        costs.push_back(speedCosts[i]+positionCosts[i]+switchCosts[i]);
        //EV <<speedCosts[i]<<"       "<<positionCosts[i]<<"        "<<switchCosts[i] <<endl;
    }
    for(int i=0; i<=NEIGHBOR_LANE_NUM; i++) {
        if (checkTargetLane(neighbors[i], currentPosi)) {
            if (minCost > costs[i]) {
                minCost = costs[i];
                minPositionCost = positionCosts[i];
                minSpeedCost = speedCosts[i];
                minSwitchCost = switchCosts[i];
                target = neighbors[i];
                frontDroneSpeed = speeds[i];
            }
        }
    }
    //EV <<"1:         "<<minSpeedCost<<"       "<<minPositionCost<<"        "<<minSwitchCost <<endl;
    if (costs[NEIGHBOR_LANE_NUM+1] <= minCost) {
        minCost = costs[NEIGHBOR_LANE_NUM+1];
        minPositionCost = positionCosts[NEIGHBOR_LANE_NUM+1];
        minSpeedCost = speedCosts[NEIGHBOR_LANE_NUM+1];
        minSwitchCost = switchCosts[NEIGHBOR_LANE_NUM+1];
        target = currentPosi;
        frontDroneSpeed = speeds[NEIGHBOR_LANE_NUM+1];
    }
    //EV <<"2:          "<<minSpeedCost<<"       "<<minPositionCost<<"        "<<minSwitchCost <<endl;
    totalCost += minCost;
    totalSpeedCost += minSpeedCost;
    totalPositionCost += minPositionCost;
    totalSwitchCost += minSwitchCost;
    //EV << simTime() <<"       "<<frontDroneSpeed <<endl;
    return target;
}


std::vector<double>  Processor::getSpeeds()
{
    Coord currentP = rb->getCurrPosition();
    std::vector<Coord> neighbors = LaneUtil::getNeighborPosi(currentP, laneRadiusMeter);
    std::vector<double> speeds;
    std::vector<double> front;
    std::vector<double> behind;
    for(int i=0; i<=NEIGHBOR_LANE_NUM+1; i++) {
        front.push_back(MAX);
        behind.push_back(MAX);
        speeds.push_back(preferredSpeed);
    }
    for (auto it = nbTable.begin(); it != nbTable.end(); ++it)
    {
        Coord otherPosition = it->second.getLastPosition();
        Coord otherVelocity = it->second.getLastVelocity();

        if (LaneUtil::isSameLane(currentP, otherPosition, laneRadiusMeter)) {
            //EV << it->first << "     " <<  otherVelocity.y << "      " <<  (otherPosition.y - currentP.y) << endl;
            if (currentP.y < otherPosition.y && (otherPosition.y - currentP.y) < front[NEIGHBOR_LANE_NUM+1]) {

                front[NEIGHBOR_LANE_NUM+1] = otherPosition.y - currentP.y;
                speeds[NEIGHBOR_LANE_NUM+1] = preferredSpeed;
                if (otherVelocity.y < preferredSpeed) {
                    //speedcosts[NEIGHBOR_LANE_NUM+1] = KAPPA_Speed * (preferredSpeed - otherVelocity.y) * (preferredSpeed - otherVelocity.y);
                    //frontDroneSpeed = otherVelocity.y;
                    speeds[NEIGHBOR_LANE_NUM+1] = otherVelocity.y;
                }
            }
        } else {
            for(int i=0; i<=NEIGHBOR_LANE_NUM; i++) {
                if (LaneUtil::isSameLane(otherPosition, neighbors[i], laneRadiusMeter)) {
                    if (currentP.y >= otherPosition.y && (currentP.y - otherPosition.y) < behind[i]) {
                        behind[i] = currentP.y - otherPosition.y;

                    }

                    if (currentP.y < otherPosition.y && (otherPosition.y - currentP.y) < front[i]) {
                        front[i] = otherPosition.y - currentP.y;
                        speeds[i] = preferredSpeed;

                        if (otherVelocity.y < preferredSpeed) {
                            //speedcosts[i] = KAPPA_Speed * (preferredSpeed - otherVelocity.y) * (preferredSpeed - otherVelocity.y);
                            //frontDroneSpeed = otherVelocity.y;
                            speeds[i] = otherVelocity.y;
                        }
                    }
                    break;
                }
            }
        }
    }
    /**
    for(int i=0; i<=NEIGHBOR_LANE_NUM; i++) {
       if (front[i] > 100) speedcosts[i] = 0;
    }
    if (front[NEIGHBOR_LANE_NUM+1] > 20) {
        speedcosts[NEIGHBOR_LANE_NUM+1] = 0;
        frontDroneSpeed = preferredSpeed;
    }
    **/
    return speeds;
}

std::vector<double> Processor::getPositionCosts()
{
    Coord currentP = rb->getCurrPosition();
    std::vector<Coord> neighbors = LaneUtil::getNeighborPosi(currentP, laneRadiusMeter);
    std::vector<double> positionCosts;
    for(int i=0; i<=NEIGHBOR_LANE_NUM+1; i++) {
        positionCosts.push_back(0);
    }
    for(int i=0; i<=NEIGHBOR_LANE_NUM; i++) {
        int t = LaneUtil::getTier(LaneUtil::coordToLane(neighbors[i], laneRadiusMeter));
        if (t >= 2) {
            positionCosts[i] = KAPPA_Position * (t-1) * (t-1);
        }
    }
    int t = LaneUtil::getTier(LaneUtil::coordToLane(currentP, laneRadiusMeter));
    if (t >= 2) {
        positionCosts[NEIGHBOR_LANE_NUM+1] = KAPPA_Position * (t-1) * (t-1);
    }
    return positionCosts;
}

std::vector<double> Processor::getSwitchCosts()
{
    std::vector<double> switchCosts;
    for(int i=0; i<=NEIGHBOR_LANE_NUM; i++) {
        switchCosts.push_back(KAPPA_Switch);
    }
    switchCosts.push_back(0);
    return switchCosts;
}
// ------------------------------------------------------------------------------------------------------------------------------------

/*
 * Check whether lanes are feasible
 */
bool Processor::checkTargetLane(Coord targetLane, Coord originLane)
{
    if (1) {
        std::vector<Coord> neighbor = LaneUtil::getNeighborPosi(targetLane, laneRadiusMeter);
        if (!checkOneLane(targetLane, CHECK_TARGET_LANE)) return false;
        if (!checkOneLane(originLane, CHECK_OWN_LANE)) return false;
            for(int i =0; i<=NEIGHBOR_LANE_NUM;i++)
            {
                Coord posi = neighbor[i];
                if ((LaneUtil::isSameLane(posi, originLane, laneRadiusMeter))) {
                    continue;
                }
                if (!checkOneLane(posi, CHECK_NEIGHBOR_LANE)) return false;
            }
        return true;
    } else {
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------

/*
 * Check whether a lane is feasible
 */

bool Processor::checkOneLane(Coord lane, int mode)
{
    for (auto it = nbTable.begin(); it != nbTable.end(); ++it)
    {
        Coord otherPosition = it->second.getLastPosition();
        Coord otherVelocity = it->second.getLastVelocity();
        if (mode == CHECK_OWN_LANE|| mode == CHECK_NEIGHBOR_LANE) {
            if (LaneUtil::isSameLane(lane, otherPosition, laneRadiusMeter) && std::abs(lane.y-otherPosition.y)<=otherLaneSafetyRange) return false;
        } else if (mode == CHECK_TARGET_LANE) {
            if (LaneUtil::isSameLane(lane, otherPosition, laneRadiusMeter) && std::abs(lane.y-otherPosition.y)<=targetLaneSafetyRange) return false;
        }
    }
    return true;
}


// ------------------------------------------------------------------------------------------------------------------------------------

void Processor::cleanupTable (void)
{
    simtime_t currTime = simTime();

    for (auto it = nbTable.begin(); it != nbTable.end();)
    {
        auto entry = it->second;
        simtime_t diff = currTime - entry.getLastBeaconReceptionTime();
        if (diff > timeoutTime)
        {
            nbTable.erase(it++);
        }
        else
        {
            ++it;
        }
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------

std::list<NodeIdentifier> Processor::getNeighborIdList(void)
{
    std::list<NodeIdentifier> result;
    for (auto it = nbTable.begin(); it != nbTable.end(); ++it)
    {
        result.push_front(it->first);
    }
    return result;
}

// ------------------------------------------------------------------------------------------------------------------------------------
/*
 * Calculate min collision time
 */
double Processor::getMinCollisionTime(Coord ownVelocity, Coord ownPosition)
{
    double min = MAX;
    for (auto it = nbTable.begin(); it != nbTable.end(); ++it)
    {
        Coord othersPosition = it->second.getLastPosition();
        Coord othersVelocity = it->second.getLastVelocity();
        double time = LaneUtil::estimateCollisionTime(ownVelocity, othersVelocity, ownPosition, othersPosition, minimumSafetyDistance, laneRadiusMeter);
        if (time >= 0 && time < min) {
            EV << it->first << endl;
            min = time;
        }
    }
    EV << "    Processor::getMinCollisionTime: minimum collision time is "<< min << endl;
    return min;
}

// ------------------------------------------------------------------------------------------------------------------------------------
/*
 * start overtake or switching
 */
void Processor::Overtake(Coord velocity)
{
    EV << "Processor::Overtake: start overtaking with speed "<< velocity << endl;
    mobility->setSpeed(velocity.x, KEEP, velocity.z);
    return;
}

// ------------------------------------------------------------------------------------------------------------------------------------
/*
 * finish overtake or switching
 */
void Processor::Overtake()
{
    EV << "Processor::Overtake: finish overtaking " << endl;
    emit(countId, true);
    mobility->move();
    mobility->setSpeed(0, frontDroneSpeed, 0);
    overtakePlan_velocity = Coord();
    overtakePlan_arrivalTime = -1;
    overtakePlan_id = MacAddress();
    overtakePlan_waitCount = 0;
    rb->samplePosition();
    return;
}

// ------------------------------------------------------------------------------------------------------------------------------------

int Processor::getPacketLoss()
{
    int loss = 0;
    for (auto it = nblostTable.begin(); it != nblostTable.end(); ++it)
    {
        auto key = it->first;
        loss += it->second;
        nblostTable[key] = 0;
    }
    return loss;
}

// ------------------------------------------------------------------------------------------------------------------------------------

std::map<NodeIdentifier, NeighborTableEntry>   Processor::getnbTable()
{
    return nbTable;
}


// ------------------------------------------------------------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const Processor& nt)
{
  int oldprec = os.precision();
  os.precision(2);
  os << "Processor[alpha=" << nt.ewmaAlpha
     << ", timeout=" << nt.timeoutTime
     << ", neighbors=";
  for (auto it = nt.nbTable.begin(); it != nt.nbTable.end(); ++it)
    {
      os << it->second;
      if (std::next(it) != nt.nbTable.end())
    os << ", ";
    }
  os << "]";
  os.precision(oldprec);
  return os;
}
