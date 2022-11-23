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

#ifndef BEACONING_PROCESSOR_PROCESSOR_H_
#define BEACONING_PROCESSOR_PROCESSOR_H_

#include <omnetpp.h>
#include <map>
#include <list>
#include <inet/common/InitStages.h>
#include <inet/linklayer/common/MacAddress.h>
#include <inet/common/geometry/common/Coord.h>
#include <inet/common/geometry/common/EulerAngles.h>
#include "../base/BeaconingBase.h"
#include "../base/Beacon_m.h"
#include "../neighbortable/PacketLossEstimator.h"
#include "../neighbortable/NeighborTable.h"
#include "../base/BeaconReport_m.h"
#include "../../mobility/DroneLinearMobility.h"
#include "../renewalbeaconing/RenewalBeaconing.h"
#include "LaneUtil.h"

using namespace omnetpp;

class Processor : public cSimpleModule {

    protected:
        std::map<NodeIdentifier, NeighborTableEntry>  nbTable;
        std::map<NodeIdentifier, int> nbSeqTable;
        std::map<NodeIdentifier, int> nblostTable;
        double     ewmaAlpha;
        simtime_t  timeoutTime;
        simtime_t  cleanupPeriod;
        cMessage  *pMsgTableCleanup;
        int        gidBeaconsIn;
        NodeIdentifier  ownIdentifier;
        DroneLinearMobility *mobility;
        RenewalBeaconing *rb;
        double enableOvertake;
        double deploymentRadiusMeter;
        double laneRadiusMeter;

        cMessage  *stopOvertake;
        cMessage  *waitingOvertake;
        cMessage  *startPlan;
        cMessage  *checkOvertakeSlow;

        double preferredSpeed;
        double maxSpeed;
        double clTimeThreshold;
        double overtakeSpeed;
        double checkOvertakeSlowPeriod = 0.5;
        double minimumSafetyDistance;
        double waitingPeriod = 0.1;
        double maxNeighborRange;
        double otherLaneSafetyRange;
        double targetLaneSafetyRange;

        double frontDroneSpeed;

        double totalSpeedCost = 0;
        double totalPositionCost = 0;
        double totalCost = 0;
        double totalSwitchCost = 0;


        double overtakePlan_arrivalTime = -1;
        Coord  overtakePlan_velocity;
        MacAddress overtakePlan_id;
        int overtakePlan_waitCount = 0;
        Coord back_lane;
        MacAddress overtakeSlow_id;

        bool isback;
        bool back = false;


        double KAPPA_Speed;
        double KAPPA_Position;
        double KAPPA_Switch;

        const int MAX_R = 5;
        const double MAX = 100000;
        const int MAX_ATTEMPT_TIME = 5;
        const int T_HALF = 7;
        const int TN_HALF = 6;
        const int TN_FULL = 5;
        const int LOOSE_TN_HALF = 4;
        const int T_FULL = 3;
        const int LOOSE_TN_FULL = 2;
        const int ALWAYS_SLOW_DOWN = 1;
        const int DO_NOTHING = 0;
        const int CHECK_OWN_LANE = 3;
        const int CHECK_NEIGHBOR_LANE = 2;
        const int CHECK_TARGET_LANE = 1;
        const int ENABLE_ALGORITHM = 1;
        const int ENABLE_O_ALGORITHM = 2;
        const int NEIGHBOR_LANE_NUM = 5;
    public:
        virtual ~Processor();
        void initialize(int stage);
        virtual int  numInitStages () const override { return inet::NUM_INIT_STAGES; };
        void handleMessage(cMessage* msg);
        MacAddress getPlanId() const {return overtakePlan_id;};

    public:

        int getPacketLoss();

        std::map<NodeIdentifier, NeighborTableEntry>  getnbTable();
        std::map<NodeIdentifier, int> getseqTable() {return nbSeqTable;};
        void finish();
        void updateNbTable(std::list<Nbt> nnbt);
        void checkCollision ();
        void setOvertakeParam(Coord lane);
        void runAlgorithm();
        Coord selectLane(Coord currentPosi);
        bool checkTargetLane(Coord targetLane, Coord originLane);
        bool checkOneLane(Coord lane, int mode);
        std::vector<Coord> getNeighborPosi(Coord currentPosi);
        void slowduringovertake(Coord pos, MacAddress id, Coord vel);
        Processor () { ewmaAlpha = 0.95; timeoutTime = 10.0; };
        Processor (double alpha, double timeout);
        bool checkTargetLaneNocertria(Coord targetLane);
        void printTable();
        bool checkOutSide();
        void moveInTube();
        double estimateCollision(Coord othersPosition, Coord ownPosition, Coord othersVelocity, Coord ownVelocity);
        void Overtake(Coord velocity);
        void Overtake();
        bool updateClTime();
        double getMinCollisionTime(Coord ownVelocity, Coord ownPosition);
        std::vector<double> getSpeeds();
        std::vector<double> getPositionCosts();
        std::vector<double> getSwitchCosts();
        double getEwmaAlpha () const {return ewmaAlpha;};
        void   setEwmaAlpha (double alpha);
        double getTimeoutTime () const {return timeoutTime.dbl();};
        void   setTimeoutTime (simtime_t timeout);
        double getCleanupPeriod () const {return cleanupPeriod.dbl();};
        void   setCleanupPeriod (simtime_t period);

        void recordBeacon(BeaconReport* beaon);
        void cleanupTable (void);

        bool entryAvailable (NodeIdentifier id) {return nbTable.find(id) != nbTable.end();};
        NeighborTableEntry& operator[] (const NodeIdentifier& id) {return nbTable[id];};
        size_t size (void) const {return nbTable.size();};

        std::list<NodeIdentifier> getNeighborIdList (void);
        friend std::ostream& operator<<(std::ostream& os, const Processor& nt);

};

#endif /* BEACONING_PROCESSOR_PROCESSOR_H_ */
