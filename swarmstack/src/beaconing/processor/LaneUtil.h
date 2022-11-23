/*
 * LaneUtil.h
 */

#ifndef BEACONING_PROCESSOR_LANEUTIL_H_
#define BEACONING_PROCESSOR_LANEUTIL_H_

#include <omnetpp.h>
#include <inet/common/geometry/common/Coord.h>



using namespace inet;

/*
 * Structure of the lane
 */

class Lane {

    protected:
        int i;
        int j;

    public:
        int getI(){return i;};
        int getJ(){return j;};
        Lane(){i=-1;j=-1;};
        Lane(int inputi, int inputj){i=inputi;j=inputj;};
        void setI(int inputi){i=inputi;};
        void setJ(int inputj){j=inputj;};

};


/*
 * Include many calculation function used in drone collision avoidance algorithm
 */

class LaneUtil {
    public:
        static Lane coordToLane(Coord c, double r);
        static Coord LaneToCoord(Lane l, double r);
        static double estimateCollisionTime(Coord myv, Coord targetv, Coord myp, Coord targetp, double minsafedis, double r);
        static double _estimateCollisionTime(double myv, double targetv, double myy, double targety, double minsafedis);
        static bool isSameLane(Lane l1, Lane l2, double r);
        static bool isSameLane(Coord c1, Coord c2, double r);
        static std::vector<Coord> getNeighborPosi(Coord currentPosi, double r);
        static int getTier(Lane l);
        static std::vector<Lane> getNeighborLane(Lane l);
};


#endif /* BEACONING_PROCESSOR_LANEUTIL_H_ */
