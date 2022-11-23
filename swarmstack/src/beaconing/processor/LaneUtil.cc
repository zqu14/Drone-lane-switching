/*
 * LaneUtil.cc
 */


#include "LaneUtil.h"
#include <inet/common/geometry/common/Coord.h>
#include <cmath>


Coord LaneUtil::LaneToCoord(Lane l, double r) {
    Coord c;
    c.x = (l.getI() * r) + (l.getJ() * r / 2);
    c.y = 0;
    c.z = l.getJ() * sqrt(3) * r / 2;
    return c;
}


Lane LaneUtil::coordToLane(Coord c, double r) {
    double j = (c.z-500) / (sqrt(3) * r / 2);
    double i = ((c.x-500) - (j * r / 2)) / r;
    return Lane(round(i), round(j));
}


bool LaneUtil::isSameLane(Lane l1, Lane l2, double r) {

    double x1 = (l1.getI() * r) + (l1.getJ() * r / 2);
    double x2 = (l2.getI() * r) + (l2.getJ() * r / 2);
    double z1 = l1.getJ() * sqrt(3) * r / 2;
    double z2 = l2.getJ() * sqrt(3) * r / 2;

    if (x1==x2 && z1==z2) return true;
    return false;
}


bool LaneUtil::isSameLane(Coord c1, Coord c2, double r) {
    Lane l1 = coordToLane(c1, r);
    Lane l2 = coordToLane(c2, r);
    return isSameLane(l1, l2, r);
}




std::vector<Coord> LaneUtil::getNeighborPosi(Coord currentPosi, double r)
{
    double pi = 3.1415926535;
    std::vector<Coord> neighbors;
    int degree = 0;
    for(int i =1; i<=6;i++) {
        Coord posi = Coord();
        posi.x = currentPosi.x + r * cos(degree*pi/180);
        posi.z = currentPosi.z + r * sin(degree*pi/180);
        posi.y = currentPosi.y;
        degree += 60;
        neighbors.push_back(posi);
    }
    return neighbors;
}


double LaneUtil::_estimateCollisionTime(double myv, double targetv, double myy, double targety, double minsafedis) {
    double t = (targety - myy - minsafedis) / (myv - targetv);
    if (t <= 0) return 0.000001;
    return t;
}


double LaneUtil::estimateCollisionTime(Coord myv, Coord targetv, Coord myp, Coord targetp, double minsafedis, double r) {

    Lane myl = coordToLane(myp, r);
    Lane targetl = coordToLane(targetp, r);

    if (isSameLane(myl, targetl, r) && myp.y < targetp.y && myv.y > targetv.y) {
        return _estimateCollisionTime(myv.y, targetv.y, myp.y, targetp.y, minsafedis);
    }
    return -1;
}


int LaneUtil::getTier(Lane l) {

    int i = l.getI();
    int j = l.getJ();
    if (i==0) return std::abs(j);
    if (j==0) return std::abs(i);
    if ((i>0&&j>0)||(i<0&&j<0)) return std::abs(j)+std::abs(i);
    return std::max(std::abs(j), std::abs(i));

}

std::vector<Lane> LaneUtil::getNeighborLane(Lane l) {

    std::vector<Lane> neighbors;

    int i = l.getI();
    int j = l.getJ();

    neighbors.push_back(Lane(i+1, j));
    neighbors.push_back(Lane(i, j+1));
    neighbors.push_back(Lane(i-1, j));
    neighbors.push_back(Lane(i, j-1));
    neighbors.push_back(Lane(i-1, j+1));
    neighbors.push_back(Lane(i+1, j-1));

    return neighbors;
}



