//
// Author: Emin Ilker Cetinbas (niw3_at_yahoo_d0t_com)
// Copyright (C) 2005 Emin Ilker Cetinbas
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#ifndef MOBILITY_DRONELINEARMOBILITY_H_
#define MOBILITY_DRONELINEARMOBILITY_H_

#include "inet/common/INETDefs.h"
#include "inet/mobility/base/MovingMobilityBase.h"
#include <omnetpp.h>
using namespace omnetpp;
using namespace inet;


#ifndef KEEP
#define KEEP        -9999
#endif

class DroneLinearMobility : public MovingMobilityBase
{
  protected:
    double speed;
    Coord direction;
    cMessage* recordPosition;
    simtime_t recordPositionPeriod;
    double lastSetSpeed;
    bool outsideFlag;


  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void finish() override;

  public:

    virtual double getMaxSpeed() const override { return speed; }
    DroneLinearMobility();
    void setDirection(double e, double h);
    void setSpeed(double speed);
    void setSpeed(double vx, double vy, double vz);
    void handleMessage(cMessage* msg);
    double distanceToCenter(Coord position);
    bool isArrival();
    virtual ~DroneLinearMobility();
    virtual void move() override;
};


#endif /* MOBILITY_DRONELINEARMOBILITY_H_ */

