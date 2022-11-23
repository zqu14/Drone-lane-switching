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

#include "inet/common/INETMath.h"
#include "DroneLinearMobility.h"

using namespace inet;

Define_Module(DroneLinearMobility);

simsignal_t speedId = cComponent::registerSignal("speed");
simsignal_t speedIntervalId = cComponent::registerSignal("speedInterval");
simsignal_t positionId = cComponent::registerSignal("position");

DroneLinearMobility::DroneLinearMobility()
{
    speed = 0;
}

void DroneLinearMobility::initialize(int stage)
{
    MovingMobilityBase::initialize(stage);

    EV_TRACE << "initializing LinearMobility stage " << stage << endl;
    if (stage == INITSTAGE_LOCAL) {
        speed = par("speed");
        stationary = (speed == 0);
        rad heading = deg(fmod(par("initialMovementHeading").doubleValue(), 360));
        rad elevation = deg(fmod(par("initialMovementElevation").doubleValue(), 360));
        direction = Quaternion(EulerAngles(heading, -elevation, rad(0))).rotate(Coord::X_AXIS);
        direction.x = 0;
        direction.z = 0;
        direction.y = 1;
        lastVelocity = direction * speed;
        lastUpdate = simTime();
        recordPosition = new cMessage("recordPosition");
        recordPositionPeriod = par("recordPositionPeriod");
        lastSetSpeed = 0;
        //emit(speedId, lastVelocity.length());
        //emit(positionId, distanceToCenter(lastPosition));
        outsideFlag = false;
        scheduleAt(simTime() + recordPositionPeriod, recordPosition);
    }



}

void DroneLinearMobility::handleMessage(cMessage* msg)
{

    if (strcmp(msg->getName(), "move") == 0)
    {
        MovingMobilityBase::handleMessage(msg);
    }

    else if (msg == recordPosition)
    {
        //emit(positionId, distanceToCenter(lastPosition));
        scheduleAt(simTime() + recordPositionPeriod, recordPosition);
    }
    else
    {
        EV << msg->getName() << endl;
        EV << "DroneLinearMobility::handleMessage: got improper message type" << endl;
        endSimulation();
    }
}

double DroneLinearMobility::distanceToCenter(Coord position)
{
    Coord center(constraintAreaMax.x/2, position.y, constraintAreaMax.z/2);
    return position.distance(center);
}


void DroneLinearMobility::move()
{
    double elapsedTime = (simTime() - lastUpdate).dbl();

    if (speed != 0) lastPosition += lastVelocity * elapsedTime;

    if (isOutside()) {
        if (!outsideFlag) {
            double currentTime =  simTime().dbl();
            outsideFlag = true;
        }

        lastPosition.y = constraintAreaMax.y;
    }
}


bool DroneLinearMobility::isArrival()
{
    if (lastPosition.y == constraintAreaMax.y) {
        return true;
    }
    return false;
}

void DroneLinearMobility::setDirection(double e, double h)
{
    direction = Quaternion(EulerAngles(deg(fmod(h,360)), -deg(fmod(e,360)), rad(0))).rotate(Coord::X_AXIS);
    direction.x = 0;
    direction.z = 0;
    direction.y = 1;
    lastVelocity = direction * speed;
}

void DroneLinearMobility::setSpeed(double speed)
{
    lastVelocity = direction * speed;
}

void DroneLinearMobility::setSpeed(double vx, double vy, double vz)
{
    if (vx != KEEP) lastVelocity.x = vx;
    if (vy != KEEP) lastVelocity.y = vy;
    if (vz != KEEP) lastVelocity.z = vz;
    direction.x = lastVelocity.x;
    direction.y = lastVelocity.y;
    direction.z = lastVelocity.z;
    direction.normalize();

    double currentTime =  simTime().dbl();
    double timeDiff = currentTime - lastSetSpeed;

    //emit(speedIntervalId, timeDiff);
    //emit(speedId, lastVelocity.length());
    lastSetSpeed = currentTime;
}

DroneLinearMobility::~DroneLinearMobility()
{
    cancelAndDelete(recordPosition);
}

void DroneLinearMobility::finish()
{
    double currentTime =  simTime().dbl();
    double timeDiff = currentTime - lastSetSpeed;
    //emit(speedIntervalId, timeDiff);
    MovingMobilityBase::finish();
}




