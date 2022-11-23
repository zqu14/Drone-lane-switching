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

#include "NodeGenerator.h"
#include "inet/visualizer/scene/NetworkNodeCanvasVisualizer.h"
#include "inet/visualizer/scene/NetworkNodeOsgVisualizer.h"
#include <inet/mobility/contract/IMobility.h>
#include "inet/common/scenario/ScenarioManager.h"
#include <algorithm>
#include <random>

Define_Module(NodeGenerator);


struct nodePosi {
    double x;
    double z;
};


void NodeGenerator::initialize() {
    currentNum = 0;
    centerX = par("centerX");
    centerZ = par("centerZ");
    maxNum = par("maxNumOfNodes");
    startGenerating1 = new cMessage("startGenerating1");
    startGenerating2 = new cMessage("startGenerating2");
    startGenerating3 = new cMessage("startGenerating3");
    startGenerating4 = new cMessage("startGenerating4");
    startGenerating5 = new cMessage("startGenerating5");
    startGenerating6 = new cMessage("startGenerating6");
    startGenerating7 = new cMessage("startGenerating7");
    scheduleAt(simTime()+par("renewalDistribution"), startGenerating1);
    scheduleAt(simTime()+par("renewalDistribution"), startGenerating2);
    scheduleAt(simTime()+par("renewalDistribution"), startGenerating3);
    scheduleAt(simTime()+par("renewalDistribution"), startGenerating4);
    scheduleAt(simTime()+par("renewalDistribution"), startGenerating5);
    scheduleAt(simTime()+par("renewalDistribution"), startGenerating6);
    scheduleAt(simTime()+par("renewalDistribution"), startGenerating7);

    radius = par("radius").doubleValue();

    LastTime =  { 0, 0, 0, 0, 0, 0, 0 };
    LastSpeed =  { 0, 0, 0, 0, 0, 0, 0 };
    failGenerateCount = 0;

}


void NodeGenerator::handleMessage(cMessage* msg) {

    //std::cout << "here handle message  " << simTime() << endl;


    if (msg == startGenerating1) {
        //std::cout << "   message is 1  " << simTime() << endl;
        startGenerate(1);
    } else if (msg == startGenerating2) {
        //std::cout << "   message is 2  " << simTime() << endl;
        startGenerate(2);
    } else if (msg == startGenerating3) {
        //std::cout << "   message is 3  " << simTime() << endl;
        startGenerate(3);
    } else if (msg == startGenerating4) {
        //std::cout << "   message is 4  " << simTime() << endl;
        startGenerate(4);
    } else if (msg == startGenerating5) {
        //std::cout << "   message is 5  " << simTime() << endl;
        startGenerate(5);
    } else if (msg == startGenerating6) {
        //std::cout << "   message is 6  " << simTime() << endl;
        startGenerate(6);
    } else if (msg == startGenerating7) {
        //std::cout << "   message is 7  " << simTime() << endl;
        startGenerate(7);
    } else {
        error("NodeGenerator::handleMessage Unknown message");
    }


    //std::cout << "here end  handle message" << simTime() <<endl;

}

void NodeGenerator::startGenerate(int index) {
    generate(index);
    if (currentNum < maxNum) {


        //std::cout << "       here start plan next generate  " << simTime() << "   "<< index << endl;

        if (index == 1) {
            scheduleAt(simTime()+par("renewalDistribution"), startGenerating1);
        } else if (index == 2) {
            scheduleAt(simTime()+par("renewalDistribution"), startGenerating2);
        } else if (index == 3) {
            scheduleAt(simTime()+par("renewalDistribution"), startGenerating3);
        } else if (index == 4) {
            scheduleAt(simTime()+par("renewalDistribution"), startGenerating4);
        } else if (index == 5) {
            scheduleAt(simTime()+par("renewalDistribution"), startGenerating5);
        } else if (index == 6) {
            scheduleAt(simTime()+par("renewalDistribution"), startGenerating6);
        } else if (index == 7) {
            scheduleAt(simTime()+par("renewalDistribution"), startGenerating7);
        }


        //std::cout << "       here end plan next generate " << simTime() <<endl;

    }
}

void NodeGenerator::generate(int index) {

    //std::cout << "     here start generate  " << simTime() << "   "<< index << endl;



    double numOfNodes = par("numOfNodesDistribution");
    std::vector<int> lanes =  { 1, 2, 3, 4, 5, 6, 7 };

    // generate random seed
    double seed = par("rparval").doubleValue() * 1000;
    std::default_random_engine e(seed);
    std::shuffle(lanes.begin(), lanes.end(), e);


    while (numOfNodes > 0) {
        if (currentNum >= maxNum) return;
        if (numOfNodes < 1) {
            double ran = par("random").doubleValue();
            if (ran >= numOfNodes) return;
        }

        // old generator code, used to generator all drones in one generator
        //int index = lanes.back();
        //lanes.pop_back();


        nodePosi posi = _getLanePosition(index);

        // record the information of generated node to prevent the immediate collision
        double thisLaneLastSpwanTime = LastTime[index-1];
        double thisLaneLastSpwanSpeed = LastSpeed[index-1];
        double maxSpeed = par("maxSpeedDistribution").doubleValue();
        double preferredSpeed = maxSpeed * par("preferredSpeedPercent").doubleValue();

        // estimate the next collision time, if this time is too short, drop this generation
        double lastDronePosition = (simTime().dbl()-thisLaneLastSpwanTime) * thisLaneLastSpwanSpeed;
        double collisionTime = lastDronePosition / (preferredSpeed-thisLaneLastSpwanSpeed);

        //std::cout << "     here end generate  " << simTime() << "   "<< index << endl;

        if ((collisionTime >0 && collisionTime < 1) || (lastDronePosition <= 0.5 && thisLaneLastSpwanSpeed!=0)) {
            //std::cout << "     generate fail " << simTime() << "   "<< index << endl;
            failGenerateCount++;
            numOfNodes--;
            continue;
        } else {
            LastTime[index-1] = simTime().dbl();
            LastSpeed[index-1] = preferredSpeed;
            // test the generator, do not actually generate node
            /*
            currentNum++;
            numOfNodes--;
            continue;
            */
            //std::cout << "lane: " << index << " time: " << LastTime[index-1] << " Total: " << currentNum << endl;

        }
        cModuleType *moduleType = cModuleType::get("swarmstack.beaconing.renewalbeaconing.RenewalBeaconingNodeSafety");
        cModule *mod = moduleType->create("nodes", this->getParentModule(),maxNum,currentNum);
        currentNum++;
        numOfNodes--;
        // set up parameters and gate sizes before we set up its submodules

        mod->par("osgModel") = "3d/drone.ive.5.scale.0,0,90.rot";
        mod->getDisplayString().parse("p=200,100;i=misc/aircraft");
        mod->finalizeParameters();

        // create internals, and schedule it
        mod->buildInside();
        auto *mobility = mod->getSubmodule("mobility");

        // if the mobility parameters have been set in .ini file, do not initialize it
        if (mobility->par("speed").doubleValue() == 0) {
            mobility->par("speed") = preferredSpeed;
            mobility->par("initialX") = posi.x;
            mobility->par("initialY") = 0.0;
            mobility->par("initialZ") = posi.z;
            mod->getSubmodule("rb")->par("preferredSpeed") = preferredSpeed;
            mod->getSubmodule("rb")->par("maxSpeed") = maxSpeed;
        } else {
            if (mobility->par("speed").doubleValue() == -1) mobility->par("speed") = 0.0;
            mod->getSubmodule("rb")->par("preferredSpeed") = mobility->par("speed").doubleValue();
            mod->getSubmodule("rb")->par("maxSpeed") = mobility->par("speed").doubleValue();
        }

        // Send signal to simulator and put the new node into simulation network
        mobility->par("initialMovementHeading") = 90.0;
        inet::cPreModuleInitNotification pre;
        pre.module = mod;
        emit(POST_MODEL_CHANGE, &pre);
        mod->callInitialize();
        inet::cPostModuleInitNotification post;
        post.module = mod;
        emit(POST_MODEL_CHANGE, &post);
        mod->scheduleStart(simTime());
    }
    nodePositions.clear();






}


nodePosi NodeGenerator::getRandomPosition() {
    nodePosi posi;
    do {
        posi = _getRandomPosition();
    } while(!checkPositionConflict(posi));

    nodePositions.push_back(posi);
    return posi;
}

nodePosi NodeGenerator::_getRandomPosition() {
    double r = par("rparval").doubleValue();
    double theta = par("thetaparval").doubleValue();
    nodePosi posi;
    posi.x =  (centerX + r * cos(theta));
    posi.z =  (centerZ + r * sin(theta));
    return posi;
}

/** Only works for two in-tube tiers drone road system model
 *  Used to calculate the coordinate of the initial position of node
 */
nodePosi NodeGenerator::_getLanePosition(int index) {
    nodePosi posi;
    double PI = 3.14159265;
    switch(index){
    case 1:
        posi.x = centerX;
        posi.z = centerZ;
        break;
    case 2:
        posi.x = centerX+radius;
        posi.z = centerZ;
        break;
    case 3:
        posi.x = centerX+radius*cos(60*PI/180);
        posi.z = centerZ+radius*sin(60*PI/180);
        break;
    case 4:
        posi.x = centerX+radius*cos(120*PI/180);
        posi.z = centerZ+radius*sin(120*PI/180);
        break;
    case 5:
        posi.x = centerX+radius*cos(180*PI/180);
        posi.z = centerZ+radius*sin(180*PI/180);
        break;
    case 6:
        posi.x = centerX+radius*cos(240*PI/180);
        posi.z = centerZ+radius*sin(240*PI/180);
        break;
    case 7:
        posi.x = centerX+radius*cos(300*PI/180);
        posi.z = centerZ+radius*sin(300*PI/180);
        break;
    }
    return posi;

}

/**
 *  Deprecated !!!
 *  Used to check the position conflict when generate more than one drone at the same time
 *  Used in drone road model without "lane"
 */
bool NodeGenerator::checkPositionConflict(nodePosi pos) {
    for (auto i : nodePositions)
    {
       double diffx = pow((i.x - pos.x),2);
       double diffz = pow((i.z - pos.z),2);
       EV << diffx+diffz << endl;
       if (diffx+diffz <= 0.25) return false;
    }
    return true;
}


NodeGenerator::NodeGenerator() {
    // TODO Auto-generated constructor stub

}

NodeGenerator::~NodeGenerator() {
    cancelAndDelete(startGenerating1);
    cancelAndDelete(startGenerating2);
    cancelAndDelete(startGenerating3);
    cancelAndDelete(startGenerating4);
    cancelAndDelete(startGenerating5);
    cancelAndDelete(startGenerating6);
    cancelAndDelete(startGenerating7);
}

void NodeGenerator::finish() {
    //std::cout << currentNum << endl;
    //std::cout << failGenerateCount << endl;
}


