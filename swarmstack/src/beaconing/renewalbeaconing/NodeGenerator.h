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

#ifndef BEACONING_RENEWALBEACONING_NODEGENERATOR_H_
#define BEACONING_RENEWALBEACONING_NODEGENERATOR_H_
#include <omnetpp.h>
#include <vector>
using namespace omnetpp;
class NodeGenerator : public cSimpleModule {

    typedef struct nodePosi nodePosi;
    public:
        void handleMessage(cMessage* msg);
        void initialize();
        void generate(int index);
        void startGenerate(int index);
        NodeGenerator();
        virtual ~NodeGenerator();
        nodePosi getRandomPosition();
        nodePosi _getRandomPosition();
        bool checkPositionConflict(nodePosi pos);
        nodePosi _getLanePosition(int index);
        virtual void finish() override;
    private:
        // parametes used in generation
        std::vector<double> LastTime;
        std::vector<double> LastSpeed;
        int failGenerateCount;
        std::vector<nodePosi> nodePositions;
        double radius;
        int maxNum;
        int currentNum;
        cMessage* startGenerating1;
        cMessage* startGenerating2;
        cMessage* startGenerating3;
        cMessage* startGenerating4;
        cMessage* startGenerating5;
        cMessage* startGenerating6;
        cMessage* startGenerating7;
        double centerX;
        double centerZ;
};

//int NodeGenerator::currentNum = 0;

#endif /* BEACONING_RENEWALBEACONING_NODEGENERATOR_H_ */
