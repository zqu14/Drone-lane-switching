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

package swarmstack.beaconing.renewalbeaconing;

simple NodeGenerator
{
    parameters:
        int maxNumOfNodes;
        volatile double numOfNodesDistribution;
       	double  centerX @unit(m);
       	double  centerZ @unit(m);
        volatile  double  rparval @unit(m);
        volatile  double  thetaparval;
        volatile double random = default(uniform(0,1));
        double radius @unit(m) = default(1m);
        volatile double renewalDistribution @unit(s);
        volatile double maxSpeedDistribution @unit(mps);
        volatile double preferredSpeedPercent;
        
}