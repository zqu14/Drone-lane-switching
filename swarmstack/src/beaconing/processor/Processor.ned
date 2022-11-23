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

package swarmstack.beaconing.processor;

simple Processor 
{
    parameters:
        double  tableCleanupPeriod @unit(s)      =  default(0.1s);
        double  tableTimeoutValue @unit(s)       =  default(10.0s);
        double  plrEstimatorAlpha                =  default(0.95);
        double  deploymentRadiusMeter @unit(m)   =  default(5m);
        double  laneRadiusMeter @unit(m)         =  default(1m);
        double  minimumSafetyDistance @unit(m)   =  default(0.5m);
        
        double  maxNeighborRange @unit(m)        =  default(100m);
        double  clTimeThreshold @unit(s)         =  default(1.5s);
        double  overtakeSpeed                    =  default(1);
        double  E1 @unit(m)    =  default(11m);
        double  E2 @unit(m)    =  default(6m);
        
        double K1 = default(1);
        double K2 = default(1);
        double K3 = default(0);
       
        
        
        
        double enableOvertake = default(5);
        volatile int randomTheta                 = default(intuniform(0, 359));
        
        bool isback = default(true);
        
        @signal[overtakeCount](type=bool);
		@statistic[overtakeCount](record=count);
		@signal[perferredSpeedValue](type=double);
		@statistic[perferredSpeedValue](record=mean);
		
		@signal[switchCost](type=double);
		@statistic[switchCost](record=mean);
		
		@signal[speedCost](type=double);
		@statistic[speedCost](record=mean);
		
		@signal[positionCost](type=double);
		@statistic[positionCost](record=mean);
		
		@signal[totalCost](type=double);
		@statistic[totalCost](record=mean);
		
	gates:
	    input  beaconsIn;
}