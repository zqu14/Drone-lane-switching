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

package swarmstack.arrivalCheck;

simple ArrivalChecker
{
	parameters:
	    
	    double constrainZ @unit(m) = default(1000m);
	    double minimumSafetyDistance @unit(m)= default(0.5m);
	    int numOfTarget = default(8000);
	    
	    
	    @signal[simTimeValue](type=double);
		@statistic[simTimeValue](record=mean);	    
		
	    @signal[collisionCount](type=bool);
		@statistic[collisionCount](record=count);
		
		
	    @signal[collisionNode](type=int);
		@statistic[collisionNode](record=vector);
		
		
	    @signal[nodeCount](type=int);
		@statistic[nodeCount](record=vector);

	    @signal[lossCount](type=int);
		@statistic[lossCount](record=vector);		
	    
	    
}