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

package swarmstack.mobility;

import inet.mobility.base.MovingMobilityBase;

simple DroneLinearMobility extends MovingMobilityBase
{
    parameters:
        double initialX @unit(m) = default(uniform(this.constraintAreaMinX, this.constraintAreaMaxX));
        double initialY @unit(m) = default(uniform(this.constraintAreaMinY, this.constraintAreaMaxY));
        double initialZ @unit(m) = default(nanToZero(uniform(this.constraintAreaMinZ, this.constraintAreaMaxZ)));
        bool initFromDisplayString = default(true); // enables one time initialization from the subject module's display string
        double speed @unit(mps) = default(0mps); // speed of the host
        double initialMovementHeading @unit(deg) = default(uniform(0deg, 360deg));
        double initialMovementElevation @unit(deg) = default(0deg);
        double recordPositionPeriod @unit(s) = default(0.1s);

        
        @signal[speed](type=double);
		@statistic[speed](record=vector);
        
        @signal[speedInterval](type=double);
		@statistic[speedInterval](record=vector);
		
		@signal[position](type=double);
		@statistic[position](record=vector);
		
        
        
        
        @class(DroneLinearMobility);
        
        
        
}

