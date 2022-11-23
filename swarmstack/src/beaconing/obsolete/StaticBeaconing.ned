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

package swarmstack.beaconing.obsolete;

//
// TODO auto-generated module
//
simple StaticBeaconing like IBeaconingProtocol
{
    parameters:
      @signal[numberOfNeighbors](type=long);
      @signal[beaconConsecutiveCount](type=long);
      @signal[beaconConsecutiveDelay](type=simtime_t;unit=s);
      @statistic[numberOfNeighbors](record=stats;title="Number of neighbors");
      @statistic[beaconConsecutiveCount](record=histogram;title="Gap sizes in received sequence numbers");
      @statistic[beaconConsecutiveDelay](record=histogram;title="Delay between beacon receptions";unit=s);        
               double  nominalPeriod @unit(s)           =  default(0.1s);
      volatile double  periodJitter @unit(s)            =  default(0.01s);
      volatile double  startTime @unit(s)               =  default(exponential(1s));
               double  positionSamplingPeriod @unit(s)  =  default(0.01s);
               int     beaconLength @unit(byte)         =  default(144B);
	gates:
      input  lowerLayerIn @labels(BStaticBeaconingPacket/up);
      output lowerLayerOut @labels(BStaticBeaconingPacket/down);
      output neighborTableOut @labels(BBeaconingProtocolPacket/up);
}
