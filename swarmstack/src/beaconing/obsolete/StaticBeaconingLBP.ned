//
// 

package swarmstack.beaconing.obsolete;

simple StaticBeaconingLBP like IBeaconingProtocol
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
