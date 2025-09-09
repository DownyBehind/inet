package inet.physicallayer.wired.ieee1901;

import inet.physicallayer.wired.common.WireJunction;
import inet.physicallayer.wired.ieee1901.IEEE1901PhyLayer;

//
// IEEE 1901 PLC Interface
// Provides PLC communication over power lines
//
module IEEE1901Interface
{
    parameters:
        @display("i=block/network2");
        
        // Physical layer parameters
        double bitrate @unit(bps) = default(10Mbps);
        double carrierFrequency @unit(Hz) = default(2MHz);
        double txPower @unit(W) = default(1W);
        
        // MAC layer parameters
        int cwMin = default(8);
        int cwMax = default(1024);
        int maxRetries = default(3);
        double slotTime @unit(s) = default(20us);
        double sifs @unit(s) = default(10us);
        double difs @unit(s) = default(50us);
        
        // SLAC parameters
        double slacTimeout @unit(s) = default(2s);
        int slacMaxRetries = default(3);
        double slacRetryBackoff @unit(s) = default(150ms);
        
    gates:
        input in @labels(IEEE1901Frame);
        output out @labels(IEEE1901Frame);
        inout plc @labels(IEEE1901Frame);
        
    submodules:
        phy: IEEE1901PhyLayer {
            @display("p=50,50");
        }
        
    connections:
        in --> phy.upperLayerIn;
        phy.upperLayerOut --> out;
        phy.plc <--> plc;
}
