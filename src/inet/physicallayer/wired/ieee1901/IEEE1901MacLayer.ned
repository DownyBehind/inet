package inet.physicallayer.wired.ieee1901;

//
// IEEE 1901 MAC Layer for PLC
// Implements CSMA/CA with priority-based access
//
module IEEE1901MacLayer
{
    parameters:
        @display("i=block/network2");
        
        // MAC parameters
        int cwMin = default(8);
        int cwMax = default(1024);
        int maxRetries = default(3);
        double slotTime @unit(s) = default(20us);
        double sifs @unit(s) = default(10us);
        double difs @unit(s) = default(50us);
        
        // Priority parameters
        int cap0CwMin = default(8);
        int cap0CwMax = default(64);
        int cap1CwMin = default(8);
        int cap1CwMax = default(64);
        int cap2CwMin = default(8);
        int cap2CwMax = default(32);
        int cap3CwMin = default(8);
        int cap3CwMax = default(32);
        
    gates:
        input upperLayerIn @labels(IEEE1901Frame);
        output upperLayerOut @labels(IEEE1901Frame);
        input phyIn @labels(IEEE1901Frame);
        output phyOut @labels(IEEE1901Frame);
}
