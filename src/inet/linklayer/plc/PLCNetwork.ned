package inet.linklayer.plc;

// Minimal channel definition only, to avoid parser issues during include
channel PLCMedium extends ned.DatarateChannel
{
    parameters:
        datarate = default(100Mbps);
        delay = default(0.1us);
}
