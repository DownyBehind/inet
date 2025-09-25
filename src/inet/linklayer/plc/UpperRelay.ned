package inet.linklayer.plc;

simple UpperRelay
{
    parameters:
        @class(UpperRelay);
        @display("i=block/app");
    gates:
        input in @labels(PLCFrame);
        output out @labels(PLCFrame);
}


